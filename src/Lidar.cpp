/*
 * Copyright 2026 徐畅
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file Lidar.cpp
 * @brief YDLIDAR X2/X2L 雷达驱动 (移植自 lidar_pkg/src/lidar_node.cpp)
 *
 *  协议要点:
 *    - TTL UART, 150000 波特
 *    - 启动指令: A5 60
 *    - 数据包: AA 55 CT LSN fSa(2) lSa(2) samples(3*LSN) checksum
 *    - 一圈 720 点, 5Hz
 */

#include "Lidar.h"
#include "PinConfig.h"

#include <string.h>

// ── 诊断开关 ─────────────────────────────────────────────
// 1 = 开机后先扫一遗候选波特率（约 11s）。
// 实测这台雷达在 150000 上 6B9377 命中最多 → 已确认，正常使用改回 0。
#define LIDAR_BAUD_SWEEP   0

// 1 = 开机自动识别「波特率 + 帧头顺序」（约 0.3~1.8s）
// 现场两台同外观雷达实测不一样：
//   A: 150000 baud, 帧头 AA 55
//   B: 115200 baud, 帧头 55 AA   ← 之前一直收不到数据的真正原因
#define LIDAR_AUTODETECT   1

// ────────────────────────── 内部变量 ──────────────────────────
// 使用 UART1 (不占用 UART0/USB-CDC 调试口)
static HardwareSerial LidarSerial(1);

// ── 单包最大样点数 ─────────────────────────────────────────
// 原值按标准 X2 的 LSN<=32 硬卡；但同系列雷达不同批次/固件会发更大的 LSN
// (36/40/48...)。上位机版用的是 std::vector，没有这个上限 —— 于是就会出现
// "同一个驱动、主机能出数据，ESP32 一包都进不来"的现象。
// 现在只按缓冲大小设限，不再按 32 卡死。
#define LIDAR_MAX_LSN           128
#define LIDAR_PACKET_BUF_SIZE   (8 + 3 * LIDAR_MAX_LSN + 16)   // 408B

// 解析状态机
enum LidarState {
    WAIT_HEADER1,
    WAIT_HEADER2,
    READ_META,
    READ_PAYLOAD
};
static LidarState state_ = WAIT_HEADER1;
static uint8_t packet_buffer_[LIDAR_PACKET_BUF_SIZE];
static uint16_t packet_len_ = 0;
static uint8_t current_lsn_ = 0;
static uint16_t target_payload_size_ = 0;

// ── 帧头字节对 ───────────────────────────────────────────
// YDLIDAR X2/X4 用 "AA 55"；另一批同外观雷达用 "55 AA"。
// 两种雷达帧头之后的字段位置完全一样，所以只要把这 2 个字节认对，
// 后面所有解析代码都不用改。开机时由 autoDetectLidar() 填。
static uint8_t  hdr_first_  = 0xAA;
static uint8_t  hdr_second_ = 0x55;

// "55 AA" 协议（雷达 B）的上一个包起始角原始值，用来算每包跨度。
// 该协议角度量纲实测：64 单位/度，模 23040 (= 360*64)，转速 5.16Hz。
// -1 = 还没收到过包。
static int32_t last_raw_angle_ = -1;
#define LIDAR_B_ANGLE_UNITS_PER_DEG   64.0f
#define LIDAR_B_ANGLE_MODULES         23040

// "55 AA" 协议（雷达 B）的距离量纲：1 单位 = 1mm。
// 实测依据（LidarTest 的 `m` 探针）：整圈 63% 的样点落在原始值 256~1023，
//   按 /4 换算只有 6.4~25.6cm —— 比雷达盲区还近，不可能；
//   按 /1 换算是 0.26~1.02m，正常室内距离。
// 且原始值 16384(0x4000) / 32768(0x8000) 成片出现（强度字节为 0），
// 是"无回波/超量程"哨兵：按 /1 对应 16.4m / 32.8m，已经是量程外。
#define LIDAR_B_DIST_MM_PER_UNIT      1.0f
#define LIDAR_B_DIST_SENTINEL         0x4000

// 扫描缓冲（双缓冲：正在填充 / 就绪待发布）
static float scan_ranges_[LIDAR_SCAN_SIZE];        // 正在填充
static float scan_ready_ranges_[LIDAR_SCAN_SIZE];  // 就绪待发布
static volatile bool scan_ready_ = false;
static uint32_t scan_timestamp_ms_ = 0;

// 圈检测
static float last_point_angle_ = 0.0f;
static uint32_t scan_start_ms_ = 0;
static bool first_packet_of_scan_ = true;
static int scan_count_ = 0;

// 调试计数器
static uint32_t dbg_bytes_rx = 0;      // UART 收到的字节数
static uint32_t dbg_packets_ok = 0;    // 解析成功的数据包数
static uint32_t dbg_packets_bad = 0;   // LSN 超缓冲上限被丢弃的包数
static uint32_t dbg_aa_seen = 0;       // 原始流里 0xAA 出现次数
static uint32_t dbg_sync_aa55 = 0;     // 原始流里 "AA 55" 同步头出现次数
static uint32_t dbg_sync_55aa = 0;     // 原始流里 "55 AA" 同步头出现次数
static uint32_t dbg_lsn_hist[LIDAR_MAX_LSN + 1] = {0};  // LSN 分布
static uint8_t  dbg_lsn_max = 0;       // 见过的最大 LSN
static uint32_t dbg_last_print = 0;

// ── 原始字节诊断（只在一个包都没解析出来时采样）──────────
// 用来区分：全 0x00/全 0xFF = 线路被拉死；均匀分布 = 噪声/悬空；
//           只有几个值占绝对多数 = 波特率错位；能看到 6B 93 77 = 不是 AA55 协议
static uint32_t dbg_hist[256] = {0};   // 字节值分布
static uint8_t  dbg_first[64];         // 开机后收到的前 64 字节
static uint8_t  dbg_first_n   = 0;
static uint32_t dbg_raw_total = 0;     // 已采样的原始字节数
static bool     dbg_first_printed = false;

// ── 6B 93 77 帧结构捕获（另一套协议的雷达）────────────────
// 用“相邻两个头之间隔了多少字节”量出帧长，并抓完整一帧下来反推协议
static uint8_t  raw_p2 = 0, raw_p1 = 0;   // 前两个字节
static uint32_t raw_pos = 0;             // 距上一个头的字节数
static uint32_t raw_gap[256] = {0};      // 帧长分布（>255 归到 255）
static uint32_t raw_hdr_n = 0;
static uint8_t  raw_frame[96];
static uint8_t  raw_frame_n = 0;
static int16_t  raw_capture = -1;        // >0 = 正在抓帧
static bool     raw_frame_printed = false;
static bool     raw_frame_ready   = false;

// ── 启动指令重发看门狗 ────────────────────────────────────
// 一直解析不出包 → 每 2 秒重发一次启动指令，A5 60(扫描) 与 A5 61(强制扫描) 交替
#define LIDAR_START_RETRY_MAX   5
static uint8_t  start_retry_n  = 0;
static uint32_t start_retry_ms = 0;

static const float ANGLE_INCREMENT = (2.0f * PI) / LIDAR_SCAN_SIZE;

// ────────────────────────── 内部函数 ──────────────────────────

static inline uint16_t bytesToUint16(const uint8_t *data, size_t index)
{
    return (uint16_t)((uint16_t)(data[index + 1]) << 8) | data[index];
}

#if LIDAR_FILTER_ENABLE
/**
 * 半径离群滤波：就地剔除孤立噪点
 * 与树莓派版 lidar_node.cpp 的 removeOutliers() 等价，只是直接在格索引上做：
 * 某点在 radius 内的邻居（含自身）少于 min_neighbors 个 → 判为噪点。
 * 一圈 200~360 点，暴力两两比较约 6~13 万次，5Hz 下 CPU 占用 <2%。
 */
static void removeOutliers()
{
    // 先把有效格转成笛卡尔坐标，避免 N² 循环里反复算 sin/cos
    static float   xs[LIDAR_SCAN_SIZE];
    static float   ys[LIDAR_SCAN_SIZE];
    static int16_t bin[LIDAR_SCAN_SIZE];

    int n = 0;
    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        float d = scan_ranges_[i];
        if (!isfinite(d)) continue;
        float a = (float)i * ANGLE_INCREMENT;
        xs[n] = d * cosf(a);
        ys[n] = d * sinf(a);
        bin[n] = (int16_t)i;
        n++;
    }

    const float r2 = LIDAR_FILTER_RADIUS_M * LIDAR_FILTER_RADIUS_M;

    for (int i = 0; i < n; i++) {
        int neighbors = 1;                     // 自己算一个
        for (int j = 0; j < n; j++) {
            if (j == i) continue;
            float dx = xs[i] - xs[j];
            float dy = ys[i] - ys[j];
            if (dx * dx + dy * dy < r2 && ++neighbors >= LIDAR_FILTER_MIN_NEIGHBORS) break;
        }
        if (neighbors < LIDAR_FILTER_MIN_NEIGHBORS) {
            scan_ranges_[bin[i]] = INFINITY;   // 孤立点 → 丢弃
        }
    }
}
#endif // LIDAR_FILTER_ENABLE

/** 一圈结束：滤波 → 拷到就绪缓冲，供 micro-ROS 侧取用 */
static void finishScan()
{
    int valid = 0;
    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        if (isfinite(scan_ranges_[i])) valid++;
    }

    // 有效点太少 → 这一圈不可信，不要拿它去污染地图
    if (valid < LIDAR_MIN_VALID_POINTS) {
        // Serial.printf("[Lidar] 本圈仅 %d 个有效点，丢弃\n", valid);
        return;
    }

#if LIDAR_FILTER_ENABLE
    removeOutliers();
#endif

    memcpy(scan_ready_ranges_, scan_ranges_, sizeof(scan_ready_ranges_));
    scan_timestamp_ms_ = scan_start_ms_;
    scan_ready_ = true;
}

/** 解析一帧数据包，写入 720 点扫描缓冲 */
static void parsePacket(const uint8_t *packet_data)
{
    if (first_packet_of_scan_) {
        scan_start_ms_ = millis();
        first_packet_of_scan_ = false;
    }

    uint8_t lsn = packet_data[3];
    if (lsn == 0) return;
    dbg_packets_ok++;

    // 第一包原样 hexdump 一次（诊断协议结构：看 LSN/角度字段是否合理）
    if (dbg_packets_ok == 1) {
        int plen = 8 + 3 * (int)lsn;
        // Serial.printf("[Lidar] 首包 LSN=%u 共%dB: ", lsn, plen);
        // for (int i = 0; i < plen && i < 48; i++) Serial.printf("%02X ", packet_data[i]);
        Serial.println(plen > 48 ? " ..." : "");
    }

    // ── 角度字段：两种雷达不一样 ───────────────────────
    // "AA 55" 协议 (YDLIDAR X2/X4)：FSA 在 [4..5]，LSA 在 [6..7]，角度 = (raw>>1)/64
    // "55 AA" 协议 (雷达 B)：[4..5] 是另一个缓慢变化的字段（和角度无关），
    //         角度在 [6..7]，量纲 64 单位/度，模 23040 (= 360*64)。
    //         实测证据：序列从 0xF8B6(63670) 回绕到 0xA084(41092)，
    //         得模 = 63670 + 456 - 41092 = 23034 ≈ 23040，对应 5.16Hz。
    const bool ydlidar = (hdr_first_ == 0xAA);

    float angle_start_deg = 0.0f;
    float diff_angle_deg  = 0.0f;

    if (ydlidar) {
        uint16_t fsangle_raw = bytesToUint16(packet_data, 4);
        uint16_t lsangle_raw = bytesToUint16(packet_data, 6);
        angle_start_deg = (float)(fsangle_raw >> 1) / 64.0f;
        float angle_end_deg = (float)(lsangle_raw >> 1) / 64.0f;
        if (lsn > 1) {
            diff_angle_deg = angle_end_deg - angle_start_deg;
            if (diff_angle_deg < 0.0f) diff_angle_deg += 360.0f;
        }
    } else {
        // 雷达 B：包里只给起始角，每包跨度从相邻两包起始角的差推算（自标定）
        int32_t ang_raw = (int32_t)bytesToUint16(packet_data, 6);
        angle_start_deg = fmodf((float)ang_raw / LIDAR_B_ANGLE_UNITS_PER_DEG, 360.0f);
        if (last_raw_angle_ >= 0) {
            int32_t d = ang_raw - last_raw_angle_;
            if (d < 0) d += LIDAR_B_ANGLE_MODULES;          // 回绕
            if (d > 0 && d < LIDAR_B_ANGLE_MODULES / 10) {  // 只采纳合理跨度 (< 36°)
                diff_angle_deg = (float)d / LIDAR_B_ANGLE_UNITS_PER_DEG;
            }
        }
        last_raw_angle_ = ang_raw;
        if (diff_angle_deg <= 0.0f) {
            diff_angle_deg = 456.0f / LIDAR_B_ANGLE_UNITS_PER_DEG;   // 兜底 7.125°
        }
    }

    for (int i = 0; i < lsn; ++i) {
        size_t offset = 8 + (size_t)i * 3;
        if (offset + 1 >= 8 + (size_t)lsn * 3) break;   // 越界保护

        uint16_t dist_raw = bytesToUint16(packet_data, offset);

        // ── 距离量纲：两种雷达不一样 ──────────────────────
        // "AA 55" (YDLIDAR)：16 位里低 2 位是标志，距离 = raw/4 (mm)
        // "55 AA" (雷达 B)：1 单位 = 1mm；raw >= 0x4000 是哨兵，判为无效
        bool  dist_invalid = false;
        float distance_mm;
        if (ydlidar) {
            distance_mm = (float)dist_raw / 4.0f;
        } else if (dist_raw >= LIDAR_B_DIST_SENTINEL) {
            dist_invalid = true;
            distance_mm  = 0.0f;
        } else {
            distance_mm = (float)dist_raw * LIDAR_B_DIST_MM_PER_UNIT;
        }
        float distance_m = distance_mm / 1000.0f;

        float angle_deg = angle_start_deg;
        if (lsn > 1) {
            angle_deg = (diff_angle_deg / (float)(lsn - 1)) * i + angle_start_deg;
        }

        // X2 特有的角度修正公式（雷达 B 那套协议没有这个修正，必须跳过）
        float angle_correct_deg = 0.0f;
        if (ydlidar && distance_mm != 0.0f) {
            float numerator   = 21.8f * (155.3f - distance_mm);
            float denominator = 155.3f * distance_mm;
            float angle_correct_rad = atanf(numerator / denominator);
            angle_correct_deg = angle_correct_rad * 180.0f / PI;
        }

        float final_angle_deg = fmodf(angle_deg + angle_correct_deg, 360.0f);
        if (final_angle_deg < 0.0f) final_angle_deg += 360.0f;
        float angle_rad = final_angle_deg * PI / 180.0f;

        // ── 过零检测（一圈结束）─────────────────────────────
        // 只看角度、不看距离是否有效：整片区域测不到点时也能正常收圈
        if (angle_rad < last_point_angle_ - PI) {
            if (scan_count_ > 0) {
                finishScan();   // 结算上一圈（滤波后拷到就绪缓冲）
            }
            scan_count_++;
            // 重置缓冲，开始新一圈
            for (int k = 0; k < LIDAR_SCAN_SIZE; k++) scan_ranges_[k] = INFINITY;
            first_packet_of_scan_ = true;
            scan_start_ms_ = millis();
        }
        last_point_angle_ = angle_rad;

        // ── 量程窗口：盲区内 / 超量程 / 哨兵，直接丢 ──
        if (dist_invalid ||
            distance_m < LIDAR_RANGE_MIN_M || distance_m > LIDAR_RANGE_MAX_M) continue;

        // ── 映射到格索引，格内取最近值（与 lidar_node.cpp 一致）──
        float index_angle = 2.0f * PI - angle_rad;
        if (index_angle >= 2.0f * PI) index_angle -= 2.0f * PI;
        if (index_angle < 0.0f)       index_angle += 2.0f * PI;

        int index = (int)roundf(index_angle / ANGLE_INCREMENT);
        if (index >= LIDAR_SCAN_SIZE) index = 0;
        if (index >= 0 && index < LIDAR_SCAN_SIZE) {
            if (scan_ranges_[index] == INFINITY || distance_m < scan_ranges_[index]) {
                scan_ranges_[index] = distance_m;
            }
        }
    }
}

/** 逐字节喂入状态机 */
static void processByte(uint8_t byte)
{
    switch (state_) {
    case WAIT_HEADER1:
        if (byte == hdr_first_) {
            if (byte == 0xAA) dbg_aa_seen++;
            state_ = WAIT_HEADER2;
            packet_len_ = 0;
            packet_buffer_[packet_len_++] = byte;
        }
        break;

    case WAIT_HEADER2:
        if (byte == hdr_second_) {
            if (hdr_first_ == 0xAA) dbg_sync_aa55++; else dbg_sync_55aa++;
            state_ = READ_META;
            packet_buffer_[packet_len_++] = byte;
        } else if (byte == hdr_first_) {
            if (byte == 0xAA) dbg_aa_seen++;
            packet_len_ = 0;
            packet_buffer_[packet_len_++] = byte;   // 连续同头字节，重新计数
        } else {
            state_ = WAIT_HEADER1;
            packet_len_ = 0;
        }
        break;

    case READ_META:
        packet_buffer_[packet_len_++] = byte;
        if (packet_len_ == 4) {
            current_lsn_ = packet_buffer_[3];
            if (current_lsn_ > dbg_lsn_max) dbg_lsn_max = current_lsn_;
            if (current_lsn_ <= LIDAR_MAX_LSN) dbg_lsn_hist[current_lsn_]++;
            if (current_lsn_ > LIDAR_MAX_LSN) {   // 只受缓冲上限限制（原为 >32）
                dbg_packets_bad++;
                state_ = WAIT_HEADER1;
                packet_len_ = 0;
                break;
            }
            target_payload_size_ = 4 + (uint16_t)current_lsn_ * 3;
            state_ = READ_PAYLOAD;
        }
        break;

    case READ_PAYLOAD:
        packet_buffer_[packet_len_++] = byte;
        if (packet_len_ == 4 + target_payload_size_) {
            parsePacket(packet_buffer_);
            state_ = WAIT_HEADER1;
            packet_len_ = 0;
        }
        break;
    }
}

/** 把已采样的原始字节值排出前 6 名（判断是不是波特率错位 / 线路拉死）*/
static void printByteHistogram()
{
    static bool used[256];
    memset(used, 0, sizeof(used));
    for (int k = 0; k < 6; k++) {
        int      best   = -1;
        uint32_t best_n = 0;
        for (int v = 0; v < 256; v++) {
            if (!used[v] && dbg_hist[v] > best_n) { best_n = dbg_hist[v]; best = v; }
        }
        if (best < 0 || best_n == 0) break;
        used[best] = true;
        // Serial.printf("[Lidar RAW]   0x%02X x%-8lu %.1f%%\n",
                      // best, (unsigned long)best_n,
                      // 100.0f * (float)best_n / (float)dbg_raw_total);
    }
}

/** 帧结构捕获：检测 6B 93 77 10 00 1C 同步头，抓到内存后统一打印（协议反推用） */
/** 独立解析任务：持续读串口、解析，不受主循环阻塞影响 */
static void lidarTask(void *arg)
{
    while (true) {
        while (LidarSerial.available() > 0) {
            uint8_t b = (uint8_t)LidarSerial.read();

            // 还没同步上 → 顺便采样原始字节（同步成功后自动停止采样）
            if (dbg_packets_ok == 0 && dbg_raw_total < 200000) {
                if (dbg_first_n < sizeof(dbg_first)) dbg_first[dbg_first_n++] = b;
                dbg_hist[b]++;
                dbg_raw_total++;

                // 6B 93 77 头：量帧长 + 抓完整一帧
                if (raw_p2 == 0x6B && raw_p1 == 0x93 && b == 0x77) {
                    if (raw_hdr_n > 0) raw_gap[(raw_pos + 1) < 255 ? (raw_pos + 1) : 255]++;
                    raw_hdr_n++;
                    raw_pos = 0;
                    if (!raw_frame_printed && raw_capture <= 0) {
                        raw_frame_n = 0;
                        raw_capture = (int16_t)sizeof(raw_frame);
                    }
                } else if (raw_pos < 1000000) {
                    raw_pos++;
                }

                if (raw_capture > 0) {
                    if (raw_frame_n < sizeof(raw_frame)) raw_frame[raw_frame_n++] = b;
                    if (--raw_capture == 0) { raw_frame_printed = true; raw_frame_ready = true; }
                }
                raw_p2 = raw_p1;
                raw_p1 = b;
            }

            processByte(b);
            dbg_bytes_rx++;
        }

        // 一直没解析出包 → 重发启动指令（有些雷达对启动时序/命令很挑）
        if (dbg_packets_ok == 0 && hdr_first_ == 0xAA &&
            start_retry_n < LIDAR_START_RETRY_MAX &&
            (uint32_t)(millis() - start_retry_ms) > 2000) {
            start_retry_ms = millis();
            start_retry_n++;
            uint8_t cmd[2] = {0xA5, (uint8_t)((start_retry_n & 1) ? 0x60 : 0x61)};
            LidarSerial.write(cmd, sizeof(cmd));
            // Serial.printf("[Lidar] 还没收到数据 → 重发启动指令 A5 %02X (第%u/%u次)\n",
                          // cmd[1], (unsigned)start_retry_n, (unsigned)LIDAR_START_RETRY_MAX);
        }

        // 每 5 秒打印一次诊断信息
        if (millis() - dbg_last_print > 5000) {
            dbg_last_print = millis();
            uint32_t valid = 0;
            for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
                if (scan_ready_ranges_[i] != INFINITY) valid++;
            }

            // LSN 分布里出现最多的那个值 → 雷达实际每包几个点
            uint8_t  lsn_mode = 0;
            uint32_t lsn_mode_n = 0;
            for (int i = 1; i <= LIDAR_MAX_LSN; i++) {
                if (dbg_lsn_hist[i] > lsn_mode_n) {
                    lsn_mode_n = dbg_lsn_hist[i];
                    lsn_mode = (uint8_t)i;
                }
            }

            // Serial.printf("[Lidar DBG] RX=%luB 头=%02X%02X AA55=%lu 55AA=%lu 包=%lu 丢=%lu 圈=%d 有效点=%lu 就绪=%d LSN=常见%u(x%lu) 最大%u\n",
                // (unsigned long)dbg_bytes_rx, hdr_first_, hdr_second_,
                // (unsigned long)dbg_sync_aa55, (unsigned long)dbg_sync_55aa,
                // (unsigned long)dbg_packets_ok, (unsigned long)dbg_packets_bad,
                // scan_count_, (unsigned long)valid, scan_ready_ ? 1 : 0,
                // lsn_mode, (unsigned long)lsn_mode_n, dbg_lsn_max);

            // 一个包都没解析出来 → 把原始字节摊开看（同步成功后不再打印）
            if (dbg_packets_ok == 0 && dbg_raw_total > 0) {
                // 头 64 字节 + 字节分布：只打一次
                if (!dbg_first_printed) {
                    dbg_first_printed = true;
                    // Serial.printf("[Lidar RAW] 前 %u 字节:", (unsigned)dbg_first_n);
                    // for (uint8_t i = 0; i < dbg_first_n; i++) Serial.printf(" %02X", dbg_first[i]);
                    Serial.println();
                    printByteHistogram();
                }

                // 6B 93 77 头命中次数 + 帧长分布（取前 4 名）
                if (raw_hdr_n > 0) {
                    static bool gused[256];
                    memset(gused, 0, sizeof(gused));
                    for (int k = 0; k < 4; k++) {
                        int      best   = -1;
                        uint32_t best_n = 0;
                        for (int v = 1; v < 256; v++) {
                            if (!gused[v] && raw_gap[v] > best_n) { best_n = raw_gap[v]; best = v; }
                        }
                        if (best < 0 || best_n == 0) break;
                        gused[best] = true;
                        // Serial.printf("[Lidar RAW] 6B9377 x%lu  帧长 %d 字节 x%lu\n",
                                      // (unsigned long)raw_hdr_n, best, (unsigned long)best_n);
                    }
                }

                // 抓到的完整一帧（只打一次，用来反推协议）
                if (raw_frame_ready) {
                    raw_frame_ready = false;
                    // Serial.printf("[Lidar RAW] 完整一帧 %u 字节:\n", (unsigned)raw_frame_n);
                    for (uint8_t i = 0; i < raw_frame_n; i++) {
                        // Serial.printf("%02X ", raw_frame[i]);
                        if ((i & 15) == 15) Serial.println();
                    }
                    Serial.println();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));   // 让出 CPU
    }
}

/** 波特率扫描：找出雷达实际波特率（诊断用，换雷达型号时跑一次） */
static void lidarBaudSweep()
{
    const uint32_t bauds[] = {115200, 150000, 230400, 256000, 100000, 460800, 128000};
    Serial.println("[Lidar] === 波特率扫描开始 ===");
    for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        LidarSerial.begin(bauds[i], SERIAL_8N1, LIDAR_RX, LIDAR_TX);
        delay(120);
        LidarSerial.flush();
        uint32_t count = 0, aa55 = 0, sync6b93 = 0;
        uint8_t b0 = 0, b1 = 0;
        uint32_t start = millis();
        while (millis() - start < 1500) {
            while (LidarSerial.available() > 0) {
                uint8_t b = (uint8_t)LidarSerial.read();
                count++;
                if (b0 == 0xAA && b == 0x55) aa55++;                 // X2 协议头
                if (b1 == 0x6B && b0 == 0x93 && b == 0x77) sync6b93++; // 新雷达出现的头
                b1 = b0; b0 = b;
            }
        }
        // Serial.printf("[Baud] %lu: RX=%lu  AA55=%lu  6B9377=%lu\n",
            // (unsigned long)bauds[i], (unsigned long)count,
            // (unsigned long)aa55, (unsigned long)sync6b93);
    }
    Serial.println("[Lidar] === 波特率扫描结束 ===");
}

// ══════════════════════ 对外接口 ══════════════════════

/**
 * 开机自动识别雷达配置（波特率 + 帧头顺序）。
 *
 * 现实情况：现场有两台外观相同的雷达，实测参数完全不同：
 *   雷达 A：150000 baud，帧头 "AA 55"（YDLIDAR X2/X4）
 *   雷达 B：115200 baud，帧头 "55 AA"
 * 「雷达 B 在 ESP32 上一包都解不出来」的真正原因就是帧头顺序认反了：
 * 字节率和帧头出现频率（各 272 次/秒）从头到尾都是对的，只是状态机在等
 * "AA 55"，而线上每包发的是 "55 AA"，永远进不了 READ_META。
 *
 * 两种雷达帧头之后的字段位置完全一样（CT、LSN、FSA、LSA、3*LSN 样点），
 * 所以只要认对波特率和帧头顺序，后面所有解析代码都不用改。
 *
 * 做法：按候选波特率依次开 WINDOW_MS 窗口，统计 "AA 55" 和 "55 AA" 各命中几次，
 *       命中多的那种就是它的帧头；命中数达阈值就提前结束。
 */
static void autoDetectLidar()
{
    static const uint32_t kCandidates[] = {150000, 115200, 128000, 100000, 230400, 256000};
    const uint32_t HIT_THRESHOLD = 20;    // 正常 260~270 包/秒，300ms 窗口约有 80 次
    const uint32_t WINDOW_MS     = 300;

    uint32_t best_baud  = LIDAR_BAUDRATE;
    uint32_t best_hits  = 0;
    bool     best_swap  = false;          // false = AA 55, true = 55 AA
    bool     found      = false;

    Serial.println("[Lidar] 自动识别波特率 / 帧头 ...");

    for (size_t i = 0; i < sizeof(kCandidates) / sizeof(kCandidates[0]); i++) {
        LidarSerial.begin(kCandidates[i], SERIAL_8N1, LIDAR_RX, LIDAR_TX);
        delay(20);
        LidarSerial.flush();
        while (LidarSerial.available() > 0) (void)LidarSerial.read();

        uint32_t n_aa55 = 0, n_55aa = 0;
        uint8_t  prev = 0;
        uint32_t t0 = millis();
        while (millis() - t0 < WINDOW_MS) {
            while (LidarSerial.available() > 0) {
                uint8_t b = (uint8_t)LidarSerial.read();
                if (prev == 0xAA && b == 0x55) n_aa55++;
                if (prev == 0x55 && b == 0xAA) n_55aa++;
                prev = b;
            }
        }

        uint32_t hits = n_aa55 + n_55aa;
        // Serial.printf("[Lidar]   %6lu baud: AA55=%lu  55AA=%lu\n",
                      // (unsigned long)kCandidates[i],
                      // (unsigned long)n_aa55, (unsigned long)n_55aa);

        if (hits > best_hits) {
            best_hits = hits;
            best_baud = kCandidates[i];
            best_swap = (n_55aa > n_aa55);
        }
        if (hits >= HIT_THRESHOLD) { found = true; break; }
    }

    hdr_first_  = best_swap ? 0x55 : 0xAA;
    hdr_second_ = best_swap ? 0xAA : 0x55;

    LidarSerial.begin(best_baud, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
    delay(20);
    LidarSerial.flush();

    // Serial.printf("[Lidar] 识别结果: %lu baud, 帧头 %02X %02X%s\n",
                  // (unsigned long)best_baud, hdr_first_, hdr_second_,
                  // found ? "" : "  (⚠ 没抓到有效帧头，用的是兜底值)");
}

void Lidar_Init()
{
    // 增大接收缓冲，防止主循环忙时丢字节（必须在第一次 begin() 之前）
    LidarSerial.setRxBufferSize(2048);

    // 清空缓冲
    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        scan_ranges_[i]      = INFINITY;
        scan_ready_ranges_[i] = INFINITY;
    }

#if LIDAR_AUTODETECT
    autoDetectLidar();
#else
    LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
    delay(100);
    LidarSerial.flush();
#endif

    // ⚠️ 不要发停止序列 A5 00 A5 65 A5 65！
    //    A5 65 = 停止扫描，而雷达上电本来就是自动扫描的。在正确波特率下发它
    //    会把雷达停掉（现象：线上全低、字节流几乎全是 0x00、一个 AA 都看不到）。
    //    主机版 lidar_node.cpp 只在**退出时**发停止序列，启动时只发 A5 60。

    // 启动指令 A5 60（雷达本来就在扫描，这条只是保底）。
    // 只对 "AA 55" 协议的 YDLIDAR 发：另一套协议的雷达不认这个命令，发了只是线上噪声。
    uint8_t start_cmd[] = {0xA5, 0x60};
    if (hdr_first_ == 0xAA) LidarSerial.write(start_cmd, sizeof(start_cmd));
    delay(50);
    LidarSerial.flush();

    // X2 收到 A5 60 会回 "A5 5A ..." 应答。看得到 = 命令生效；
    // 看不到 = 命令没被雷达认下来（电平/时序/方向问题），这是关键证据。
    uint8_t  ack[40];
    uint8_t  ack_n    = 0;
    uint8_t  ack_p1   = 0;
    uint8_t  ack_hits = 0;
    uint32_t ack_t0   = millis();
    while (millis() - ack_t0 < 300) {
        while (LidarSerial.available() > 0 && ack_n < sizeof(ack)) {
            uint8_t b = (uint8_t)LidarSerial.read();
            ack[ack_n++] = b;
            if (ack_p1 == 0xA5 && b == 0x5A) ack_hits++;
            ack_p1 = b;
        }
    }
    // Serial.printf("[Lidar] 启动后 300ms 收到 %u 字节, A5-5A 应答 %u 次: ",
                  // (unsigned)ack_n, (unsigned)ack_hits);
    // for (uint8_t i = 0; i < ack_n && i < 16; i++) Serial.printf("%02X ", ack[i]);
    Serial.println();
    if (ack_hits == 0) {
        Serial.println("[Lidar] ⚠ 没看到应答：启动命令可能没送到雷达（后续会自动重发）");
    }

#if LIDAR_BAUD_SWEEP
    // 诊断：扫一遗候选波特率，找雷达真正的速率（约 11 秒）
    lidarBaudSweep();
    // 扫描会把串口停在最后一个波特率上，恢复成工作波特率
    LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
    delay(50);
    LidarSerial.flush();
#endif

    // 创建独立解析任务（栈 4096，优先级 2）
    xTaskCreate(lidarTask, "lidar_task", 4096, NULL, 2, NULL);

    // Serial.printf("[Lidar] init done, UART1 %d baud (RX=%d TX=%d)\n",
        // LIDAR_BAUDRATE, LIDAR_RX, LIDAR_TX);
}

bool Lidar_isScanReady()
{
    return scan_ready_;
}

void Lidar_getScan(float *ranges, uint32_t *timestamp_ms)
{
    memcpy(ranges, scan_ready_ranges_, sizeof(scan_ready_ranges_));
    if (timestamp_ms) *timestamp_ms = scan_timestamp_ms_;
    scan_ready_ = false;
}
