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

// ────────────────────────── 内部变量 ──────────────────────────
// 使用 UART1 (不占用 UART0/USB-CDC 调试口)
static HardwareSerial LidarSerial(1);

// 解析状态机
enum LidarState {
    WAIT_HEADER1,
    WAIT_HEADER2,
    READ_META,
    READ_PAYLOAD
};
static LidarState state_ = WAIT_HEADER1;
static uint8_t packet_buffer_[160];      // 最大包 8 + 3*32 = 104 字节
static uint16_t packet_len_ = 0;
static uint8_t current_lsn_ = 0;
static uint16_t target_payload_size_ = 0;

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
static uint32_t dbg_last_print = 0;

static const float ANGLE_INCREMENT = (2.0f * PI) / LIDAR_SCAN_SIZE;

// ────────────────────────── 内部函数 ──────────────────────────

static inline uint16_t bytesToUint16(const uint8_t *data, size_t index)
{
    return (uint16_t)((uint16_t)(data[index + 1]) << 8) | data[index];
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

    uint16_t fsangle_raw = bytesToUint16(packet_data, 4);
    uint16_t lsangle_raw = bytesToUint16(packet_data, 6);

    float angle_start_deg = (float)(fsangle_raw >> 1) / 64.0f;
    float angle_end_deg   = (float)(lsangle_raw >> 1) / 64.0f;

    float diff_angle_deg = 0.0f;
    if (lsn > 1) {
        diff_angle_deg = angle_end_deg - angle_start_deg;
        if (diff_angle_deg < 0.0f) diff_angle_deg += 360.0f;
    }

    for (int i = 0; i < lsn; ++i) {
        size_t offset = 8 + (size_t)i * 3;
        if (offset + 1 >= 8 + (size_t)lsn * 3) break;   // 越界保护

        uint16_t dist_raw = bytesToUint16(packet_data, offset);

        float distance_mm = (float)dist_raw / 4.0f;
        float distance_m  = distance_mm / 1000.0f;

        float angle_deg = angle_start_deg;
        if (lsn > 1) {
            angle_deg = (diff_angle_deg / (float)(lsn - 1)) * i + angle_start_deg;
        }

        // X2 特有的角度修正公式
        float angle_correct_deg = 0.0f;
        if (distance_mm != 0.0f) {
            float numerator   = 21.8f * (155.3f - distance_mm);
            float denominator = 155.3f * distance_mm;
            float angle_correct_rad = atanf(numerator / denominator);
            angle_correct_deg = angle_correct_rad * 180.0f / PI;
        }

        float final_angle_deg = fmodf(angle_deg + angle_correct_deg, 360.0f);
        if (final_angle_deg < 0.0f) final_angle_deg += 360.0f;
        float angle_rad = final_angle_deg * PI / 180.0f;

        if (distance_m > 0.01f) {
            // ── 检测过零（一圈结束）──
            if (angle_rad < last_point_angle_ - PI) {
                if (scan_count_ > 0) {
                    // 上一圈已完整：拷到就绪缓冲
                    memcpy(scan_ready_ranges_, scan_ranges_, sizeof(scan_ready_ranges_));
                    scan_timestamp_ms_ = scan_start_ms_;
                    scan_ready_ = true;
                }
                scan_count_++;
                // 重置缓冲，开始新一圈
                for (int k = 0; k < LIDAR_SCAN_SIZE; k++) scan_ranges_[k] = INFINITY;
                first_packet_of_scan_ = true;
                scan_start_ms_ = millis();
            }

            // ── 映射到 720 索引（与 lidar_node.cpp 一致）──
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
            last_point_angle_ = angle_rad;
        }
    }
}

/** 逐字节喂入状态机 */
static void processByte(uint8_t byte)
{
    switch (state_) {
    case WAIT_HEADER1:
        if (byte == 0xAA) {
            state_ = WAIT_HEADER2;
            packet_len_ = 0;
            packet_buffer_[packet_len_++] = byte;
        }
        break;

    case WAIT_HEADER2:
        if (byte == 0x55) {
            state_ = READ_META;
            packet_buffer_[packet_len_++] = byte;
        } else if (byte == 0xAA) {
            packet_len_ = 0;
            packet_buffer_[packet_len_++] = byte;   // 连续 AA，重新计数
        } else {
            state_ = WAIT_HEADER1;
            packet_len_ = 0;
        }
        break;

    case READ_META:
        packet_buffer_[packet_len_++] = byte;
        if (packet_len_ == 4) {
            current_lsn_ = packet_buffer_[3];
            if (current_lsn_ > 32) {   // 非法包保护
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

/** 帧结构捕获：检测 6B 93 77 10 00 1C 同步头，抓到内存后统一打印（协议反推用） */
/** 独立解析任务：持续读串口、解析，不受主循环阻塞影响 */
static void lidarTask(void *arg)
{
    while (true) {
        while (LidarSerial.available() > 0) {
            uint8_t b = (uint8_t)LidarSerial.read();
            processByte(b);
            dbg_bytes_rx++;
        }

        // 每 5 秒打印一次诊断信息
        if (millis() - dbg_last_print > 5000) {
            dbg_last_print = millis();
            uint32_t valid = 0;
            for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
                if (scan_ready_ranges_[i] != INFINITY) valid++;
            }
            Serial.printf("[Lidar DBG] RX=%luB 包=%lu 圈=%d 有效点=%lu 就绪=%d\n",
                (unsigned long)dbg_bytes_rx, (unsigned long)dbg_packets_ok,
                scan_count_, (unsigned long)valid, scan_ready_ ? 1 : 0);
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
        Serial.printf("[Baud] %lu: RX=%lu  AA55=%lu  6B9377=%lu\n",
            (unsigned long)bauds[i], (unsigned long)count,
            (unsigned long)aa55, (unsigned long)sync6b93);
    }
    Serial.println("[Lidar] === 波特率扫描结束 ===");
}

// ────────────────────────── 对外接口 ──────────────────────────

void Lidar_Init()
{
    // 增大接收缓冲，防止主循环忙时丢字节（必须在第一次 begin() 之前）
    LidarSerial.setRxBufferSize(2048);

    // 波特率已确认 = 150000，扫描不再自动执行。
    // 若换雷达型号需要重新扫描，把下面这行取消注释：
    // lidarBaudSweep();

    // 清空缓冲
    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        scan_ranges_[i]      = INFINITY;
        scan_ready_ranges_[i] = INFINITY;
    }

    LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
    delay(100);
    LidarSerial.flush();

    // 发送启动指令 A5 60
    uint8_t start_cmd[] = {0xA5, 0x60};
    LidarSerial.write(start_cmd, sizeof(start_cmd));
    delay(50);
    LidarSerial.flush();

    // 创建独立解析任务（栈 4096，优先级 2）
    xTaskCreate(lidarTask, "lidar_task", 4096, NULL, 2, NULL);

    Serial.printf("[Lidar] init done, UART1 %d baud (RX=%d TX=%d)\n",
        LIDAR_BAUDRATE, LIDAR_RX, LIDAR_TX);
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
