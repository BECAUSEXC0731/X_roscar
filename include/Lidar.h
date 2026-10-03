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

#ifndef LIDAR__H
#define LIDAR__H

#include <Arduino.h>

// scan 分辨率：一圈 224 个点（≈1.61°/点）
//
// ⚠️ 点数受**两端**共同限制（LaserScan 序列化大小 ≈ 点数×4 + 72B，含 XRCE 帧头）：
//
//   ① 客户端（本固件）：custom.meta 里 UCLIENT_CUSTOM_TRANSPORT_MTU=2048（已重建库生效）
//
//   ② Agent 端：老版 micro_ros_agent（fishros 镜像 :humble）的 UDP MTU 是**编译期常量**，
//      命令行没有 --mtu 选项，改不了。实测：
//        200 点 /  860 B  ✅ 通过
//        350 点 / 1472 B  ❌ deserialization error processing WRITE_DATA submessage
//      → 实际上限约 1024 B，所以取 224 点（968 B），留约 56 B 余量。
//      （想再挤一点可以试 228/232，但余量只剩 20~40 B；要明显更高分辨率就得
//        自己编译一个 MTU 更大的 agent，见 MICROROS_MTU_REBUILD.md）
#define LIDAR_SCAN_SIZE 224

// ── 有效量程窗口 ───────────────────────────────────────────
// X2 盲区约 0.1m，最大 8m。窗口内的点才写入扫描缓冲，
// 免得把近场/超量程的噪声塞进地图（下游 range_min/max 各消费者行为不一致）
#define LIDAR_RANGE_MIN_M   0.10f
#define LIDAR_RANGE_MAX_M   8.00f

// ── 离群点滤波 ─────────────────────────────────────────────
// 与树莓派版 lidar_pkg/config/lidar_params.yaml 对齐：
// 某点在半径内（含自身）邻居少于 min_neighbors 个 → 判为孤立噪点剔除
#define LIDAR_FILTER_ENABLE         1
#define LIDAR_FILTER_RADIUS_M       0.10f
#define LIDAR_FILTER_MIN_NEIGHBORS  2

// 一圈有效点少于该数量时认为数据不可信，丢弃不发布
#define LIDAR_MIN_VALID_POINTS      10

/**
 * 初始化雷达：
 *  - 配置 UART1 (150000 波特, RX=44, TX=43)
 *  - 发送启动指令 A5 60
 *  - 创建独立解析任务
 */
void Lidar_Init();

/**
 * 是否有新的一整圈数据就绪（供 micro-ROS 侧查询）
 */
bool Lidar_isScanReady();

/**
 * 把就绪的 scan 数据拷贝到用户缓冲区
 * @param ranges       接收缓冲区，至少 LIDAR_SCAN_SIZE 个 float
 * @param timestamp_ms 可选，返回该圈开始时的 millis() 时间戳
 */
void Lidar_getScan(float *ranges, uint32_t *timestamp_ms = nullptr);

#endif // LIDAR__H
