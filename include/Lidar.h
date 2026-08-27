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

// scan 分辨率：一圈 200 个点（1.8°/点）
// ⚠️ 受当前 micro-ROS MTU=1024 限制：360点(1.5KB) 会超限发布失败，
//    200点(约870B) 可正常发布。若想用更高分辨率(360/720点)，
//    需重建 micro-ROS 库并把 custom.meta 的 MTU 提到 2048（见 platformio.ini）
#define LIDAR_SCAN_SIZE 200

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
