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

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "PinConfig.h"
#include "newencoder.h"

// 创建 4 个编码器对象
ESP32Encoder encoders[4];

// ============================================================
// 测速参数
// ============================================================
// 固定采样窗口：不再每个 loop 都算一次速度，而是每 20ms 结算一次。
// 速度量子 = MM_PER_TICK / 0.02s = 14 mm/s
// （原来按约 2ms 的 loop 周期算，量子是 140 mm/s，所以低速时只在 0/140 之间跳）
static const uint32_t VELOCITY_SAMPLE_US = 20000;

// 每个编码器计数对应的轮子位移（沿用原标定）
static const float MM_PER_TICK = 0.28f;

// 一阶低通滤波系数（0~1）：抑制编码器量化带来的速度跳变
static const float VELOCITY_ALPHA = 0.5f;

// 编码器初始化
void Encoder_Init()
{
    // 启用内部弱上拉（多数编码器需要）
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // 初始化 4 个编码器
    encoders[3].attachHalfQuad(ENCODER3_A, ENCODER3_B);
    encoders[1].attachHalfQuad(ENCODER1_A, ENCODER1_B);

    encoders[0].attachHalfQuad(ENCODER0_A, ENCODER0_B);
    encoders[2].attachHalfQuad(ENCODER2_A, ENCODER2_B);

    for (int i = 0; i < 4; i++) {
        encoders[i].setFilter(100);
    }
}

// 编码器计数（保留原接口，仅读取一次，不打印）
void Encoder_Check()
{
    for (int i = 0; i < 4; i++) {
        (void)encoders[i].getCount();
    }
}

int64_t last_encoder[4] = {0};   // 上一次结算时的编码器值
float   Velocity[4]     = {0};   // 四个轮子的速度（mm/s，滤波后）

static bool     velocity_started = false;
static uint32_t last_sample_us   = 0;

// 固定 20ms 窗口测速 + 一阶低通滤波
// 返回值：true = 本次结算出了新速度（主循环据此以 50Hz 运行 PID）
bool Velocity_Check()
{
    uint32_t now = micros();

    // 第一次调用只记录基准计数与时间，不产生速度
    if (!velocity_started) {
        for (int i = 0; i < 4; i++) {
            last_encoder[i] = encoders[i].getCount();
        }
        last_sample_us = now;
        velocity_started = true;
        return false;
    }

    // 无符号相减，micros() 回绕（约 71 分钟）时依然正确
    uint32_t elapsed = now - last_sample_us;
    if (elapsed < VELOCITY_SAMPLE_US) return false;   // 固定周期未到，沿用上次速度

    float dt = elapsed / 1000000.0f;   // 秒

    for (int i = 0; i < 4; i++) {
        int64_t current = encoders[i].getCount();
        // 整个窗口内的位移 / 窗口时长，量化误差被摊薄到 20ms
        float raw = (current - last_encoder[i]) * MM_PER_TICK / dt;
        last_encoder[i] = current;

        // 一阶低通：v = v + α(raw - v)
        Velocity[i] += VELOCITY_ALPHA * (raw - Velocity[i]);
    }

    last_sample_us = now;
    return true;
}
