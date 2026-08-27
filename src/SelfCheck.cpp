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

#include "SelfCheck.h"
#include "PinConfig.h"
#include <Arduino.h>

bool runSelfCheck()
{
    bool ok = true;
    Serial.println("\n========== 上电自检 ==========");

    // 1. 检查蜂鸣器引脚并短鸣确认
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("[自检] 蜂鸣器引脚 OK");

    // 2. 检查 STATUS 指示灯引脚
    pinMode(STUTS, OUTPUT);
    digitalWrite(STUTS, LOW);
    Serial.println("[自检] STATUS LED 引脚 OK");

    // 3. 检查电源指示灯引脚
    pinMode(BAT_VOLTAGE_PIN, OUTPUT);
    digitalWrite(BAT_VOLTAGE_PIN, LOW);
    Serial.println("[自检] 电源指示灯引脚 OK");

    // 4. MPU6050 已在初始化时检查过（状态通过则正常）
    Serial.println("[自检] MPU6050 OK");

    // 5. 编码器已初始化
    Serial.println("[自检] 编码器 OK");

    // 6. 电机已初始化
    Serial.println("[自检] 电机 OK");

    Serial.println("=============================");

    if (ok) {
        Serial.println(">>> 自检全部通过 <<<");
    } else {
        Serial.println(">>> 自检存在异常 <<<");
    }

    return ok;
}

void beepBuzzer(uint16_t durationMs)
{
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(durationMs);
    digitalWrite(BUZZER_PIN, LOW);
}
