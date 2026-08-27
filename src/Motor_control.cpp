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
#include <Motor_control.h>
#include "PinConfig.h"

// ============================================================
// 每个电机的引脚与配置
// ============================================================
static const motor_cfg_t motor_cfg[] = {
    // { IN1, IN2, PWM, ledc_ch, reverse }
    { MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, 0, false },  // 电机1
    { MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, 1, true  },  // 电机2
    { MOTOR3_IN1, MOTOR3_IN2, MOTOR3_PWM, 2, true  },  // 电机3
    { MOTOR4_IN1, MOTOR4_IN2, MOTOR4_PWM, 3, true  },  // 电机4
};

#define MOTOR_COUNT  (sizeof(motor_cfg) / sizeof(motor_cfg[0]))

// 死区：低于此 PWM 值电机转不动，直接跳过
#define MOTOR_DEAD_ZONE  5

// ============================================================
// 一次性初始化所有电机 — 放到 setup() 开头调用一次
// ============================================================
void My_motor::Motor_Setup()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        const auto &m = motor_cfg[i];

        pinMode(m.pin_in1, OUTPUT);
        pinMode(m.pin_in2, OUTPUT);
        pinMode(m.pin_pwm, OUTPUT);

        ledcSetup(m.ledc_ch, 20000, 8);
        ledcAttachPin(m.pin_pwm, m.ledc_ch);

        // 初始状态：刹车
        digitalWrite(m.pin_in1, HIGH);
        digitalWrite(m.pin_in2, HIGH);
        ledcWrite(m.ledc_ch, 0);
    }
}

// ============================================================
// 设置 IN1/IN2 方向电平
// ============================================================
void My_motor::set_direction(int ID, bool forward)
{
    if (ID < 1 || ID > MOTOR_COUNT) return;
    const auto &m = motor_cfg[ID - 1];

    // reverse ^ forward：异或，硬件反接时自动翻转方向
    bool in1 =  forward ^ m.reverse;
    bool in2 = !forward ^ m.reverse;

    digitalWrite(m.pin_in1, in1 ? HIGH : LOW);
    digitalWrite(m.pin_in2, in2 ? HIGH : LOW);
}

// ============================================================
// 运行电机：pwm = -100 ~ 100
//   正值 → 正转，负值 → 反转，接近 0 → 刹车
// ============================================================
void My_motor::Motor_Run(int ID, float pwm)
{
    if (ID < 1 || ID > MOTOR_COUNT) return;
    const auto &m = motor_cfg[ID - 1];

    // pwm 接近 0 → 刹车
    if (fabs(pwm) < 0.5f) {
        Motor_Brake(ID);
        return;
    }

    // 限幅
    if (pwm >  100.0f) pwm =  100.0f;
    if (pwm < -100.0f) pwm = -100.0f;

    bool forward = (pwm > 0);
    float speed = fabs(pwm) / 100.0f;         // 0.0 ~ 1.0
    int duty = (int)(speed * 255);

    // 死区补偿：低于阈值则直接拉到最小值，避免电机堵转不转
    if (duty > 0 && duty < MOTOR_DEAD_ZONE) {
        duty = MOTOR_DEAD_ZONE;
    }

    set_direction(ID, forward);
    ledcWrite(m.ledc_ch, duty);
}

// ============================================================
// 刹车：IN1=H, IN2=H  → 电机端子短路，急停
// ============================================================
void My_motor::Motor_Brake(int ID)
{
    if (ID < 1 || ID > MOTOR_COUNT) return;
    const auto &m = motor_cfg[ID - 1];

    digitalWrite(m.pin_in1, HIGH);
    digitalWrite(m.pin_in2, HIGH);
    ledcWrite(m.ledc_ch, 0);
}

// ============================================================
// 滑行：IN1=L, IN2=L  → 电机自由转动
// ============================================================
void My_motor::Motor_Coast(int ID)
{
    if (ID < 1 || ID > MOTOR_COUNT) return;
    const auto &m = motor_cfg[ID - 1];

    digitalWrite(m.pin_in1, LOW);
    digitalWrite(m.pin_in2, LOW);
    ledcWrite(m.ledc_ch, 0);
}
