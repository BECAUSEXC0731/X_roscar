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

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 每个电机的引脚与配置
struct motor_cfg_t {
    uint8_t pin_in1;
    uint8_t pin_in2;
    uint8_t pin_pwm;
    uint8_t ledc_ch;   // LEDC 通道
    bool    reverse;    // true = 硬件接线相反，软件取反
};

class My_motor
{
public:
    My_motor() = default;

    // 一次性初始化所有电机（放到 setup() 中调用一次即可）
    void Motor_Setup();

    // 运行电机：pwm = -100 ~ 100，自动处理方向、刹车
    void Motor_Run(int ID, float pwm);

    //刹车
    void Motor_Brake(int ID);

    //滑行
    void Motor_Coast(int ID);

private:
    // 根据 forward + reverse 标志，设置 IN1/IN2 方向电平
    void set_direction(int ID, bool forward);
};

extern My_motor my_motor[4];
#endif
