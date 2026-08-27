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

#ifndef Kinematics_h
#define Kinematics_h
#include "Arduino.h"

// //将小车的实际轮速转换为小车整体的角速度和线速度
// typedef struct
// {
//     float pluse_distance;
//     int16_t Velocity;
//     int64_t last_encoder;
// }motor_date;

// class Kinematics

// {

// private:float wheel_distance;

// public:
//     Kinematics() = default;
//     ~Kinematics() = default;
// 运动学正解，输出的机器人线速度和角速度
void Kinematics_up(float Motor1, float Motor3, float Motor4, float Motor2, float *out_angular_speed, float *out_linear_speed);
// 运动学逆解，输出机器人的轮速
void Kinematics_down(float linear_speed, float angular_speed, float *out_MotorL, float *out_MotorR);
// 设置轮子间距

// 里程计结构体的参数
typedef struct
{ 
    float x;
    float y;
    float angle;
    float linear_speed;
    float angular_speed;
} odom_t;


//里程计函数
//里程计更新含函数
void odom_update();
//角度从yaw转换为-Π到Π
void angle_trans(float angle,float &angle_out);
//获取里程计值

// };

#endif
