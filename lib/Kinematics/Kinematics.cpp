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

#include "Kinematics.h"
#include "Arduino.h"
 

extern float Velocity[4];
int64_t last_time_K = 0;

odom_t odom;                // 用于存储数据的结构体
float wheel_distance = 165; // 单位mm 设置轮距---------------------------------------------------------------------------

// 输出机人的角速度和线速度
void Kinematics_up(float Motor1, float Motor3, float Motor4, float Motor2, float *out_angular_speed, float *out_linear_speed)

{
    float left_avg = (Motor1 + Motor3) / 2.0;
    float right_avg = (Motor2 + Motor4) / 2.0;

    *out_linear_speed = (left_avg + right_avg) / 2.0;
    *out_angular_speed = (right_avg - left_avg) / wheel_distance;
}

// 运动学逆解，输出机器人的轮速
void Kinematics_down(float linear_speed, float angular_speed, float *out_MotorL, float *out_MotorR)
{
    if (linear_speed == 0 && angular_speed == 0)
    {
        *out_MotorL = 0;
        *out_MotorR = 0;
    }
    else
    {
        *out_MotorL = linear_speed - angular_speed * wheel_distance / 2;
        *out_MotorR = linear_speed + angular_speed * wheel_distance / 2;
    }
}

// 里程计更新含函数
void odom_update()
{
    // 时间间隔
    int64_t now_K = millis();
    float dt = now_K - last_time_K;
    float dt_s = float(dt) / 1000.0;
    // 获取当前的角速度，然后用运动学正解为轮子的速度
    Kinematics_up(Velocity[0], Velocity[1], Velocity[2], Velocity[3], &odom.angular_speed, &odom.linear_speed);

    // 角度积分
    odom.angle += odom.angular_speed * dt_s;

    // 角度转换
    angle_trans(odom.angle, odom.angle);

    // 计算机器人行走的距离单位（m）
    float delt_distance = odom.linear_speed * dt_s;
    odom.x = odom.x + delt_distance * cos(odom.angle)/1000;
    odom.y = odom.y + delt_distance * sin(odom.angle)/1000;
    last_time_K = now_K;

    //Serial.printf(  "x:%f,y:%f,angle:%f\n", odom.x, odom.y, odom.angle );
}

// 将角度从yaw转换为-Π到Π
void angle_trans(float angle, float &angle_out)
{
    if (angle > PI)
        angle_out = angle - 2 * PI;
    else if (angle < -PI)
        angle_out = angle + 2 * PI;
    else
        angle_out = angle;
}
