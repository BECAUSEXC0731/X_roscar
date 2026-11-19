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

// ============================================================
// 轮子下标约定（与电机 / 编码器编号一一对应，不可随意调换）
//   0 → 电机1 / 编码器0   左前 FL
//   1 → 电机2 / 编码器1   左后 RL
//   2 → 电机3 / 编码器2   右后 RR
//   3 → 电机4 / 编码器3   右前 FR
// ============================================================
#define WHEEL_FL   0
#define WHEEL_RL   1
#define WHEEL_RR   2
#define WHEEL_FR   3
#define WHEEL_NUM  4

// 底盘运动学模型
enum KinType : uint8_t
{
    KIN_DIFF    = 0,   // 四轮差速：左右两侧各两轮同速
    KIN_MECANUM = 1,   // 四轮麦克纳姆：可全向平移
};

// 运动学参数（存 NVS，可由配网页修改）
typedef struct
{
    uint8_t type;        // KinType
    float   track;       // 轮距 mm，左右轮中心距
    float   wheel_base;  // 轴距 mm，前后轮中心距（差速模型不参与计算）
    float   omega_scale; // 角速度校准系数，1.0 = 不校准
} KinematicsConfig;

// 设置 / 读取当前运动学参数（开机时从 NVS 载入一次）
void Kinematics_SetConfig(const KinematicsConfig &cfg);
const KinematicsConfig &Kinematics_GetConfig();

// 逆解：底盘速度 → 四个轮速
//   linear_x   mm/s   前进为正
//   linear_y   mm/s   左移为正（差速模型忽略）
//   angular_z  rad/s  逆时针为正
//   out[4]     轮速 mm/s，下标见 WHEEL_xxx
void Kinematics_inverse(float linear_x, float linear_y, float angular_z, float out[WHEEL_NUM]);

// 正解：四个轮速 → 底盘速度
void Kinematics_forward(const float wheel[WHEEL_NUM],
                        float *out_linear_x, float *out_linear_y, float *out_angular_z);

// 里程计结构体
typedef struct
{
    float x;              // m
    float y;              // m
    float angle;          // rad，-PI ~ PI
    float linear_speed;   // mm/s，前进方向
    float lateral_speed;  // mm/s，左移方向（差速模型恒为 0）
    float angular_speed;  // rad/s
} odom_t;

extern odom_t odom;

// 里程计更新（在 micro-ROS 定时器里周期调用）
//   mpu_yaw_deg  外部航向（MPU 偏航角，单位度），用于抑制打滑带来的角度漂移
//   mpu_valid    mpu_yaw_deg 是否有效；为 false 时退化为纯轮速积分
void odom_update(float mpu_yaw_deg = 0.0f, bool mpu_valid = false);

// 把角度归一化到 -PI ~ PI
float angle_normalize(float angle);

#endif
