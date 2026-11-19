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

extern float Velocity[WHEEL_NUM];

// 默认值与改造前的差速模型完全一致：老设备 NVS 里没有运动学参数时行为不变
static KinematicsConfig cfg_ = {KIN_DIFF, 165.0f, 0.0f, 1.0f};

int64_t last_time_K = 0;
odom_t odom;

// 航向融合增益：每个里程计周期把轮速积分出的角度往 MPU 航向拉一点。
// 50ms 周期下 0.02 约为 2.5s 时间常数；置 0 即完全退化为纯轮速积分。
#define ODOM_YAW_FUSE_GAIN 0.02f

// ω = 1rad/s 时单侧轮速的变化量：
//   差速 → 轮距/2        麦轮 → (轮距+轴距)/2
// 正逆解共用同一个 k，保证两者互为逆运算；角速度校准系数也在这里统一生效。
static float rotation_radius()
{
    float base = (cfg_.type == KIN_MECANUM)
                     ? (cfg_.track + cfg_.wheel_base) / 2.0f
                     : cfg_.track / 2.0f;
    if (base < 1.0f) base = 1.0f;   // 参数填 0 时兜底，避免除零
    return base * cfg_.omega_scale;
}

void Kinematics_SetConfig(const KinematicsConfig &cfg)
{
    cfg_ = cfg;
    if (cfg_.type != KIN_MECANUM) cfg_.type = KIN_DIFF;
    if (cfg_.track < 1.0f) cfg_.track = 165.0f;
    if (cfg_.wheel_base < 0.0f) cfg_.wheel_base = 0.0f;
    if (cfg_.omega_scale <= 0.0f) cfg_.omega_scale = 1.0f;

    // Serial.printf("[运动学] 模型=%s 轮距=%.1fmm 轴距=%.1fmm 角速度系数=%.3f\n",
                  // cfg_.type == KIN_MECANUM ? "麦克纳姆" : "差速",
                  // cfg_.track, cfg_.wheel_base, cfg_.omega_scale);

    if (cfg_.type == KIN_MECANUM && cfg_.wheel_base <= 0.0f)
        Serial.println("[运动学] ⚠️ 麦轮模式轴距为 0，原地旋转速度会错，请在配网页填写轴距");
}

const KinematicsConfig &Kinematics_GetConfig()
{
    return cfg_;
}

// 运动学逆解，输出机器人的轮速
void Kinematics_inverse(float linear_x, float linear_y, float angular_z, float out[WHEEL_NUM])
{
    float rot = angular_z * rotation_radius();

    if (cfg_.type == KIN_MECANUM)
    {
        out[WHEEL_FL] = linear_x - linear_y - rot;
        out[WHEEL_RL] = linear_x + linear_y - rot;
        out[WHEEL_RR] = linear_x - linear_y + rot;
        out[WHEEL_FR] = linear_x + linear_y + rot;
    }
    else
    {
        // 差速底盘没有横向自由度，linear_y 直接忽略
        out[WHEEL_FL] = linear_x - rot;
        out[WHEEL_RL] = linear_x - rot;
        out[WHEEL_RR] = linear_x + rot;
        out[WHEEL_FR] = linear_x + rot;
    }
}

// 运动学正解，输出机器人的线速度和角速度
void Kinematics_forward(const float wheel[WHEEL_NUM],
                        float *out_linear_x, float *out_linear_y, float *out_angular_z)
{
    float radius = rotation_radius();

    if (cfg_.type == KIN_MECANUM)
    {
        *out_linear_x = (wheel[WHEEL_FL] + wheel[WHEEL_RL] + wheel[WHEEL_RR] + wheel[WHEEL_FR]) / 4.0f;
        *out_linear_y = (-wheel[WHEEL_FL] + wheel[WHEEL_RL] - wheel[WHEEL_RR] + wheel[WHEEL_FR]) / 4.0f;
        *out_angular_z = (-wheel[WHEEL_FL] - wheel[WHEEL_RL] + wheel[WHEEL_RR] + wheel[WHEEL_FR]) / (4.0f * radius);
    }
    else
    {
        float left_avg = (wheel[WHEEL_FL] + wheel[WHEEL_RL]) / 2.0f;
        float right_avg = (wheel[WHEEL_RR] + wheel[WHEEL_FR]) / 2.0f;

        *out_linear_x = (left_avg + right_avg) / 2.0f;
        *out_linear_y = 0.0f;
        *out_angular_z = (right_avg - left_avg) / (2.0f * radius);
    }
}

// 将角度归一化到 -PI ~ PI
float angle_normalize(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle <= -PI) angle += 2.0f * PI;
    return angle;
}

// 里程计更新
void odom_update(float mpu_yaw_deg, bool mpu_valid)
{
    int64_t now_K = millis();
    if (last_time_K == 0)
    { // 第一次只建立时间基准，避免把开机到现在的时长当成 dt 积进去
        last_time_K = now_K;
        return;
    }

    float dt_s = float(now_K - last_time_K) / 1000.0f;
    last_time_K = now_K;
    if (dt_s <= 0.0f) return;

    Kinematics_forward(Velocity, &odom.linear_speed, &odom.lateral_speed, &odom.angular_speed);

    // 轮速积分得到航向的短期变化
    odom.angle = angle_normalize(odom.angle + odom.angular_speed * dt_s);

    // 再用 MPU 航向做慢速修正：打滑时轮速算出的转角会偏，用陀螺拉回来
    static float mpu_yaw_offset = 0.0f;
    static bool  mpu_yaw_offset_ready = false;
    if (mpu_valid && ODOM_YAW_FUSE_GAIN > 0.0f)
    {
        if (!mpu_yaw_offset_ready)
        { // 以首次运行时的 MPU 航向为 0 点，保持 odom.angle 从 0 开始
            mpu_yaw_offset = mpu_yaw_deg;
            mpu_yaw_offset_ready = true;
        }
        float yaw_rad = (mpu_yaw_deg - mpu_yaw_offset) * DEG_TO_RAD;
        float err = angle_normalize(yaw_rad - odom.angle);
        odom.angle = angle_normalize(odom.angle + err * ODOM_YAW_FUSE_GAIN);
    }

    // 车体系 → 世界系：横移量要随航向一起旋转
    float c = cosf(odom.angle);
    float s = sinf(odom.angle);
    odom.x += (odom.linear_speed * c - odom.lateral_speed * s) * dt_s / 1000.0f;
    odom.y += (odom.linear_speed * s + odom.lateral_speed * c) * dt_s / 1000.0f;
}
