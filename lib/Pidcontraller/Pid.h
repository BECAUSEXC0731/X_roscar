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

#ifndef PID_H
#define PID_H

class PIDController
{
public:
    PIDController() = default;
    PIDController(float Kp, float Ki, float Kd);

private:
    float target_=0, output_up=0, output_down=0;
    float Kp_=0, Ki_=0, Kd_=0;
    float error_=0, error_last_=0, error_sum_=0, derror_=0;
    
    float intergral_up_ = 800; // 设置积分上下限

public:
    float update(float current);                // 提供当前值，返回下次输出值，也就是PID的结果
    void Set_Target(float target);              // 设置目标值
    void Set_Pid(float Kp, float Ki, float Kd); // 设置PID的参数
    void Reset();                               // 重置PID
    void Out_limit(float max, float min);       // 设置输出限幅
};

#endif
