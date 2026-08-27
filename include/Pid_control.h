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

#ifndef PID_CONTROL_H
#define PID_CONTROL_H
#include "Arduino.h"
#include "Pid.h"


void Pid_controller_init();

void Pid_controller_run(float TARGET);//一次控制所有电机


class PID_run
{
public:
    PID_run() = default;
   
private:
    int ID_;
    float TARGET_;
    float lastOutput_ = 0;     // 上次 PID 输出值（用于缓动限幅）

public:
    void Pid_run_();
    void Pid_setgoal(int ID,float TARGET);
};




extern PID_run run[4];
extern PIDController pid_controller[4];
#endif
