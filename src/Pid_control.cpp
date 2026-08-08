#include "Pid.h"
#include "Motor_control.h"
#include "newencoder.h"
#include "Arduino.h"
#include "Pid_control.h"

#define KP1 0.28
#define KI1 0.10
#define KD1 0.01

#define KP2 0.254
#define KI2 0.08
#define KD2 0.05

#define KP3 0.255
#define KI3 0.08
#define KD3 0.01

#define KP4 0.255
#define KI4 0.09
#define KD4 0.01

#define LIMIT 100

PIDController pid_controller[4];
extern My_motor my_motor[4];
extern float Velocity[4];

// 初始化PID三个参数-------------------------------------------------------------
void Pid_controller_init()
{

    pid_controller[0].Set_Pid(KP1, KI1, KD1);
    pid_controller[1].Set_Pid(KP2, KI2, KD2);
    pid_controller[2].Set_Pid(KP3, KI3, KD3);
    pid_controller[3].Set_Pid(KP4, KI4, KD4);

    pid_controller[0].Out_limit(LIMIT, -LIMIT);
    pid_controller[1].Out_limit(LIMIT, -LIMIT);
    pid_controller[2].Out_limit(LIMIT, -LIMIT);
    pid_controller[3].Out_limit(LIMIT, -LIMIT);
}

// PID测试，一次控制所有电机运动-----------------------------------------------------
void Pid_controller_run(float TARGET)
{
    pid_controller[0].Set_Target(TARGET);
    pid_controller[1].Set_Target(TARGET);
    pid_controller[2].Set_Target(TARGET);
    pid_controller[3].Set_Target(TARGET);
   
   
    my_motor[0].Motor_Run(1, pid_controller[0].update(Velocity[0]));
    my_motor[1].Motor_Run(2, pid_controller[1].update(Velocity[1]));
    my_motor[2].Motor_Run(3, pid_controller[2].update(Velocity[2]));
    my_motor[3].Motor_Run(4, pid_controller[3].update(Velocity[3]));
}

// ── 每周期最大输出变化量（缓动限幅，防止抽搐）────────
#define SLEW_RATE  12.0f

//--------------------------------------------------PID运行------------------
void PID_run::Pid_run_()
{
    float raw = pid_controller[ID_].update(Velocity[ID_]);

    // 缓动限幅：本次输出不能比上次突变超过 SLEW_RATE
    float delta = raw - lastOutput_;
    if (delta >  SLEW_RATE) delta =  SLEW_RATE;
    if (delta < -SLEW_RATE) delta = -SLEW_RATE;
    float smoothed = lastOutput_ + delta;
    lastOutput_ = smoothed;

    my_motor[ID_].Motor_Run((ID_ + 1), smoothed);
}

void PID_run::Pid_setgoal(int ID, float TARGET)//给PID控制器设置目标值
{
    ID_ = ID - 1;
    TARGET_ = TARGET;
    pid_controller[ID_].Set_Target(TARGET_);
}

