#include <Arduino.h>
#include "Pid.h"
float stop = 0;
// 构造函数传入PID参数
PIDController::PIDController(float Kp, float Ki, float Kd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

// 设置目标值
void PIDController::Set_Target(float target)
{
    target_ = target;
}

// 计算PID输出
float PIDController::update(float current)
{
    if (target_ != 0)
    {
        error_ = target_ - current; // 计算误差的积分，并限幅
        error_sum_ += error_;
        if (error_sum_ > intergral_up_)
            error_sum_ = intergral_up_;
        if (error_sum_ < -1 * intergral_up_)
            error_sum_ = -1 * intergral_up_;

        derror_ = error_ - error_last_; // 记录误差的变化率
        error_last_ = error_;           // 更新误差

        float output = Kp_ * error_ + Ki_ * error_sum_ + Kd_ * derror_;

        if (output > output_up)
            output = output_up; // 输出值限幅
        else if (output < output_down)
            output = output_down;
        stop = output;
        return output;
    }
    else
    {
    error_sum_ = 0;
    // 对stop进行衰减，每次减少一定比例，直到小于某个阈值然后置0
    stop = stop * 0.9;   // 或者0.8，根据需求调整
    if (fabs(stop) < 0.1) // 假设阈值0.1
        stop = 0;

    // 限幅
    if (stop > output_up)
        stop = output_up;
    else if (stop < output_down)
        stop = output_down;

    return stop;
}


}

// 初始化设置参数
void PIDController::Set_Pid(float Kp, float Ki, float Kd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

// 重置参数
void PIDController::Reset()
{
    Kp_ = 0;
    Ki_ = 0;
    Kd_ = 0;
    error_ = 0;
    error_last_ = 0;
    error_sum_ = 0;
}

// 设置输出限幅的值
void PIDController::Out_limit(float max, float min)
{
    output_down = min;
    output_up = max;
}