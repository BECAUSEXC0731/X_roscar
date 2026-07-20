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
