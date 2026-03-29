#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Motor_control.h>



//定义四个轮子


#define MOTOR1_ID 1
#define MOTOR1_IN1 7                            // 11111111             4444444444
#define MOTOR1_IN2 15                           // 22222222             3333333333
#define MOTOR1_PWM 16

#define MOTOR2_ID 2
#define MOTOR2_IN1 40
#define MOTOR2_IN2 41
#define MOTOR2_PWM 39

#define MOTOR3_ID 3
#define MOTOR3_IN1 2
#define MOTOR3_IN2 42
#define MOTOR3_PWM 1


#define MOTOR4_ID 4
#define MOTOR4_IN1 5
#define MOTOR4_IN2 6
#define MOTOR4_PWM 4



void My_motor::Motor_Init(int ID, int turn)
{
    // 电机一--------------------------------------------------------------------------
    // 正反转
    if (ID == 1)
    {
        pinMode(MOTOR1_IN2, OUTPUT);
        pinMode(MOTOR1_IN1, OUTPUT);
        if (turn == 0)
        {
            digitalWrite(MOTOR1_IN1, HIGH);
            digitalWrite(MOTOR1_IN2, LOW);
        }
        else
        {
            digitalWrite(MOTOR1_IN1, LOW);
            digitalWrite(MOTOR1_IN2, HIGH);
        }
        // PWM
        pinMode(MOTOR1_PWM, OUTPUT);
    }
    // 电机二---------------------------------------------------------------------------
    if (ID == 2)
    {
        pinMode(MOTOR2_IN2, OUTPUT);
        pinMode(MOTOR2_IN1, OUTPUT);
        if (turn == 1)
        {
            digitalWrite(MOTOR2_IN1, HIGH);
            digitalWrite(MOTOR2_IN2, LOW);
        }
        else
        {
            digitalWrite(MOTOR2_IN1, LOW);
            digitalWrite(MOTOR2_IN2, HIGH);
        }
        pinMode(MOTOR2_PWM, OUTPUT);
    }
    // 电机三---------------------------------------------------------------------------
    if (ID == 3)
    {
        pinMode(MOTOR3_IN2, OUTPUT);
        pinMode(MOTOR3_IN1, OUTPUT);
        if (turn == 1)
        {
            digitalWrite(MOTOR3_IN1, HIGH);
            digitalWrite(MOTOR3_IN2, LOW);
        }
        else
        {
            digitalWrite(MOTOR3_IN1, LOW);
            digitalWrite(MOTOR3_IN2, HIGH);
        }
        pinMode(MOTOR3_PWM, OUTPUT);
    }
    // 电机四---------------------------------------------------------------------------
    if (ID == 4)
    {
        pinMode(MOTOR4_IN2, OUTPUT);
        pinMode(MOTOR4_IN1, OUTPUT);
        if (turn == 0)
        {
            digitalWrite(MOTOR4_IN1, HIGH);
            digitalWrite(MOTOR4_IN2, LOW);
        }
        else
        {
            digitalWrite(MOTOR4_IN1, LOW);
            digitalWrite(MOTOR4_IN2, HIGH);
        }
        pinMode(MOTOR4_PWM, OUTPUT);
    }
}
//
// 输入电机的编号和速度实现速度占空比控速
//
void My_motor::Motor_Run(int ID, float  pwm)
{
    float a = pwm * 0.01;
    if (pwm > 0)
    {
        //设置电机的正反转
        if (ID == 1)
        {
            Motor_Init(1, 0);
            analogWrite(MOTOR1_PWM, int(255 * a));
        }
        if (ID == 2)
        {
            Motor_Init(2, 0);
            analogWrite(MOTOR2_PWM, int(255 * a));
        }
        if (
            ID == 3)
        {
            Motor_Init(3, 0);
            analogWrite(MOTOR3_PWM, int(255 * a));
        }
        if (ID == 4)
        {
            Motor_Init(4, 1);
            analogWrite(MOTOR4_PWM, int(255 * a));
        }
    }
    else if (pwm < 0)
    {
        a = -a;
        if (ID == 1)
        {
            Motor_Init(1, 1);
            analogWrite(MOTOR1_PWM, int(255 * a));
        }
        if (ID == 2)
        {
            Motor_Init(2, 1);
            analogWrite(MOTOR2_PWM, int(255 * a));
        }
        if (
            ID == 3)
        {
            Motor_Init(3, 1);
            analogWrite(MOTOR3_PWM, int(255 * a));
        }
        if (ID == 4)
        {
            Motor_Init(4, 0);
            analogWrite(MOTOR4_PWM, int(255 * a));
        }
    }
}
void My_motor::Motor_Speed(int V)
{
}
