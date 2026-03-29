#include <Arduino.h>

#include "Diantance_check.h" //测量距离

#include "MPU6050.h" //测量姿态

#include "Motor_control.h" //电机初始化

#include "Pid_control.h" //使用PID

#include "Kinematics.h" //运动学正逆解

#include "Servo.h" //舵机

#include "newencoder.h" //编码器测速

#include "Netprint.h" //UDP调试

#include "Microros.h" //发布ros的

#include "St7735.h" //屏幕初始化

#include "WiFi.h"

My_motor my_motor[4];                   // 电机实例化
PID_run run[4];                         // 使用PID控制电机
extern float Velocity[4];               // 拿到测量后的速度
extern PIDController pid_controller[4]; // PID控制器实例化
extern IPAddress serverIP;

void setup()
{
    Serial.begin(115200);
    xTaskCreate(micro_ros_task, "uros_task", 32768, NULL, 1, NULL); // 发布话题

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print("."); // wifi初始化
    }
    Serial.println("\nWiFi connected!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP()); // 打印wifi的ip

    // MPU6050_Init(); // 初始化MPU6050

    // St7735_Init(); // 初始化屏幕

    // Distance_chect_Init();//初始化测距模块

    Encoder_Init(); // 编码器初始化

    Pid_controller_init(); // PID控制器初始化

    Serial.println("初始化成功");
}
void loop()
{
    // 测姿态-----------------------------------------------------------------------------------------------------------------------------------------------
    // MPU6050_check();

    // 测距离------------------------------------------------------------------------------------------------------------------------------------------------------
    // float distance = Distance_chect();
    // Serial.print("距离：");
    // Serial.print(distance);
    // Serial.println("cm");
    // float distance = 0;
    // netPrintf("距离：%fcm\n", distance);

    // 测速度-----------------------------------------------------------------------------------------------------------------------------------------------
    // Encoder_Check();
    Velocity_Check();

    // 里程计打印--------------------------------------------------------------------------------------------------------------------------------
    odom_update();


    run[0].Pid_run_();
    run[1].Pid_run_();
    run[2].Pid_run_();
    run[3].Pid_run_();
    // St7735屏幕的测试
    // St7735_test();
}
