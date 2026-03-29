#include <Arduino.h>
#include "Diantance_check.h"
#include "MPU6050.h"
#include "Motor_control.h" //电机初始化
#include "Pid_control.h"   //使用PID
#include "Kinematics.h"    //运动学正逆解
#include "Servo.h"         //舵机
#include "newencoder.h"    //编码器测速
#include "Netprint.h"      //UDP调试
#include "Microros.h"
#include "St7735.h"

// Wifi配置用于UDP调试，如果需要网络打印功能--------------------------------------------------------------------
// const char *ssid = "doomsday";
// const char *password = "123123123";
// 目标服务器的配置
// WiFiUDP udp;
// IPAddress serverIP(10, 38, 172, 51); // 上位机的ip

My_motor my_motor[4]; // 电机实例化
PID_run run[4];
float a=0,b=0,c=0,d=0;//读取PWM
extern float Velocity[4];
extern PIDController pid_controller[4];
void setup()
{
    Serial.begin(115200);
    xTaskCreate(micro_ros_task, "uros_task", 32768, NULL, 1, NULL); // 发布话题
    // Servo_init();
    //----------------------------------------------------------------------------------------------------
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(500);
    //     // Serial.print(".");                                                            //wifi初始化
    // }
    // Serial.println("\nWiFi connected!");
    // Serial.print("ESP32 IP: ");
    // Serial.println(WiFi.localIP());
    //------------------------------------------------------------------------------------------------------
    // MPU6050_Init();//初始化MPU6050

    St7735_Init();//初始化屏幕

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
     
    // 测电机---------------------------------------------------------------------------------------------------------------------------------
    //  my_motor[0].Motor_Run(1,50);
    //  my_motor[1].Motor_Run(2,50);
    //  my_motor[2].Motor_Run(3,50);
    //  my_motor[3].Motor_Run(4,50);

    //  delay(3000);

    //  my_motor[0].Motor_Run(1,0);
    //  my_motor[1].Motor_Run(2,0);
    //  my_motor[2].Motor_Run(3,0);
    //  my_motor[3].Motor_Run(4,0);
     
    //  delay(3000);


    // Motor_test1(100);

    // 测PID-------------------------------------------------------------------------------------------------------------------------------
    // Pid_controller_run(400.0);//单位mm/s
     

    // for (int i = 0; i < 4; i++)
    // {
    //     run[i].Pid_run_();
    // }

    // 测运动学解算---------------------------------------------------------------------------------------------------------------------------
    // float target_linear = 100;
    // float target_angular = 0.5;
    // float out_MotorL, out_MotorR;
    // Kinematics_down(target_linear, target_angular, &out_MotorL, &out_MotorR);
    // run[0].Pid_init(1, out_MotorL);
    // run[1].Pid_init(2, out_MotorL);
    // run[2].Pid_init(3, out_MotorR);
    // run[3].Pid_init(4, out_MotorR);

    run[0].Pid_run_();
    run[1].Pid_run_();
    run[2].Pid_run_();
    run[3].Pid_run_();

    //查看实际给车子的脉冲的占空比
    // a=pid_controller[1].update(Velocity[0]);
    // b=pid_controller[2].update(Velocity[1]);
    // c=pid_controller[3].update(Velocity[2]);
    // d=pid_controller[4].update(Velocity[3]);
    // Serial.printf("给电机的pwm：%f,%f,%f,%f\n",a,b,c,d);

    // 里程计打印--------------------------------------------------------------------------------------------------------------------------------
     odom_update();

    // St7735屏幕的测试
    //t7735_test();

}
