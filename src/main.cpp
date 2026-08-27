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

#include <Arduino.h>

#include "Distance_check.h" //测量距离

#include "MPU6050.h" //测量姿态

#include "Motor_control.h" //电机初始化

#include "Pid_control.h" //使用PID

#include "Kinematics.h" //运动学正逆解

#include "Servo.h" //舵机

#include "newencoder.h" //编码器测速

#include "Microros.h" //发布ros话题

#include "ConfigManager.h" //配网管理

#include "OledDisplay.h" // OLED 屏幕

#include "PinConfig.h" // 引脚集中配置

#include "SelfCheck.h" // 上电自检

#include "Lidar.h" //雷达驱动 (UART1)

#include "WiFi.h"

My_motor my_motor[4];                   // 电机实例化
PID_run run[4];                         // 使用PID控制电机
extern float Velocity[4];               // 拿到测量后的速度
extern PIDController pid_controller[4]; // PID控制器实例化

// 记录最后收到 cmd_vel 的时间戳
volatile unsigned long lastCmdReceivedMs = 0;

// ════════════════════════════════════════════════════════════
//  默认 WiFi 配置
// ════════════════════════════════════════════════════════════
static const char DEFAULT_SSID[] = "doomsday";
static const char DEFAULT_PASS[] = "123123123";
static const char DEFAULT_AGENT_IP[] = "10.109.159.51";

void setup()
{
    // 读配置按键（GPIO11，接地清空配置并进入配网模式）
    pinMode(CFG_BUTTON, INPUT_PULLUP);
    delay(5);
    bool cfgBtnPressed = (digitalRead(CFG_BUTTON) == LOW);

    Serial.begin(115200);  // UART0 → 连接到板子的 UART USB 口（稳定不掉线）
    delay(500);

    //初始化配置管理器（NVS 读写，必须在 configManager 其他调用之前）
    configManager.begin();

    if (cfgBtnPressed) {
        Serial.println("清空配置并进入配网模式");
        configManager.clearConfig();
        configManager.startAP();
        while (true) {
            configManager.handleClient();
        }
        return;
    }

    // NVS 为空时用编译时的默认配置
    configManager.setDefaults(DEFAULT_SSID, DEFAULT_PASS, DEFAULT_AGENT_IP);

    //已有配置就正常启动
    ConfigData cfg = configManager.getConfig();
    Serial.printf("使用配置: SSID=%s, Agent IP=%s\n", cfg.ssid, cfg.agent_ip);

    xTaskCreate(micro_ros_task, "uros_task", 32768, NULL, 1, NULL); // 发布话题=============================

    WiFi.begin(cfg.ssid, cfg.password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print("."); // wifi初始化
    }
    Serial.println("\nWiFi connected!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP()); // 打印wifi的ip

    // ── MPU6050 初始化（同时初始化 I2C 总线 & 启用 Bypass 模式）─
    MPU6050_Init(); // 初始化MPU6050

    // ── OLED 初始化（通过 MPU XDA/XCL 旁路）────────────────
    oledDisplay.begin();
    oledDisplay.showInitMessage("Initializing...");
    delay(500);
    oledDisplay.setIP(WiFi.localIP().toString().c_str());

    Distance_chect_Init();//初始化测距模块

    Encoder_Init(); // 编码器初始化

    my_motor[0].Motor_Setup(); // 初始化电机引脚和PWM

    Pid_controller_init(); // PID控制器初始化

    Lidar_Init();          // 雷达初始化 (UART1, 150000 波特)

    Servo_init();          // 舵机初始化

    // ── 上电自检 ──────────────────────────────────────────
    bool selfCheckPass = runSelfCheck();

    if (selfCheckPass) {
        Serial.println("初始化成功");
        // 自检通过 → 蜂鸣器短鸣一声（200ms）
        oledDisplay.showInitMessage("SelfCheck OK");
        beepBuzzer(200);
    } else {
        Serial.println("初始化异常！");
        oledDisplay.showInitMessage("SelfCheck FAIL");
        // 自检失败 → 蜂鸣器长鸣 1 秒报警
        beepBuzzer(1000);
    }

    // 初始化 STATUS 指示灯
    pinMode(STUTS, OUTPUT);
    digitalWrite(STUTS, LOW);

    // ── 电源指示灯：BAT_VOLTAGE_PIN 输出低电平常亮 ────────
    pinMode(BAT_VOLTAGE_PIN, OUTPUT);
    digitalWrite(BAT_VOLTAGE_PIN, LOW);

}
void loop()
{
    // ── STATUS 指示灯：收到 cmd_vel 后亮 150ms 后熄灭 ────
    if (millis() - lastCmdReceivedMs > 150) {
        digitalWrite(STUTS, LOW);
    }

    // ── 更新 MPU 姿态数据（每次 loop 都需要调用）───────────
    updateMPU();

    // 测姿态-----------------------------------------------------------------------------------------------------------------------------------------------
    MPU6050_check();

    // 测距离------------------------------------------------------------------------------------------------------------------------------------------------------
    float distance = Distance_chect();
    Serial.print("距离：");
    Serial.print(distance);
    Serial.println("cm");

    // 测速度-----------------------------------------------------------------------------------------------------------------------------------------------
    // Encoder_Check();
    Velocity_Check();

    // ── 计算综合速度（4 轮速度绝对值的平均值）───────────────
    float combinedSpeed = (fabs(Velocity[0]) + fabs(Velocity[1])
                         + fabs(Velocity[2]) + fabs(Velocity[3])) / 4.0f;

    // Pid_controller_run(250); //测试轮子的正反

    run[0].Pid_run_();
    run[1].Pid_run_();
    run[2].Pid_run_();
    run[3].Pid_run_();

    // Servo_run();           // 舵机 UDP 控制

    // ── OLED 刷新（4 路轮速带正负号）──────────────────────
    oledDisplay.updateDisplay(Velocity);
}
