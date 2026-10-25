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

#include <Wire.h>          // 配网模式下直接点亮 OLED 需要

My_motor my_motor[4];                   // 电机实例化
PID_run run[4];                         // 使用PID控制电机
extern float Velocity[4];               // 拿到测量后的速度
extern PIDController pid_controller[4]; // PID控制器实例化

// 记录最后收到 cmd_vel 的时间戳
volatile unsigned long lastCmdReceivedMs = 0;

// ════════════════════════════════════════════════════════════
//  默认 WiFi 配置（仅当 NVS 为空时生效）
// ════════════════════════════════════════════════════════════
static const char DEFAULT_SSID[] = "doomsday";
static const char DEFAULT_PASS[] = "123123123";
static const char DEFAULT_AGENT_IP[] = "10.136.202.51";

// 正常启动等待 WiFi 的最长时间；超时自动进入配网模式（绝不无限死等）
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;

// ── 配网键 ──────────────────────────────────────────────────
// 按下/松开对应的电平（极性在 PinConfig.h 里用 CFG_BUTTON_ACTIVE_LOW 切换）
#if CFG_BUTTON_ACTIVE_LOW
  #define CFG_BTN_PRESSED  LOW
#else
  #define CFG_BTN_PRESSED  HIGH
#endif

// 连续按住多久才算一次有效按键（防抖）
static const uint32_t CFG_BUTTON_HOLD_MS = 300;

static void cfgButtonInit()
{
#if CFG_BUTTON_ACTIVE_LOW
    pinMode(CFG_BUTTON, INPUT_PULLUP);
#else
    pinMode(CFG_BUTTON, INPUT_PULLDOWN);
#endif
}

// 当前是否正被按住（连续 holdMs 都是"按下"电平）
static bool isCfgButtonHeld(uint32_t holdMs)
{
    uint32_t start = millis();
    while (millis() - start < holdMs) {
        if (digitalRead(CFG_BUTTON) != CFG_BTN_PRESSED) return false;
        delay(1);
    }
    return true;
}

// ── 进配网前刹住四个轮子 ────────────────────────────────────
// enterConfigMode() 不会返回，loop()/PID 停了但 PWM 会保持最后一个值，车会一直跑
static void stopAllMotors()
{
    for (int id = 1; id <= 4; id++) my_motor[0].Motor_Brake(id);
}

// ── 独立点亮 OLED ───────────────────────────────────────────
static void oledBeginViaBypass()
{
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.beginTransmission(0x68);   // MPU6050
    Wire.write(0x37);               // INT_PIN_CFG
    Wire.write(0x02);               // BYPASS_EN
    Wire.endTransmission();
    delay(10);

    oledDisplay.begin();            // 失败自己会打印日志，不影响后续
}

// ── 进入配网模式（永不返回）────────────────────────────────
static void enterConfigMode(const char *reason)
{
    Serial.printf("\n[配网] 进入配网模式（原因：%s）\n", reason);

    // 蜂鸣器 3 短鸣 + OLED 提示，没有串口也能确认状态
    for (int i = 0; i < 3; i++) {
        beepBuzzer(100);
        delay(150);
    }
    oledBeginViaBypass();
    oledDisplay.showInitMessage("== CONFIG MODE ==\nAP: CAR_CONFIG\n192.168.4.1");

    // 保留上次的配置作为网页默认值：不用重打一遍，
    // 也能一眼看出"刚才连不上的到底是哪个 SSID、密码长度对不对"
    configManager.startAP();

    while (true) {
        configManager.handleClient();
        delay(2);   // 让出 CPU，避免独占 core1 饿死 idle 任务 / 影响 USB-CDC
    }
}

static void checkCfgButton()
{
    static bool seenIdle = false;    // 先见到过"松开"状态，才认可后面的按下

    if (digitalRead(CFG_BUTTON) != CFG_BTN_PRESSED) {
        seenIdle = true;
        return;
    }
    if (!seenIdle) return;           // 一直是"按下"电平 → 判定为接线/极性不对，不动作
    if (!isCfgButtonHeld(CFG_BUTTON_HOLD_MS)) return;

    seenIdle = false;
    stopAllMotors();
    enterConfigMode("按下配网键");
}

void setup()
{
    Serial.begin(115200);  // 原生 USB-CDC → 插板子的 USB 口（稳定不掉线）
    delay(500);

    // 配网键：打印当前电平，用来核对极性（按下时应为 CFG_BTN_PRESSED）
    cfgButtonInit();
    delay(50);
    Serial.printf("配网键 GPIO%d 当前电平=%d，按下时应为 %d\n",
                  CFG_BUTTON, digitalRead(CFG_BUTTON), (int)CFG_BTN_PRESSED);

    //初始化配置管理器（NVS 读写，必须在 configManager 其他调用之前）
    configManager.begin();

    // NVS 为空时用编译时的默认配置
    configManager.setDefaults(DEFAULT_SSID, DEFAULT_PASS, DEFAULT_AGENT_IP);

    //已有配置就正常启动
    ConfigData cfg = configManager.getConfig();
    Serial.printf("使用配置: SSID=%s, 密码长度=%u, Agent IP=%s\n",
                  cfg.ssid, (unsigned)strlen(cfg.password), cfg.agent_ip);

    xTaskCreate(micro_ros_task, "uros_task", 32768, NULL, 1, NULL); // micro-ROS 任务

    // ── 连接 WiFi：带超时，连不上就自动进配网，绝不无限死等 ──
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.ssid, cfg.password);
    Serial.printf("正在连接热点 %s ", cfg.ssid);

    uint32_t wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - wifiStart > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.printf("\n❌ 热点 %s 连接超时(%us)\n",
                          cfg.ssid, (unsigned)(WIFI_CONNECT_TIMEOUT_MS / 1000));
            enterConfigMode("WiFi 连接超时");
        }
        delay(500);
        Serial.print("."); // wifi初始化
    }
    Serial.printf("\nWiFi connected! ESP32 IP: %s\n", WiFi.localIP().toString().c_str());

    // ── MPU6050 初始化（同时初始化 I2C 总线 & 启用 Bypass 模式）─
    MPU6050_Init(); // 初始化MPU6050

    // ── OLED 初始化（通过 MPU XDA/XCL 旁路）────────────────
    oledDisplay.begin();
    oledDisplay.showInitMessage("Initializing...");
    delay(500);
    oledDisplay.setIP(WiFi.localIP().toString().c_str());

    // Distance_chect_Init();//初始化测距模块

    Encoder_Init(); // 编码器初始化

    my_motor[0].Motor_Setup(); // 初始化电机引脚和PWM

    Pid_controller_init(); // PID控制器初始化

    Lidar_Init();          // 雷达初始化 (UART1, 150000 波特)

    // Servo_init();          // 舵机初始化

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
}

void loop()
{
    // ── 配网键：运行中按下就进配网（刹车后不返回）──────────
    checkCfgButton();

    // ── STATUS 指示灯：收到 cmd_vel 后亮 150ms 后熄灭 ────
    if (millis() - lastCmdReceivedMs > 150) {
        digitalWrite(STUTS, LOW);
    }

    // ── 更新 MPU 姿态数据（每次 loop 都需要调用）───────────
    updateMPU();
    //MPU6050_check();

    // ── 测速 + 四轮 PID 闭环 ──────────────────────────────
    Velocity_Check();

    run[0].Pid_run_();
    run[1].Pid_run_();
    run[2].Pid_run_();
    run[3].Pid_run_();

    // Servo_run();           // 舵机 UDP 控制

    // ── OLED 刷新（4 路轮速带正负号）──────────────────────
    oledDisplay.updateDisplay(Velocity);
}
