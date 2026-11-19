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

#include "OledDisplay.h"
#include "PinConfig.h"

// 全局单例
OledDisplay oledDisplay;

// ============================================================
// 构造函数
// ============================================================
OledDisplay::OledDisplay()
    : display_(OLED_SCREEN_W, OLED_SCREEN_H, &Wire, OLED_RESET_PIN)
    , initialized_(false)
    , lastUpdateMs_(0)
{
    ip_[0] = '\0';
}

// ============================================================
// 初始化 OLED
// Wire 由 MPU6050_Init() 预先初始化（含 I2C Bypass 模式）
// ============================================================
bool OledDisplay::begin()
{
    Wire.setClock(400000);      // 提升 I2C 速度到 400kHz
    delay(50);

    if (!display_.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("[OLED] SSD1306 初始化失败！检查接线");
        return false;
    }

    initialized_ = true;
    display_.clearDisplay();
    display_.setTextSize(1);
    display_.setTextColor(SSD1306_WHITE);
    display_.cp437(true);       // 支持完整 ASCII
    display_.display();

    Serial.println("[OLED] SSD1306 初始化成功");
    return true;
}

// ============================================================
// 显示初始化消息
// ============================================================
void OledDisplay::showInitMessage(const char *msg)
{
    if (!initialized_) return;

    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.setTextSize(1);
    display_.println(msg);
    display_.display();
}

// ============================================================
// 设置 IP 地址
// ============================================================
void OledDisplay::setIP(const char *ip)
{
    strncpy(ip_, ip, sizeof(ip_) - 1);
    ip_[sizeof(ip_) - 1] = '\0';
}

// ============================================================
// 清屏
// ============================================================
void OledDisplay::clear()
{
    if (!initialized_) return;
    display_.clearDisplay();
    display_.display();
}

// ============================================================
// 刷新界面：IP + 4 路轮速（带正负号）+ 航向角
// 限频 10Hz（100ms）
//
// 布局（128×64 OLED）：
//   行0 (y=0)  : IP: xxx.xxx.xxx.xxx
//   行1 (y=14) : M1:+123  M2:-456
//   行2 (y=30) : M3:+078  M4:-090
//   行3 (y=44) : YAW:+123.4  MPU6050 偏航角
//   行4 (y=54) : UP: HH:MM:SS 启动后运行时间
// ============================================================
void OledDisplay::updateDisplay(const float speeds[4], float yawDeg)
{
    if (!initialized_) return;

    unsigned long now = millis();
    // 整屏 I2C 刷新会阻塞约 20~30ms，降到 10Hz 以减小对控制环的干扰
    if (now - lastUpdateMs_ < 100) return;  // 10Hz 限频
    lastUpdateMs_ = now;

    display_.clearDisplay();
    char buf[22];

    // ── 第 0 行：IP 地址 ──────────────────────────────────
    display_.setCursor(0, 0);
    display_.setTextSize(1);
    display_.print("IP: ");
    display_.println(ip_);

    // ── 第 1 行：M1 和 M2 轮速 ────────────────────────────
    display_.setCursor(0, 14);
    snprintf(buf, sizeof(buf), "M1:%+5.0f  M2:%+5.0f", speeds[0], speeds[1]);
    display_.println(buf);

    // ── 第 1 行下方画小进度条：M1 ─────────────────────────
    int barY1 = 22;
    drawMiniBar(0, barY1, 60, 4, speeds[0], 300.0f);
    // M2 进度条在右侧
    drawMiniBar(66, barY1, 60, 4, speeds[1], 300.0f);

    // ── 第 2 行：M3 和 M4 轮速 ────────────────────────────
    display_.setCursor(0, 30);
    snprintf(buf, sizeof(buf), "M3:%+5.0f  M4:%+5.0f", speeds[2], speeds[3]);
    display_.println(buf);

    // ── 第 2 行下方画小进度条：M3、M4 ─────────────────────
    int barY2 = 38;
    drawMiniBar(0, barY2, 60, 4, speeds[2], 300.0f);
    drawMiniBar(66, barY2, 60, 4, speeds[3], 300.0f);

    // ── 第 3 行：MPU6050 航向角 ──────────────────────────────
    snprintf(buf, sizeof(buf), "YAW:%+6.1f", yawDeg);
    display_.setCursor(0, 44);
    display_.print(buf);

    // ── 底部：启动后运行时间 ────────────────────────────────
    unsigned long upSec = millis() / 1000;
    snprintf(buf, sizeof(buf), "UP: %02lu:%02lu:%02lu",
             upSec / 3600, (upSec / 60) % 60, upSec % 60);
    display_.setCursor(0, 54);
    display_.print(buf);

    display_.display();
}

// ============================================================
// 绘制小进度条（用于单个轮速）
// ============================================================
void OledDisplay::drawMiniBar(int x, int y, int w, int h, float speed, float maxSpeed)
{
    float ratio = fabs(speed) / maxSpeed;
    if (ratio > 1.0f) ratio = 1.0f;
    int fillW = (int)(ratio * (w - 2));
    display_.drawRect(x, y, w, h, SSD1306_WHITE);
    if (fillW > 0) {
        display_.fillRect(x + 1, y + 1, fillW, h - 2, SSD1306_WHITE);
    }
}
