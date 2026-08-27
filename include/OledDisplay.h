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

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SCREEN_W  128
#define OLED_SCREEN_H  64
#define OLED_RESET_PIN -1   // I2C 模式无需复位引脚

class OledDisplay {
public:
    OledDisplay();

    // 初始化 OLED（Wire.begin(11,12) 在内部调用）
    bool begin();

    // 一次性显示初始化消息
    void showInitMessage(const char *msg);

    // 设置 IP 地址（在 WiFi 连接后调用）
    void setIP(const char *ip);

    // 刷新界面：IP + 4 路轮速（带正负号），限频 20Hz
    void updateDisplay(const float speeds[4]);

    // 清屏
    void clear();

private:
    Adafruit_SSD1306 display_;
    bool initialized_;
    char ip_[16];
    unsigned long lastUpdateMs_;
    unsigned int animFrame_;  // 底部动画帧计数器
    void drawMiniBar(int x, int y, int w, int h, float speed, float maxSpeed);
};

extern OledDisplay oledDisplay;

#endif
