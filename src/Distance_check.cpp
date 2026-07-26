#include <Arduino.h>
#include "Distance_check.h"
#include "PinConfig.h"

//测距函数初始化
void Distance_chect_Init()
{
    pinMode(SONIC_TRIG, OUTPUT);
    pinMode(SONIC_ECHO, INPUT); // 回波检测
    digitalWrite(SONIC_TRIG, LOW); // 初始化为低，避免误触发
}

//测距函数
float Distance_chect()
{
    static unsigned long lastTrigMs = 0;
    unsigned long now = millis();

    // HC-SR04 要求两次触发间隔 >= 60ms，否则读数不可靠
    if (now - lastTrigMs < 60) {
        return -2.0;  // 间隔太短，跳过
    }
    lastTrigMs = now;

    digitalWrite(SONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONIC_TRIG, LOW);

    unsigned long duration = pulseIn(SONIC_ECHO, HIGH, 30000); // 最长等 30ms
    if (duration == 0) {
        Serial.println("[距离] 超时：未收到回波");
        return -1.0;
    }

    float distance = duration * 0.034 / 2;
    Serial.printf("[距离] dur=%lu  dist=%.1fcm\n", duration, distance);

    if (distance < 2.0 || distance > 400.0) {
        Serial.println("[距离] 超出范围");
        return -1.0;
    }

    return distance;
}  
   