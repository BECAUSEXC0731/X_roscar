#include <Arduino.h>
// 定义引脚
#define TRIG 13 // 触发引脚
#define ECHO 45 // 回波引脚

//测距函数初始化
void Distance_chect_Init()
{
   
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT); // 回波检测
    digitalWrite(TRIG, LOW); // 初始化为低，避免误触发
   
}

//测距函数
float Distance_chect()
{
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
     
    double duration = pulseIn(ECHO, HIGH,30000);//检测声波来回的时间
    if (duration == 0) {
        return -1.0;  
    }
    float distance = duration * 0.034 / 2;
   

    
     if (distance < 2.0 || distance > 400.0) {
        return -1.0;
    }
     Serial.print("成功：");
    return distance;
}  
   