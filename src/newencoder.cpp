#include <Arduino.h>
#include <ESP32Encoder.h>
#include "PinConfig.h"
// 创建 4 个编码器对象
ESP32Encoder encoders[4];

// 编码器初始化
void Encoder_Init()
{
    // 启用内部弱上拉（多数编码器需要）
    ESP32Encoder::useInternalWeakPullResistors =puType::up;

    // 初始化 4 个编码器
    encoders[3].attachHalfQuad(ENCODER3_A, ENCODER3_B);
    encoders[1].attachHalfQuad(ENCODER1_A, ENCODER1_B);


    encoders[0].attachHalfQuad(ENCODER0_A, ENCODER0_B);
    encoders[2].attachHalfQuad(ENCODER2_A, ENCODER2_B);

    for (int i = 0; i < 4; i++){
        encoders[i].setFilter(100); 
    }
}

// 编码器计数
void Encoder_Check()
{
    delay(10);
    int64_t t0 = encoders[0].getCount();
    int64_t t1 = encoders[1].getCount();
    int64_t t2 = encoders[2].getCount();
    int64_t t3 = encoders[3].getCount();

    //打印调试
    //Serial.printf("tick1=%lld,tick2=%lld,tick3=%lld,tick4=%lld\n", t0,t1, t2, t3 );//串口

}


int64_t last_encoder[4] = {0};//上一次编码器值
int64_t last_time = 0;//上一次时间
float Velocity[4] = {0};//四个轮子的速度

//获取轮子的速度
void Velocity_Check()
{
    int64_t now = millis();
    float dt = (now - last_time) / 1000.0f; // 秒

    for (int i = 0; i < 4; i++) {
        int64_t current = encoders[i].getCount();
        float delta = (current - last_encoder[i]) * 0.28f; // 每tick 0.5mm
        Velocity[i] = dt > 0 ? delta / dt : 0;
        last_encoder[i] = current;
    }
    last_time = now;


    //打印调试
    Serial.printf("Velocity1=%.3f,Velocity2=%.3f,Velocity3=%.3f,Velocity4=%.3f\n",Velocity[0], Velocity[1], Velocity[2], Velocity[3]);
              
}