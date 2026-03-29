#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// === UDP 配置 ===
unsigned int localPort = 4210;      // 本地监听端口
WiFiUDP udp;
const int packetSize = 2;           // 每次收 2 字节：[pan, tilt]

// === 舵机引脚 ===
const int PAN_PIN = 18;
const int TILT_PIN = 17;
Servo panServo;
Servo tiltServo;

void Servo_init() {
    

    // 初始化舵机
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    panServo.setPeriodHertz(50);
    tiltServo.setPeriodHertz(50);
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    panServo.write(90);
    tiltServo.write(90);

    // 启动 UDP
    udp.begin(localPort);
    
}

void Servo_run() {
    int packetLen = udp.parsePacket();
    if (packetLen == packetSize) {
        uint8_t buffer[2];
        udp.read(buffer, packetSize);

        int pan = buffer[0];   // 第一个字节：0~255 → 我们只用 0~180
        int tilt = buffer[1];  // 第二个字节

        // 安全限制
        if (pan > 180) pan = 180;
        if (tilt > 180) tilt = 180;

        panServo.write(pan);
        tiltServo.write(tilt);

        // 打印调试信息
        Serial.print("UDP Received: pan=");
        Serial.print(pan);
        Serial.print(", tilt=");
        Serial.println(tilt);

       
    }

    delay(10); // 避免 CPU 占用 100%
}