# X_roscar — ESP32-S3 机器人底盘控制项目

基于 **PlatformIO + Arduino 框架** 的 ESP32-S3 机器人运动控制平台。
集成电机控制（TB6612）、编码器测速、PID 闭环控制、运动学解算、里程计、
micro-ROS (ROS2) 通信、IMU 姿态检测、超声波测距、OLED 显示、
舵机云台控制、Web 配网、电池电压监测、上电自检等功能。

---

## 硬件平台

| 组件 | 说明 |
|------|------|
| **主控** | ESP32-S3 (ESP32-S3-DevKitC-1) |
| **电机驱动** | 4 × TB6612 直流有刷电机驱动 |
| **编码器** | 4 路 AB 相增量式编码器 (ESP32 PCNT 硬件计数) |
| **姿态传感器** | MPU6050 (I2C) — 加速度计 + 陀螺仪 |
| **测距模块** | HC-SR04 超声波测距 |
| **OLED 显示屏** | 0.96" SSD1306 (I2C，通过 MPU6050 XDA/XCL 旁路) |
| **舵机** | 2 路舵机 (Pan/Tilt 云台) |
| **蜂鸣器** | 有源蜂鸣器 (GPIO12) |
| **状态指示灯** | 板载 LED (GPIO2) — 收到 /cmd_vel 时闪烁 |
| **电池监测** | ADC 分压采集 (GPIO20) |
| **通信** | WiFi (micro-ROS / Web 配网) |
| **配置按钮** | GPIO43 → GND（上电按下清空配置并进入配网模式） |

---

## 项目结构

```
X_roscar/
├── platformio.ini              # PlatformIO 构建配置
├── boards/                     # 自定义板级配置
│   └── fishbot_motion_control_four_driver_v2.json
│
├── src/                        # 主程序源文件
│   ├── main.cpp                # 程序入口 — 配网判断 → 初始化 → 自检 → loop
│   ├── ConfigManager.cpp       # Web 配网页面 + NVS 闪存读写
│   ├── Motor_control.cpp       # 电机初始化 (LEDC 20kHz) + 运行 + 刹车
│   ├── newencoder.cpp          # 编码器初始化与速度测量
│   ├── Pid_control.cpp         # PID 控制器封装 (4路独立PID)
│   ├── Microros.cpp            # micro-ROS 节点 (订阅 /cmd_vel, 发布 /odom + /battery_voltage)
│   ├── Kinematics.cpp          # 运动学正逆解 + 里程计更新
│   ├── MPU6050.cpp             # MPU6050 姿态检测（含 I2C Bypass 启用）
│   ├── OledDisplay.cpp         # SSD1306 OLED 显示 (IP + 速度条形图)
│   ├── Battery.cpp             # 电池电压 ADC 采集
│   ├── SelfCheck.cpp           # 上电自检 + 蜂鸣器控制
│   ├── Distance_check.cpp      # HC-SR04 超声波测距
│   ├── Servo.cpp               # 舵机云台 UDP 控制
│   └── ...                     # ESP32Encoder 等
│
├── include/                    # 头文件
│   ├── PinConfig.h             # 引脚集中配置（所有 GPIO 宏定义）
│   ├── ConfigManager.h         # 配网管理器 (AP + WebServer + NVS)
│   ├── Motor_control.h         # 电机类 (Setup / Run / Brake / Coast)
│   ├── newencoder.h            # 编码器接口 (Init / Check / Velocity)
│   ├── Pid_control.h           # PID 控制器类
│   ├── Microros.h              # micro-ROS 任务声明
│   ├── Kinematics.h            # 运动学 / 里程计结构体
│   ├── MPU6050.h               # MPU6050 接口
│   ├── OledDisplay.h           # OLED 显示接口
│   ├── Battery.h               # 电池电压采集接口
│   ├── SelfCheck.h             # 自检 + 蜂鸣器接口
│   ├── Distance_check.h        # 测距接口
│   ├── Servo.h                 # 舵机接口
│   └── Netprint.h              # 网络打印接口
│
├── lib/                        # 自定义库
│   ├── Kinematics/             # 运动学解算库
│   ├── Pidcontraller/          # PID 控制器库
│   └── Driver/                 # 底层驱动 (MPU6050_light, ESP32Encoder, Esp32McpwmMotor)
│
├── Example/                    # 独立示例代码
│   ├── control2motor.cpp       # 双电机开环控制示例
│   ├── control4motor.cpp       # 四电机开环控制示例
│   ├── GetAllData/             # MPU6050 全数据读取 (Arduino IDE)
│   └── GetAngle/               # MPU6050 角度读取 + Python 动态绘图
│
└── test/
```

---

## 快速开始

### 1. 安装 PlatformIO

在 VS Code 中安装 **PlatformIO IDE** 扩展。

### 2. 编译 & 上传

```bash
pio run --target upload
```

### 3. 第一次使用 — Web 配网

项目内置了 AP 配网功能，**无需修改代码中的 WiFi 配置**：

| 步骤 | 操作 |
|:----:|------|
| ① | 给 ESP32 上电 |
| ② | 手机搜索 WiFi，连接 **`CAR_CONFIG`**（无密码） |
| ③ | 打开手机浏览器访问 **`http://192.168.4.1`** |
| ④ | 填写：手机热点名称、密码、Agent IP |
| ⑤ | 点击保存 → ESP32 自动重启连接 |

配置存储在 ESP32 的 NVS 闪存中，掉电不丢失。

### 4. 换热点 / 重新配网

按住 **GPIO43 → GND 的按钮** 上电，即可清空配置并重新进入配网模式。

### 5. 查看串口输出

```bash
pio device monitor
```

> **注意**：`platformio.ini` 中设置 `build_flags = -D ARDUINO_USB_CDC_ON_BOOT=1`，ESP32-S3 使用 USB 串口输出日志，波特率 `115200`。

---

## 引脚定义

所有引脚宏定义集中在 `include/PinConfig.h` 中：

| 功能 | 引脚宏 | GPIO |
|------|--------|:----:|
| 电机1 IN1/IN2/PWM | `MOTOR1_IN1/IN2/PWM` | 7 / 15 / 16 |
| 电机2 IN1/IN2/PWM | `MOTOR2_IN1/IN2/PWM` | 40 / 41 / 39 |
| 电机3 IN1/IN2/PWM | `MOTOR3_IN1/IN2/PWM` | 13 / 42 / 1 |
| 电机4 IN1/IN2/PWM | `MOTOR4_IN1/IN2/PWM` | 5 / 6 / 4 |
| 编码器0 A/B | `ENCODER0_A/B` | 3 / 8 |
| 编码器1 A/B | `ENCODER1_A/B` | 48 / 36 |
| 编码器2 A/B | `ENCODER2_A/B` | 38 / 37 |
| 编码器3 A/B | `ENCODER3_A/B` | 17 / 18 |
| 超声波 TRIG/ECHO | `SONIC_TRIG/ECHO` | 47 / 21 |
| MPU6050 SDA/SCL | `MPU_SDA/SCL` | 9 / 14 |
| OLED SDA/SCL | `OLED_SDA/SCL` | 9 / 14 (与 MPU 共用) |
| 舵机 Pan/Tilt | `SERVO_PAN/TILT` | 46 / 45 |
| 蜂鸣器 | `BUZZER_PIN` | 12 |
| 电池电压 ADC | `BAT_VOLTAGE_PIN` | 20 |
| 状态指示灯 | `STUTS` | 2 |
| 配网按钮 | `CFG_BUTTON` | 43 |

---

## 功能模块说明

### 🌐 Web 配网 (`ConfigManager.cpp`)

| 功能 | 说明 |
|------|------|
| AP 热点 | 名称 `CAR_CONFIG`，无密码 |
| Web 页面 | 浏览器访问 `http://192.168.4.1`，内嵌 HTML 可自定义 |
| 保存内容 | WiFi SSID、密码、Agent IP |
| 存储位置 | ESP32 NVS 闪存（掉电不丢） |
| 清空方式 | GPIO43 低电平上电 或 代码调 `clearConfig()` |

```cpp
configManager.begin();              // 初始化，读取 NVS
configManager.isConfigured();       // 检查是否有有效配置
configManager.getConfig();          // 获取 ConfigData {ssid, password, agent_ip}
configManager.startAP();            // 开启 AP + Web 服务器
configManager.handleClient();       // 处理 Web 请求
configManager.clearConfig();        // 清空 NVS 配置
```

> **自定义配网页面的图片/文字**：编辑 `ConfigManager.cpp` 中 `handleRoot()` 函数的 HTML 内容（`R"rawliteral(...)rawliteral"` 包裹部分）。可用 Emoji 或 Base64 图片，需保留三段动态插值代码（`config_.ssid` / `config_.password` / `config_.agent_ip`）。

---

### 🏎️ 电机控制 (`Motor_control.cpp`)

4 路直流电机通过 **TB6612** 驱动。PWM 使用 **ESP32 LEDC 硬件 PWM**，频率 **20kHz**。

**引脚映射：**

| 电机 | IN1 | IN2 | PWM | LEDC 通道 | 接线反向 |
|------|-----|-----|-----|:---------:|:--------:|
| 电机1 | GPIO7  | GPIO15 | GPIO16 | 0 | ❌ 否 |
| 电机2 | GPIO40 | GPIO41 | GPIO39 | 1 | ✅ 是 |
| 电机3 | GPIO13 | GPIO42 | GPIO1  | 2 | ✅ 是 |
| 电机4 | GPIO5  | GPIO6  | GPIO4  | 3 | ✅ 是 |

**关键特性：**

| 特性 | 说明 |
|------|------|
| TB6612 刹车 | `pwm ≈ 0` 时自动刹车 (IN1=H, IN2=H)，急停 |
| TB6612 滑行 | `Motor_Coast()` 释放电机 (IN1=L, IN2=L) |
| 死区补偿 | 低于 `MOTOR_DEAD_ZONE=5` 的 PWM 自动提升，防止电机堵转 |
| 接线反向 | `reverse=true` 的电机在软件层面自动取反 IN1/IN2 |

```cpp
my_motor[0].Motor_Setup();           // 初始化所有电机的引脚 + LEDC PWM
my_motor[i].Motor_Run(id, pwm);      // 运行电机：-100 ~ 100，自动刹车
my_motor[i].Motor_Brake(id);         // 急停
my_motor[i].Motor_Coast(id);         // 自由滑行
```

如需调整方向，修改 `motor_cfg[]` 数组中的 `reverse` 字段：

```cpp
static const motor_cfg_t motor_cfg[] = {
    {   7,  15,  16,  0,   false },  // 电机1
    {  40,  41,  39,  1,   true  },  // ← 改成 false 即反转方向
    // ...
};
```

---

### 🔄 编码器测速 (`newencoder.cpp`)

使用 ESP32 硬件 PCNT 模块读取 4 路 AB 相编码器，并计算轮速 (mm/s)。

| 编码器 | A相 | B相 |
|--------|-----|-----|
| 编码器0 | GPIO3  | GPIO8  |
| 编码器1 | GPIO48 | GPIO36 |
| 编码器2 | GPIO38 | GPIO37 |
| 编码器3 | GPIO17 | GPIO18 |

```cpp
Encoder_Init();         // 初始化编码器
Encoder_Check();        // 读取编码器计数值 (调试用)
Velocity_Check();       // 计算各轮速度 (mm/s)，结果存入 Velocity[4]
```

---

### 🎯 PID 控制 (`Pid_control.cpp` + `lib/Pidcontraller/`)

四路独立 PID 控制器，对每个电机的实际速度进行闭环调节。

**PID 参数（在 `Pid_control.cpp` 中定义）：**

| 电机 | Kp  | Ki  | Kd  | 输出限幅 |
|------|-----|-----|-----|---------|
| 电机1 | 0.30 | 0.07 | 0.10 | ±100 |
| 电机2 | 0.30 | 0.05 | 0.05 | ±100 |
| 电机3 | 0.40 | 0.08 | 0.05 | ±100 |
| 电机4 | 0.30 | 0.08 | 0.04 | ±100 |

```cpp
Pid_controller_init();               // 初始化 PID 参数
run[i].Pid_setgoal(id, target);      // 设置目标速度 (mm/s)
run[i].Pid_run_();                   // 执行一次 PID 运算并输出 PWM
```

---

### 📐 运动学 & 里程计 (`Kinematics.cpp`)

**运动学逆解**：目标线速度 + 角速度 → 左右轮速

```cpp
Kinematics_down(linear_mm_s, angular_rad_s, &out_MotorL, &out_MotorR);
```

**运动学正解**：实测轮速 → 机器人线速度 + 角速度

```cpp
Kinematics_up(M1, M2, M3, M4, &angular, &linear);
```

**里程计**：基于正解结果，积分更新位置 (x, y, angle)

```cpp
odom_update();    // 更新里程计数据
```

里程计数据存储在 `odom_t` 结构体中：

```cpp
typedef struct {
    float x, y;          // 位置坐标 (mm)
    float angle;         // 朝向角 (rad)
    float linear_speed;  // 线速度 (mm/s)
    float angular_speed; // 角速度 (rad/s)
} odom_t;
```

> 轮距 `wheel_distance = 165mm` 定义在 `Kinematics.cpp` 中，请根据实际车体修改。

---

### 🌐 micro-ROS (ROS2) 通信 (`Microros.cpp`)

通过 WiFi 与 ROS2 上位机通信，在独立 FreeRTOS 任务中运行。

**订阅话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 接收运动指令 |

**发布话题：**

| 话题 | 类型 | 频率 | 说明 |
|------|------|:----:|------|
| `/odom` | `nav_msgs/Odometry` | 20Hz | 里程计数据 |
| `/battery_voltage` | `std_msgs/Float32` | 20Hz | 电池电压 (V) |

```bash
# 上位机发送运动指令
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 查看里程计
ros2 topic echo /odom

# 查看电池电压
ros2 topic echo /battery_voltage
```

**启动 Agent：**

```bash
# WSL / Linux 中
source /opt/ros/humble/setup.bash
micro_ros_agent udp4 --port 8888
```

**STATUS 指示灯**：收到 `/cmd_vel` 时 GPIO2 (STUTS) 点亮，150ms 无新指令自动熄灭。

---

### 📡 MPU6050 姿态检测 + OLED 显示 (`MPU6050.cpp` + `OledDisplay.cpp`)

**I2C 总线架构**（重要特性）：

```
ESP32 GPIO9(SDA) ──── MPU6050 SDA
ESP32 GPIO14(SCL) ──── MPU6050 SCL
MPU6050 XDA ──────── OLED SDA
MPU6050 XCL ──────── OLED SCL
```

MPU6050 启用 **I2C Bypass 模式**（寄存器 `0x37` 设 `0x02`），将主 I2C 总线直通到 XDA/XCL，ESP32 通过一组引脚同时控制 MPU6050 和 OLED。

**MPU6050 初始化流程：**
- `Wire.begin(9, 14)` 初始化 I2C 总线
- `mpu.begin()` 检测 MPU6050
- `mpu.writeData(0x37, 0x02)` 启用 I2C 旁路模式

```cpp
MPU6050_Init();       // 初始化 MPU6050 并启用 I2C Bypass
MPU6050_check();      // 每秒串口打印加速度/角速度/角度数据
```

**OLED 显示内容（20Hz 刷新）：**
- 第 0 行：IP 地址
- 分隔线
- 4 路速度条形图（M1 ~ M4，带数值）

```cpp
oledDisplay.begin();                  // 初始化 OLED
oledDisplay.showInitMessage("...");   // 显示初始化消息
oledDisplay.setIP("192.168.x.x");     // 设置 IP
oledDisplay.updateSpeeds(speeds);     // 刷新速度条形图
```

---

### 🔋 电池电压监测 (`Battery.cpp`)

使用 GPIO20 (ADC) 采集电池电压，通过分压电阻换算实际电压。

| 参数 | 值 |
|------|:---:|
| ADC 引脚 | GPIO20 |
| ADC 分辨率 | 12-bit (0~4095) |
| 参考电压 | 3.3V |
| 默认分压比 | 2.0 (需根据实际电阻修改) |
| 采样方式 | 8 次取平均 |
| 发布话题 | `/battery_voltage` (20Hz) |

```cpp
float voltage = readBatteryVoltage();  // 返回电池电压 (V)
```

> ⚠️ **重要**：`Battery.cpp` 中 `VOLTAGE_DIVIDER_RATIO` 默认为 2.0，假设使用两个 100kΩ 电阻分压。请根据你的实际分压电路修改该值：
> ```cpp
> #define VOLTAGE_DIVIDER_RATIO  2.0f   // 改为你的实际分压比
> ```

---

### ✅ 上电自检 (`SelfCheck.cpp`)

上电后 WiFi 连接完成、所有外设初始化完毕后自动执行。

| 检查项 | 说明 |
|--------|------|
| 蜂鸣器引脚 | 配置为 OUTPUT |
| STATUS LED 引脚 | 配置为 OUTPUT |
| 电池 ADC 引脚 | 配置为 INPUT |
| MPU6050 | 初始化中检测（失败会 while 阻塞） |
| 编码器 | 硬件初始化 |
| 电机 | PWM 和引脚初始化 |

**自检结果反馈：**

| 自检结果 | 蜂鸣器 | OLED 显示 |
|----------|:------:|:---------:|
| ✅ 全部通过 | 短鸣 200ms | `SelfCheck OK` |
| ❌ 存在异常 | 长鸣 1000ms | `SelfCheck FAIL` |

```cpp
bool ok = runSelfCheck();     // 执行自检
beepBuzzer(200);              // 蜂鸣 200ms
beepBuzzer(1000);             // 蜂鸣 1000ms
```

---

### 📏 超声波测距 (`Distance_check.cpp`)

HC-SR04 测距模块 (TRIG=GPIO47, ECHO=GPIO21)。

```cpp
Distance_chect_Init();         // 初始化
float dist = Distance_chect(); // 测量距离 (cm)，返回 -1.0 表示无效
```

---
### 🎮 舵机云台 (`Servo.cpp`)

通过 UDP (本地端口 4210) 接收 2 字节数据控制 Pan/Tilt 舵机。

| 舵机 | GPIO |
|:----:|:----:|
| Pan  | 46 |
| Tilt | 45 |

```python
# Python 控制示例
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(bytes([90, 45]), ("<ESP32_IP>", 4210))
```

---

## 依赖库

项目在 `platformio.ini` 的 `lib_deps` 中声明：

| 库 | 用途 |
|---|------|
| `micro_ros_platformio` (Gitee 镜像) | micro-ROS 客户端 |
| `Adafruit GFX Library` | 图形基库 |
| `Adafruit SSD1306` | SSD1306 OLED 驱动 |
| `ESP32Servo` | 舵机驱动 |

---

## 常见问题

### 1. 串口无输出
确认 `platformio.ini` 中已添加：
```
build_flags = -D ARDUINO_USB_CDC_ON_BOOT=1
```
且使用 USB 口（而非 UART）连接。

### 2. 如何重新配网？
**方法一**：按住 GPIO43 → GND 的按钮，按复位键或重新上电。  
**方法二**：代码中调用 `configManager.clearConfig();` 后重启。

### 3. micro-ROS 连不上 Agent
- 确认 ESP32 和电脑连接的是**同一个手机热点**
- 在电脑上运行 Agent：
  ```bash
  micro_ros_agent udp4 --port 8888
  ```
- 如果换了热点，重新配网更新 Agent IP

### 4. 电机不转或方向反
- 检查电机引脚接线与 `motor_cfg[]` 数组是否一致
- 修改对应电机的 `reverse` 字段来调方向
- 检查 PID 输出限幅（默认 ±100）

### 5. 里程计不准
- 校准 `wheel_distance`（轮距，当前 165mm）
- 校准编码器每 tick 对应距离（`newencoder.cpp` 中 `0.28f`）
- 检查轮胎是否打滑

### 6. 电池电压读数不准
- 检查分压电阻值，修改 `Battery.cpp` 中的 `VOLTAGE_DIVIDER_RATIO`
- 用万用表实测电池电压，反推正确的分压比

### 7. OLED 不显示
- 确认 MPU6050 的 I2C Bypass 模式已启用（串口应有 `I2C Bypass enabled` 日志）
- 检查 OLED I2C 地址是否为 `0x3C`
- 检查 XDA/XCL 接线

### 8. 上电时电机抖动一下
这是 ESP32 复位期间 GPIO 浮空导致的正常现象。如需消除，可在 TB6612 的 IN1/IN2 引脚加 10kΩ 下拉电阻到 GND。
