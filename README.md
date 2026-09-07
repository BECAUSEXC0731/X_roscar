# X_roscar — ESP32-S3 机器人底盘固件（micro-ROS / ROS 2）

基于 **PlatformIO + Arduino 框架** 的 ESP32-S3 **四轮差速机器人底盘**固件，通过 **micro-ROS over WiFi** 接入 ROS 2：订阅 `/cmd_vel` 运动指令，发布 `/odom`、`/imu`、`/scan` 数据。

集成电机驱动、编码器测速、PID 速度闭环、差速运动学与里程计、MPU6050 姿态、YDLIDAR X2 激光雷达、HC-SR04 超声波、OLED 显示、舵机云台、Web 配网与上电自检等功能，可作为低成本 ROS 2 机器人底盘方案。

## 功能特性

- **电机控制** — 4 路 TB6612 驱动，LEDC PWM 20kHz；支持刹车 / 滑行 / 死区补偿 / 接线反向
- **编码器测速** — 4 路 AB 相正交编码器（PCNT 硬件计数），每 tick ≈ 0.28mm
- **速度闭环** — 4 路独立增量式 PID，带缓动限幅（防抽搐）
- **运动学 & 里程计** — 差速模型正逆解 + 里程计积分（x / y / angle）
- **micro-ROS** — 订阅 `/cmd_vel`；发布 `/odom`、`/imu`、`/scan`（WiFi UDP 8888）
- **姿态** — MPU6050 六轴 + 欧拉角，I2C Bypass 旁路扩展 OLED
- **激光雷达** — YDLIDAR X2/X2L（UART1 150000 波特），独立解析任务 + 双缓冲
- **测距 / 显示 / 云台** — HC-SR04 超声波、0.96″ SSD1306 OLED、2 路舵机（UDP 4210）
- **配网 & 自检** — AP 热点 Web 配网（配置存 NVS，掉电不丢）、上电自检 + 蜂鸣器反馈

## 功能模块说明

| 模块 | 文件 | 说明 |
|------|------|------|
| 主流程 | `src/main.cpp` | 配网键判断 → 各外设初始化 → 上电自检 → 主循环 |
| Web 配网 | `src/ConfigManager.cpp` | AP 热点配网页 + NVS 读写 |
| 电机控制 | `src/Motor_control.cpp` | 4 路 LEDC PWM：运行 / 刹车 / 滑行 / 死区补偿 |
| 编码器 | `src/newencoder.cpp` | PCNT 硬件计数测速，得到 4 路轮速 |
| PID | `src/Pid_control.cpp` | 4 路独立增量式 PID + 输出缓动限幅 |
| 运动学 | `lib/Kinematics/` | 差速正逆解 + 里程计积分 |
| micro-ROS | `src/Microros.cpp` | 独立任务：等 WiFi → 建节点 → 50ms 发布，订阅 `/cmd_vel` |
| 姿态 | `src/MPU6050.cpp` | MPU6050 读取（含 I2C Bypass 使能） |
| 显示 | `src/OledDisplay.cpp` | OLED：IP / 4 路轮速 / 进度条 / 动画 |
| 测距 | `src/Distance_check.cpp` | HC-SR04 超声波测距 |
| 云台 | `src/Servo.cpp` | 2 路舵机，UDP 4210 收发 `[pan, tilt]` |
| 雷达 | `src/Lidar.cpp` | YDLIDAR X2 解析任务，发布 `/scan` |
| 自检 | `src/SelfCheck.cpp` | 上电自检，蜂鸣器 + OLED 反馈结果 |

> 各模块的实现细节（引脚映射、算法参数、数据格式）见对应源码及 `include/` 头文件注释。

## 项目结构

```
X_roscar/
├── platformio.ini      # PlatformIO 构建配置（env:xc）
├── custom.meta         # micro-ROS 自定义构建参数（MTU=2048，支持更大 /scan）
├── boards/             # 自定义板级配置
├── src/                # 主程序源码（main.cpp + 各功能模块）
├── include/            # 头文件（PinConfig.h 引脚集中配置）
├── lib/                # 自定义库（Kinematics / Pidcontraller / Driver）
├── lidar_pkg/          # ROS 2 功能包（USB 版雷达节点，备用）
├── host/               # 上位机辅助脚本
├── Example/            # 独立示例代码
├── discard/            # 废弃的旧驱动（保留参考）
└── test/
```

> 根目录另含 `问题.txt`（排障记录）、`LICENCE`、`preview_config.html`（配网页静态预览）。

## Web 配网

首次上电需通过 AP 热点完成配网（SSID / 密码 / Agent IP 存 NVS，掉电不丢）：

1. 按住 **GPIO11 按键**上电（接地）→ 清空配置并进入配网模式。
2. 手机连接热点 **`CAR_CONFIG`**（无密码）。
3. 浏览器打开 **`http://192.168.4.1`**，填写：热点 SSID、密码、Agent IP（运行 micro-ROS Agent 的电脑 IP）。
4. 保存后自动重启并连接。

> 当 NVS 为空时，固件使用 `src/main.cpp` 中 `DEFAULT_SSID / DEFAULT_PASS / DEFAULT_AGENT_IP` 编译默认值。

## 引脚定义

完整宏定义集中在 `include/PinConfig.h`，关键映射如下：

| 功能 | GPIO | 备注 |
|------|:----:|------|
| 电机 1~4 IN1/IN2/PWM | 7/15/16 · 41/40/39 · 13/42/1 · 6/5/4 | 经 TB6612；M2~M4 软件反向 |
| 编码器 0~3 A/B | 3/8 · 48/36 · 38/37 · 17/18 | PCNT 正交计数 |
| 雷达 TX / RX | 43 / 44 | UART1，150000 波特 |
| MPU6050 / OLED | 9（SDA）/ 14（SCL） | OLED 走 I2C Bypass，地址 0x3C |
| 超声波 TRIG / ECHO | 47 / 21 | HC-SR04 |
| 舵机 Pan / Tilt | 45 / 46 | 云台，UDP 控制 |
| 配网键 | 11 | 接地清配置进配网 |
| 蜂鸣器 / 电源灯 / 状态灯 | 12 / 20 / 2 | 自检与状态指示 |

> ⚠️ **原生 USB-CDC**：日志、下载、串口监视请插板子**原生 USB 口（GPIO19/20）**，不是 CP2102 的 UART 口；这样才能把 UART0（43/44）腾给雷达使用。
> ⚠️ **GPIO46** 是输入专用引脚，不能输出 PWM（详见 `问题.txt`）。
