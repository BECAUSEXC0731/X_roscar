# X_roscar — ESP32-S3 机器人底盘固件（micro-ROS / ROS 2）

基于 **PlatformIO + Arduino 框架** 的 ESP32-S3 四轮差速机器人底盘控制固件。
集成电机驱动（TB6612）、编码器测速、PID 闭环、运动学解算与里程计、
micro-ROS（ROS 2）通信（订阅 `/cmd_vel`，发布 `/odom` `/imu` `/scan`）、
MPU6050 姿态、YDLIDAR X2 激光雷达、HC-SR04 超声波、OLED 显示、舵机云台、
Web 配网与上电自检等功能。

---

## 功能特性

| 类别 | 说明 |
|------|------|
| 电机控制 | 4 路直流电机，LEDC PWM 20kHz；支持刹车 / 滑行 / 死区补偿 / 接线反向 |
| 编码器测速 | 4 路 AB 相正交编码器（ESP32 PCNT 硬件计数），每 tick ≈ 0.28mm |
| 速度闭环 | 4 路独立增量式 PID，带输出缓动限幅（防抽搐） |
| 运动学 | 差速模型正逆解 + 里程计积分（x / y / angle） |
| ROS 2 | micro-ROS over WiFi：订阅 `/cmd_vel`，发布 `/odom`、`/imu`、`/scan` |
| 姿态 | MPU6050 六轴（加速度计 + 陀螺仪 + 欧拉角），I2C Bypass 扩展 OLED |
| 雷达 | YDLIDAR X2/X2L（UART1 150000 波特），独立解析任务 + 双缓冲，发布 200 点 `/scan` |
| 测距 | HC-SR04 超声波（2 ~ 400cm） |
| 显示 | 0.96" SSD1306 OLED：IP + 4 路轮速 + 进度条 + 动画 |
| 云台 | 2 路舵机 Pan / Tilt，UDP 4210 控制 |
| 配网 | AP 热点 Web 配网（SSID / 密码 / Agent IP 存 NVS，掉电不丢） |
| 自检 | 上电自检 + 蜂鸣器 / OLED 结果反馈 |
| 指示 | 状态灯（收到 `/cmd_vel` 点亮 150ms）、电源指示灯（常亮） |

---

## 硬件平台

| 组件 | 型号 / 说明 |
|------|------|
| 主控 | ESP32-S3-DevKitC-1（原生 USB-CDC 调试） |
| 电机驱动 | 4 × TB6612 直流电机驱动 |
| 编码器 | 4 路 AB 相增量式编码器 |
| 姿态 | MPU6050（I2C） |
| 雷达 | YDLIDAR X2 / X2L（TTL UART） |
| 测距 | HC-SR04 超声波 |
| 显示 | 0.96" SSD1306 OLED（I2C 地址 0x3C） |
| 云台 | 2 路舵机（Pan / Tilt） |
| 其他 | 有源蜂鸣器、状态灯、电源指示灯、配网按键 |

---

## 快速开始

### 1. 环境与编译

PlatformIO 项目，环境名为 `xc`。

> ⚠️ 系统 PATH 里的 `pio` 可能是废弃版本（v4.x，会报 `Unknown platform espressif32`）。
> 请使用 PlatformIO 自带的 Python 环境：
> ```bash
> ~/.platformio/penv/bin/pio run             # 编译
> ~/.platformio/penv/bin/pio run -t upload   # 烧录
> ~/.platformio/penv/bin/pio device monitor  # 串口监视
> ```

### 2. 串口 / 烧录注意事项（重要）

`platformio.ini` 启用了原生 USB-CDC：

```ini
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
```

- 串口日志、下载、监视器都要插板子的 **原生 USB 口（GPIO19/20）**，**不是** CP2102 的 UART 口。
- 波特率 `115200`。
- 这样把 UART0（GPIO43/44）腾出来给激光雷达使用。

### 3. 第一次使用：Web 配网

上电时按住 **GPIO11 按键**（接地）→ 清空配置并进入配网模式：

1. 手机连接 WiFi 热点 **`CAR_CONFIG`**（无密码）。
2. 浏览器访问 **`http://192.168.4.1`**。
3. 填写：手机热点 SSID、密码、Agent IP（运行 micro_ros_agent 的电脑 IP）。
4. 点击保存 → 自动重启并连接。

配置保存在 NVS 闪存，掉电不丢。代码内置默认配置（`main.cpp`：SSID `doomsday` / 密码 `123123123` / Agent IP `10.87.104.51`），仅当 NVS 为空时生效。

### 4. 上位机（micro-ROS Agent）

电脑与 ESP32 连**同一个热点**，启动 Agent：

```bash
micro_ros_agent udp4 --port 8888
```

然后 `ros2 topic list` 可看到：

| 方向 | 话题 | 类型 |
|:----:|------|------|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/odom` | `nav_msgs/Odometry` |
| 发布 | `/imu` | `sensor_msgs/Imu` |
| 发布 | `/scan` | `sensor_msgs/LaserScan` |

---

## 引脚定义

所有引脚宏集中在 `include/PinConfig.h`：

| 功能 | 宏 | GPIO |
|------|----|:----:|
| 电机1 IN1 / IN2 / PWM | `MOTOR1_IN1/IN2/PWM` | 7 / 15 / 16 |
| 电机2 IN1 / IN2 / PWM | `MOTOR2_IN1/IN2/PWM` | 41 / 40 / 39 |
| 电机3 IN1 / IN2 / PWM | `MOTOR3_IN1/IN2/PWM` | 13 / 42 / 1 |
| 电机4 IN1 / IN2 / PWM | `MOTOR4_IN1/IN2/PWM` | 6 / 5 / 4 |
| 编码器0 A / B | `ENCODER0_A/B` | 3 / 8 |
| 编码器1 A / B | `ENCODER1_A/B` | 48 / 36 |
| 编码器2 A / B | `ENCODER2_A/B` | 38 / 37 |
| 编码器3 A / B | `ENCODER3_A/B` | 17 / 18 |
| 超声波 TRIG / ECHO | `SONIC_TRIG/ECHO` | 47 / 21 |
| MPU6050 SDA / SCL | `MPU_SDA/SCL` | 9 / 14 |
| OLED SDA / SCL | `OLED_SDA/SCL` | 9 / 14（与 MPU 共用，走旁路） |
| 舵机 Pan / Tilt | `SERVO_PAN/TILT` | 45 / 46 |
| 配网按键 | `CFG_BUTTON` | 11（接地清配置进配网） |
| 蜂鸣器 | `BUZZER_PIN` | 12 |
| 电源指示灯 | `BAT_VOLTAGE_PIN` | 20（低电平常亮） |
| 状态灯 | `STUTS` | 2 |
| 雷达 TX / RX | `LIDAR_TX/RX` | 43 / 44（UART1，150000 波特） |

> ⚠️ ESP32-S3 的 **GPIO46 是输入专用引脚，不能输出 PWM**（见 `问题.txt`）。

---

## 项目结构

```
X_roscar/
├── platformio.ini              # PlatformIO 构建配置（env:xc）
├── custom.meta                 # micro-ROS 自定义构建参数（MTU=2048）
├── boards/                     # 自定义板级配置
│   └── fishbot_motion_control_four_driver_v2.json
│
├── src/                        # 主程序源文件
│   ├── main.cpp                # 入口：配网判断 → 初始化 → 自检 → loop
│   ├── ConfigManager.cpp       # Web 配网（AP 热点 + NVS 读写）
│   ├── Motor_control.cpp       # 电机初始化 / 运行 / 刹车 / 滑行（LEDC 20kHz）
│   ├── newencoder.cpp          # 编码器初始化与速度测量
│   ├── Pid_control.cpp         # 4 路独立 PID + 缓动限幅
│   ├── Microros.cpp            # micro-ROS 节点（/cmd_vel、/odom、/imu、/scan）
│   ├── MPU6050.cpp             # MPU6050 姿态（含 I2C Bypass）
│   ├── OledDisplay.cpp         # SSD1306 OLED 显示
│   ├── SelfCheck.cpp           # 上电自检 + 蜂鸣器
│   ├── Distance_check.cpp      # HC-SR04 超声波测距
│   ├── Servo.cpp               # 舵机云台 UDP 控制
│   └── Lidar.cpp               # YDLIDAR X2 雷达驱动（UART1 解析任务）
│
├── include/                    # 头文件
│   ├── PinConfig.h             # 引脚集中配置
│   ├── ConfigManager.h         # 配网管理器
│   ├── Motor_control.h         # 电机类
│   ├── newencoder.h            # 编码器接口
│   ├── Pid_control.h           # PID 控制类
│   ├── Microros.h              # micro-ROS 任务声明
│   ├── MPU6050.h               # MPU6050 接口
│   ├── OledDisplay.h           # OLED 接口
│   ├── SelfCheck.h             # 自检 + 蜂鸣器接口
│   ├── Distance_check.h        # 测距接口
│   ├── Servo.h                 # 舵机接口
│   └── Lidar.h                 # 雷达接口
│
├── lib/                        # 自定义库
│   ├── Kinematics/             # 运动学解算 + 里程计
│   ├── Pidcontraller/          # PID 控制器
│   └── Driver/                 # 底层驱动（MPU6050_light / ESP32Encoder / Esp32McpwmMotor 等）
│
├── lidar_pkg/                  # ROS 2 功能包（USB 版雷达节点，见下文）
├── host/                       # 上位机辅助脚本
├── Example/                    # 独立示例代码（双/四电机、MPU6050 数据读取等）
├── discard/                    # 已废弃的旧驱动（保留参考）
└── test/
```

---

## 功能模块说明

### 🌐 Web 配网（`ConfigManager.cpp`）

- AP 热点 `CAR_CONFIG`（无密码），Web 页面 `http://192.168.4.1`。
- 保存 SSID / 密码 / Agent IP 到 **NVS 闪存**（掉电不丢）。
- `main.cpp` 上电读 GPIO11：按下 → `clearConfig()` + `startAP()` 进入配网循环。
- NVS 为空时写入代码里的默认配置（`main.cpp` 中的 `DEFAULT_SSID/DEFAULT_PASS/DEFAULT_AGENT_IP`）。

```cpp
configManager.begin();          // 初始化，读取 NVS
configManager.getConfig();      // 获取 ConfigData {ssid, password, agent_ip}
configManager.startAP();        // 开启 AP + Web 服务器
configManager.handleClient();   // 处理 Web 请求
configManager.clearConfig();    // 清空 NVS 配置
```

---

### 🏎️ 电机控制（`Motor_control.cpp`）

4 路直流电机通过 **TB6612** 驱动，PWM 使用 **LEDC 硬件 PWM（20kHz，8bit）**。

| 电机 | IN1 | IN2 | PWM | LEDC 通道 | 接线反向 |
|------|-----|-----|-----|:---------:|:--------:|
| 电机1 | GPIO7  | GPIO15 | GPIO16 | 0 | ❌ |
| 电机2 | GPIO41 | GPIO40 | GPIO39 | 1 | ✅ |
| 电机3 | GPIO13 | GPIO42 | GPIO1  | 2 | ✅ |
| 电机4 | GPIO6  | GPIO5  | GPIO4  | 3 | ✅ |

- `Motor_Run(id, pwm)`：`pwm ∈ [-100, 100]`，接近 0 自动刹车；带**死区补偿**（`MOTOR_DEAD_ZONE=5`）。
- `Motor_Brake()`：IN1=H, IN2=H 短路急停；`Motor_Coast()`：IN1=L, IN2=L 滑行。
- 接线反向：`motor_cfg[]` 数组中的 `reverse` 字段（M2/M3/M4 为 `true`），软件层面异或取反方向。

```cpp
my_motor[i].Motor_Setup();      // 初始化所有电机引脚 + LEDC PWM
my_motor[i].Motor_Run(id, pwm); // 运行电机 -100 ~ 100
my_motor[i].Motor_Brake(id);    // 急停
my_motor[i].Motor_Coast(id);    // 滑行
```

---

### 🔄 编码器测速（`newencoder.cpp`）

使用 `ESP32Encoder`（PCNT 硬件计数）half-quad 模式 + 100 滤波读取 4 路编码器。

| 编码器 | A | B |
|:------:|:--:|:--:|
| 0 | GPIO3  | GPIO8  |
| 1 | GPIO48 | GPIO36 |
| 2 | GPIO38 | GPIO37 |
| 3 | GPIO17 | GPIO18 |

- `Velocity_Check()`：差分计算各轮速度（mm/s），结果存入 `Velocity[4]`。
- 每 tick ≈ `0.28mm`（见 `Velocity_Check()` 中的系数，可按车轮/减速比校准）。

---

### 🎯 PID 闭环（`Pid_control.cpp` + `lib/Pidcontraller/`）

4 路独立增量式 PID，参数定义在 `Pid_control.cpp`：

| 电机 | Kp | Ki | Kd | 输出限幅 |
|:----:|:---:|:---:|:---:|:-------:|
| M1 | 0.28   | 0.10 | 0.01 | ±100 |
| M2 | 0.254  | 0.08 | 0.05 | ±100 |
| M3 | 0.255  | 0.08 | 0.01 | ±100 |
| M4 | 0.255  | 0.09 | 0.01 | ±100 |

- `PID_run::Pid_run_()` 带**缓动限幅**（`SLEW_RATE=12`）：输出每次变化不超过该值，防止抽搐。
- 停止时 PID 输出按 `×0.9` 衰减到 0（`Pid.cpp`），避免急停抖动。

```cpp
run[i].Pid_setgoal(id, target);  // 设置目标速度
run[i].Pid_run_();               // 执行一次 PID 运算并输出 PWM
```

---

### 📐 运动学 & 里程计（`lib/Kinematics/`）

差速模型，轮距 `wheel_distance = 165mm`（`Kinematics.cpp`，按车体实际校准）。

- **逆解** `Kinematics_down(linear, angular, &L, &R)`：目标线速度 + 角速度 → 左右轮速。
- **正解** `Kinematics_up(M1, M2, M3, M4, &angular, &linear)`：实测轮速 → 整车速度。
- **里程计** `odom_update()`：积分 x / y / angle；发布时线速度 mm/s → m/s（÷1000）。

```cpp
typedef struct {
    float x, y;          // 位置 (m)
    float angle;         // 朝向 (rad)
    float linear_speed;  // 线速度 (mm/s)
    float angular_speed; // 角速度 (rad/s)
} odom_t;
```

---

### 🌐 micro-ROS（`Microros.cpp`）

- 独立任务 `micro_ros_task`：等 WiFi → 读 Agent IP（无效回退默认）→ 初始化节点 → 循环。
- 节点名 `fish_node`；transport = WiFi，端口 8888。
- 订阅 `/cmd_vel`（best_effort）→ `Kinematics_down` → 设置 4 路 PID 目标；接近 0 时刹车。
- **50ms 定时器**：`odom_update()` 并发布 `/odom`、`/imu`、`/scan`。
- 坐标帧：`odom` / `base_footprint`（/odom）、`base_link`（/imu）、`laser_frame`（/scan）。
- 启动时做 micro-ROS 时间同步（`rmw_uros_epoch_synchronized()`）。
- 收到 `/cmd_vel` 点亮状态灯，150ms 无新指令自动熄灭（`main.cpp` loop 处理）。

```bash
# 上位机发送运动指令
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
# 查看数据
ros2 topic echo /odom
ros2 topic echo /imu
ros2 topic echo /scan
```

---

### 📡 MPU6050 + OLED（`MPU6050.cpp` / `OledDisplay.cpp`）

**I2C 架构（Bypass 旁路模式）**：

```
GPIO9(SDA) ── MPU6050 ── XDA ── OLED
GPIO14(SCL) ── MPU6050 ── XCL ── OLED
```

- `MPU6050_Init()`：`Wire.begin(9,14)` → `mpu.begin()` → 写 `0x37=0x02` 启用 I2C Bypass → 偏移校准。
- OLED 地址 `0x3C`，I2C 400kHz。
- OLED 显示（20Hz 限频）：IP 地址、M1~M4 轮速（带正负号）、进度条、底部弹跳小球动画。
- IMU 消息：欧拉角 → 四元数、陀螺仪 rad/s、加速度 g → m/s²。

```cpp
MPU6050_Init();                 // 初始化 MPU6050 并启用 I2C Bypass
updateMPU();                    // 每次读取前更新
getMPUYaw() / getMPURoll() / getMPUPitch() ...  // 欧拉角（度）
```

---

### 📏 超声波测距（`Distance_check.cpp`）

HC-SR04（TRIG=GPIO47, ECHO=GPIO21）。`Distance_chect()` 触发间隔 ≥60ms，返回 cm；
超时 / 超范围返回 -1.0，间隔太短返回 -2.0。

---

### 🎮 舵机云台（`Servo.cpp`）

- UDP 端口 **4210**，每包 2 字节 `[pan, tilt]`（0~180，超限截断）。
- Pan = GPIO45、Tilt = GPIO46，50Hz。

```python
# Python 控制示例
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(bytes([90, 45]), ("<ESP32_IP>", 4210))
```

---

### 📡 激光雷达（`Lidar.cpp` + `include/Lidar.h`）

- YDLIDAR X2/X2L，UART1 **150000 波特**，启动指令 `A5 60`。
- 协议：`AA 55 CT LSN fSa lSa samples(3*LSN) checksum`，原生一圈 720 点、5Hz。
- 独立解析任务 `lidar_task`（栈 4096，优先级 2）+ 逐字节状态机 + **双缓冲**。
- 输出 200 点 `/scan`（1.8°/点，`LIDAR_SCAN_SIZE` 可调），angle 0~2π、range 0.05~8m。
- 内含 `lidarBaudSweep()` 波特率扫描（换雷达型号诊断用，默认注释）。
- 每 5 秒串口打印诊断：`[Lidar DBG] RX=… 包=… 圈=… 有效点=… 就绪=…`。

> ⚠️ **micro-ROS MTU 限制**：默认 MTU=1024B，720/360 点 LaserScan 会超限发布失败，
> 当前用 200 点（≈870B）。`custom.meta` 已把 MTU 提到 2048，改后需删除
> `.pio/libdeps/xc/micro_ros_platformio/libmicroros/` 目录强制重建库才生效。

---

### ✅ 上电自检（`SelfCheck.cpp`）

- 检查蜂鸣器 / 状态灯 / 电源灯引脚，确认 MPU6050、编码器、电机均已初始化。
- **通过** → 蜂鸣短鸣 200ms + OLED `SelfCheck OK`；
  **失败** → 蜂鸣长鸣 1s + OLED `SelfCheck FAIL`。

---

## ROS 2 主机侧（`lidar_pkg/`）

- ROS 2 功能包，含 **USB 版**雷达节点 `lidar_node`（Linux 串口 + OpenCV 可视化 + 离群点滤波），
  串口默认 `/dev/lidar`，frame `laser_frame`（参数见 `config/lidar_params.yaml`）。
- ⚠️ **当前 ESP32 已直接通过 micro-ROS 发布 `/scan`**，USB 版 `lidar_node` 可停用；
  上位机只需运行 `micro_ros_agent` 转发即可。
- launch：`lidar.launch.py`、`monitor.launch.py`；RViz 配置 `rviz/lidar.rviz`。

```bash
# （若仍用 USB 版节点）
colcon build --packages-select lidar_pkg
source install/setup.bash
ros2 launch lidar_pkg lidar.launch.py
```

---

## 依赖库（`platformio.ini` 的 `lib_deps`）

| 库 | 用途 |
|----|------|
| `micro_ros_platformio`（Gitee 镜像） | micro-ROS 客户端 |
| `Adafruit GFX Library` | 图形基库 |
| `Adafruit SSD1306` | SSD1306 OLED 驱动 |
| `ESP32Servo` | 舵机驱动 |

---

## 已知问题 / 排障

1. **GPIO46 不能输出 PWM**：ESP32-S3 的 GPIO46 为输入专用引脚，仅可做输入。
2. **串口初始化成功后自动断开**：见根目录 `问题.txt`，重点排查 USB-CDC / 复位相关。
3. **串口无输出**：确认插的是板子**原生 USB 口**（GPIO19/20），并已加 `-DARDUINO_USB_CDC_ON_BOOT=1`。
4. **micro-ROS 连不上 Agent**：ESP32 与电脑连同一热点；Agent 用 `udp4 --port 8888`；换热点后重新配网更新 Agent IP。
5. **电机不转 / 方向反**：核对 `motor_cfg[]` 接线与 `reverse` 字段；检查 PID 输出限幅（±100）。
6. **里程计不准**：校准 `wheel_distance`（165mm）与编码器系数（0.28）。
7. **雷达无数据**：看串口 `[Lidar DBG]` 的 `RX=`——RX=0 说明信号没进引脚
   （检查供电 / 电机干扰 / 引脚冲突），不是解析问题。
8. **雷达 720 点发不出**：micro-ROS MTU 限制，用 200 点；或重建库后 MTU 提到 2048。
9. **OLED 不显示**：确认串口有 `I2C Bypass enabled` 日志；检查 OLED 地址 `0x3C` 与 XDA/XCL 接线。
