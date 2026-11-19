# X_roscar — ESP32-S3 机器人底盘固件（micro-ROS / ROS 2）

基于 **PlatformIO + Arduino 框架** 的 ESP32-S3 **四轮底盘**固件（**四轮差速 / 麦克纳姆轮**，可在配网页切换），通过 **micro-ROS over WiFi** 接入 ROS 2：订阅 `/cmd_vel` 运动指令，发布 `/odom`、`/imu`、`/scan` 数据。

集成电机驱动、编码器测速、PID 速度闭环、可配置运动学（差速 / 麦轮）与里程计、MPU6050 姿态、YDLIDAR X2 激光雷达、HC-SR04 超声波、OLED 显示、舵机云台、Web 配网与上电自检等功能，可作为低成本 ROS 2 机器人底盘方案。

## 功能特性

- **电机控制** — 4 路 TB6612 驱动，LEDC PWM 20kHz；支持刹车 / 滑行 / 死区补偿 / 接线反向
- **编码器测速** — 4 路 AB 相正交编码器（PCNT 硬件计数），每 tick ≈ 0.28mm
- **速度闭环** — 4 路独立增量式 PID，带缓动限幅（防抽搐）
- **运动学 & 里程计** — 差速 / 麦克纳姆双模型正逆解 + 里程计积分（x / y / angle），模型与几何参数在配网页可改
- **航向融合** — 里程计航向用轮速积分做短期、MPU6050 偏航做长期修正，抑制打滑漂移
- **micro-ROS** — 订阅 `/cmd_vel`；发布 `/odom`、`/imu`、`/scan`（WiFi UDP 8888）
- **姿态** — MPU6050 六轴 + 欧拉角，I2C Bypass 旁路扩展 OLED
- **激光雷达** — YDLIDAR X2/X2L（UART1 150000 波特），独立解析任务 + 双缓冲
- **测距 / 显示 / 云台** — HC-SR04 超声波、0.96″ SSD1306 OLED、2 路舵机（UDP 4210）
- **配网 & 自检** — AP 热点 Web 配网（配置存 NVS，掉电不丢）、上电自检 + 蜂鸣器反馈
- **OTA 升级** — WiFi 无线烧录固件（ArduinoOTA / UDP 3232，带密码，升级前自动刹车）

## 功能模块说明

| 模块 | 文件 | 说明 |
|------|------|------|
| 主流程 | `src/main.cpp` | 配网键判断 → 各外设初始化 → 上电自检 → 主循环 |
| Web 配网 | `src/ConfigManager.cpp` | AP 热点配网页（WiFi + 运动学参数）+ NVS 读写 |
| 电机控制 | `src/Motor_control.cpp` | 4 路 LEDC PWM：运行 / 刹车 / 滑行 / 死区补偿 |
| 编码器 | `src/newencoder.cpp` | PCNT 硬件计数测速，得到 4 路轮速 |
| PID | `src/Pid_control.cpp` | 4 路独立增量式 PID + 输出缓动限幅 |
| 运动学 | `lib/Kinematics/` | 差速 / 麦轮正逆解（运行时按 NVS 配置切换）+ 里程计积分 |
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

1. 小车正常启动后，**按一下配网键**（详见下表）→ 蜂鸣器 3 短鸣 + OLED 显示 `CONFIG MODE`。
2. 手机连接热点 **`CAR_CONFIG`**（无密码）。
3. 浏览器打开 **`http://192.168.4.1`**，填写：热点 SSID、密码（**必须 2.4GHz 热点**）、Agent IP（运行 micro-ROS Agent 的电脑 IP）。
4. 同页下半部分是**运动学配置**（见下节），按实际底盘填写。
5. 保存后自动重启并连接。

### 运动学配置（配网页可改）

| 字段 | 说明 | 默认 |
|------|------|------|
| 底盘类型 | `四轮差速` / `麦克纳姆轮` | 四轮差速 |
| 轮距 | 左右轮中心距，mm | 165 |
| 轴距 | 前后轮中心距，mm（差速不用，麦轮必填） | 0 |
| 角速度校准系数 | 原地旋转比指令转得多就调小，转得不够就调大 | 1.000 |

- 参数存 NVS（`kin_type` / `track` / `wheelbase` / `omega_scale`），掉电不丢；**老固件升级后读不到这些 key 时沿用差速 + 165mm，行为与改造前一致**，不用重新配网。
- 差速模型的旋转半径是 `轮距/2`，麦轮是 `(轮距+轴距)/2` —— 二者不同，固件内部各自计算，不要用同一个数去套。
- 麦轮轮位约定：**电机 1 = 左前、2 = 左后、3 = 右后、4 = 右前**。麦轮必须按 X 形装：左前/右后同一种旋向，右前/左后另一种，否则直行会斜着走、平移会原地打转（属于装轮问题，软件改不了）。
- `/cmd_vel` 的 `linear.y`（左移为正）在麦轮模式下生效，差速模式忽略。

**进入配网模式的方式**（都会**蜂鸣器 3 短鸣** + OLED 显示 `CONFIG MODE` 作为确认）：

| 方式 | 触发条件 |
|------|----------|
| 按键 | **运行中**按下配网键并保持 300ms（边沿触发：必须是"松开→按下"） |
| 自动兜底 | 正常启动时 **20 秒**连不上配置里的热点，自动转入配网模式 |

> 按键用**边沿触发**：上电时若该脚已处于"按下"电平（引脚被拉低/极性配反）不会误进配网，
> 开机日志会打印 `配网键 GPIOxx 当前电平=x，按下时应为 y`，据此核对极性
> （极性在 `include/PinConfig.h` 的 `CFG_BUTTON_ACTIVE_LOW` 里切换）。
> 连不上热点时日志会给出原因，例如 `最终状态=1：找不到这个热点：名字写错/热点没开/是5GHz`。
> 进配网不会清空原有配置，配网页会带出上次的 SSID/密码/IP，方便只改错的那一项。
> 当 NVS 为空时，固件使用 `src/main.cpp` 中 `DEFAULT_SSID / DEFAULT_PASS / DEFAULT_AGENT_IP` 编译默认值。

## OTA 无线升级

烧过一次带 OTA 功能的固件后，以后改代码可以**不插 USB 线**，通过 WiFi 升级（ArduinoOTA，UDP 3232）。

> 前提：分区表必须含 `otadata + app0 + app1`（本项目默认的 8MB 分区表已满足，编译产物里可直接看到）。
> 改分区表、首次启用 OTA、或 OTA 把固件刷坏时，都必须用 **USB 线**烧录一次。

### 操作步骤

```bash
# ① 首次（以及救砖）：USB 线连接，正常烧录
~/.platformio/penv/bin/platformio run -e xc -t upload

# ② 运行起来后，看 OLED 上的 IP（如 192.168.1.50），确认电脑与小车在同一 WiFi/网段
ping 192.168.1.50

# ③ 改完代码，无线升级（IP 写在 platformio.ini 的 [env:xc_ota] upload_port）
~/.platformio/penv/bin/platformio run -e xc_ota -t upload
```

### 说明

- **网络方向**：小车作为 STA 连你的路由器/热点（正常运行模式），电脑连同一个网络后**主动推固件**给小车；
  小车自己的 `CAR_CONFIG` 热点（配网模式）里没有 OTA。
- **自动重启**：写入并校验完成后固件自己重启，**不需要手动按 RESET**；NVS 配置（WiFi）保留，不用重新配网。
- **失败不变砖**：双 app 分区乒乓，传输中断 / 校验失败时不会切换启动分区，小车继续跑旧固件。
- **安全**：密码 `xroscar_ota` 定义在 `src/main.cpp` 的 `OTA_PASSWORD`，必须与 `platformio.ini` 中
  `upload_flags = --auth=` 一致；主机名 `x_roscar`，局域网可用 `x_roscar.local` 访问。
- **升级期间电机会被刹停**：`onStart` 里调用 `stopAllMotors()` + PID 目标清零，但重启那几秒引脚复位、
  电机不受控，**升级时请把车架空或放在平地上**，并让上位机停止发布 `/cmd_vel`。

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
| 配网键 | 46 | 运行中按下进配网（⚠️ GPIO46 是 S3 启动绑带脚，内部弱下拉，注意 `CFG_BUTTON_ACTIVE_LOW` 极性） |
| 蜂鸣器 / 电源灯 / 状态灯 | 12 / 20 / 2 | 自检与状态指示 |

> ⚠️ **原生 USB-CDC**：日志、下载、串口监视请插板子**原生 USB 口（GPIO19/20）**，不是 CP2102 的 UART 口；这样才能把 UART0（43/44）腾给雷达使用。
> ⚠️ **GPIO46** 是输入专用引脚，不能输出 PWM（详见 `问题.txt`）。
