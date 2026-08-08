#ifndef MPU6050_h
#define MPU6050_h 

void MPU6050_Init();
void MPU6050_check();

// 更新 MPU 数据（必须在读取任何角度/加速度之前调用）
void updateMPU();

// 获取偏航角（Z 轴旋转角度，单位：度）
float getMPUYaw();

// 获取俯仰角（X 轴）
float getMPUPitch();

// 获取翻滚角（Y 轴）
float getMPURoll();

// ── 陀螺仪数据（单位：度/秒）──────────────────────────
float getMPUGyroX();
float getMPUGyroY();
float getMPUGyroZ();

// ── 加速度计数据（单位：g，1g = 9.81 m/s²）────────────
float getMPUAccelX();
float getMPUAccelY();
float getMPUAccelZ();

#endif
