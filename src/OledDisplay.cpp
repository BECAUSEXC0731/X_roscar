#include "OledDisplay.h"
#include "PinConfig.h"

// 全局单例
OledDisplay oledDisplay;

// ============================================================
// 构造函数
// ============================================================
OledDisplay::OledDisplay()
    : display_(OLED_SCREEN_W, OLED_SCREEN_H, &Wire, OLED_RESET_PIN)
    , initialized_(false)
    , lastUpdateMs_(0)
    , animFrame_(0)
{
    ip_[0] = '\0';
}

// ============================================================
// 初始化 OLED
// Wire 由 MPU6050_Init() 预先初始化（含 I2C Bypass 模式）
// ============================================================
bool OledDisplay::begin()
{
    Wire.setClock(400000);      // 提升 I2C 速度到 400kHz
    delay(50);

    if (!display_.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("[OLED] SSD1306 初始化失败！检查接线");
        return false;
    }

    initialized_ = true;
    display_.clearDisplay();
    display_.setTextSize(1);
    display_.setTextColor(SSD1306_WHITE);
    display_.cp437(true);       // 支持完整 ASCII
    display_.display();

    Serial.println("[OLED] SSD1306 初始化成功");
    return true;
}

// ============================================================
// 显示初始化消息
// ============================================================
void OledDisplay::showInitMessage(const char *msg)
{
    if (!initialized_) return;

    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.setTextSize(1);
    display_.println(msg);
    display_.display();
}

// ============================================================
// 设置 IP 地址
// ============================================================
void OledDisplay::setIP(const char *ip)
{
    strncpy(ip_, ip, sizeof(ip_) - 1);
    ip_[sizeof(ip_) - 1] = '\0';
}

// ============================================================
// 清屏
// ============================================================
void OledDisplay::clear()
{
    if (!initialized_) return;
    display_.clearDisplay();
    display_.display();
}

// ============================================================
// 刷新界面：IP + 4 路轮速（带正负号）
// 限频 20Hz（50ms）
//
// 布局（128×64 OLED）：
//   行0 (y=0)  : IP: xxx.xxx.xxx.xxx
//   行1 (y=14) : M1:+123  M2:-456
//   行2 (y=28) : M3:+078  M4:-090
//   行3 (y=44) : ● 弹跳小球动画
// ============================================================
void OledDisplay::updateDisplay(const float speeds[4])
{
    if (!initialized_) return;

    unsigned long now = millis();
    if (now - lastUpdateMs_ < 50) return;   // 20Hz 限频
    lastUpdateMs_ = now;

    display_.clearDisplay();
    char buf[22];

    // ── 第 0 行：IP 地址 ──────────────────────────────────
    display_.setCursor(0, 0);
    display_.setTextSize(1);
    display_.print("IP: ");
    display_.println(ip_);

    // ── 第 1 行：M1 和 M2 轮速 ────────────────────────────
    display_.setCursor(0, 14);
    snprintf(buf, sizeof(buf), "M1:%+5.0f  M2:%+5.0f", speeds[0], speeds[1]);
    display_.println(buf);

    // ── 第 1 行下方画小进度条：M1 ─────────────────────────
    int barY1 = 22;
    drawMiniBar(0, barY1, 60, 4, speeds[0], 300.0f);
    // M2 进度条在右侧
    drawMiniBar(66, barY1, 60, 4, speeds[1], 300.0f);

    // ── 第 2 行：M3 和 M4 轮速 ────────────────────────────
    display_.setCursor(0, 30);
    snprintf(buf, sizeof(buf), "M3:%+5.0f  M4:%+5.0f", speeds[2], speeds[3]);
    display_.println(buf);

    // ── 第 2 行下方画小进度条：M3、M4 ─────────────────────
    int barY2 = 38;
    drawMiniBar(0, barY2, 60, 4, speeds[2], 300.0f);
    drawMiniBar(66, barY2, 60, 4, speeds[3], 300.0f);

    // ── 底部动画：弹跳小球 ──────────────────────────────────
    animFrame_++;
    int ballRadius = 2;
    int ballY = OLED_SCREEN_H - ballRadius - 1;
    int period = 120;
    int half    = period / 2;
    int phase   = animFrame_ % period;
    int ballX   = (phase < half)
                  ? map(phase, 0, half - 1, ballRadius, OLED_SCREEN_W - ballRadius)
                  : map(phase - half, 0, half - 1, OLED_SCREEN_W - ballRadius, ballRadius);
    display_.fillCircle(ballX, ballY, ballRadius, SSD1306_WHITE);

    display_.display();
}

// ============================================================
// 绘制小进度条（用于单个轮速）
// ============================================================
void OledDisplay::drawMiniBar(int x, int y, int w, int h, float speed, float maxSpeed)
{
    float ratio = fabs(speed) / maxSpeed;
    if (ratio > 1.0f) ratio = 1.0f;
    int fillW = (int)(ratio * (w - 2));
    display_.drawRect(x, y, w, h, SSD1306_WHITE);
    if (fillW > 0) {
        display_.fillRect(x + 1, y + 1, fillW, h - 2, SSD1306_WHITE);
    }
}
