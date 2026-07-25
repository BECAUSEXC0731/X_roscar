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
{
    ip_[0] = '\0';
}

// ============================================================
// 初始化 OLED（I2C 引脚 SDA=11, SCL=12）
// ============================================================
bool OledDisplay::begin()
{
    Wire.begin(OLED_SDA, OLED_SCL);
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
// 刷新完整界面：IP + 4 路速度条形图
// 限频 20Hz（50ms），避免 I2C 占用过多 CPU
// ============================================================
void OledDisplay::updateSpeeds(const float speeds[4])
{
    if (!initialized_) return;

    unsigned long now = millis();
    if (now - lastUpdateMs_ < 50) return;   // 20Hz 限频
    lastUpdateMs_ = now;

    display_.clearDisplay();

    // ── 第 0 行：IP 地址 ──────────────────────────────────
    display_.setCursor(0, 0);
    display_.setTextSize(1);
    display_.print("IP: ");
    display_.println(ip_);

    // ── 分隔线 ──────────────────────────────────────────────
    display_.drawFastHLine(0, 9, OLED_SCREEN_W, SSD1306_WHITE);

    // ── 4 个速度条 ─────────────────────────────────────────
    for (int i = 0; i < 4; i++) {
        drawSpeedBar(i, speeds[i]);
    }

    display_.display();
}

// ============================================================
// 绘制单个速度条
//
// 布局（每行 12px）：
//   y+0: "M1" 标签（12px 宽）
//   y+1: ▓▓▓▓▓▓▓▓▓▓▓▓▓▓ 条形图（8px 高）
//   y+1: 右侧显示数值
// ============================================================
void OledDisplay::drawSpeedBar(int index, float speed)
{
    static const char *labels[] = {"M1", "M2", "M3", "M4"};
    int y = 14 + index * 12;   // 每行 12px

    // ── 标签 ───────────────────────────────────────────────
    display_.setCursor(2, y);
    display_.setTextSize(1);
    display_.print(labels[index]);

    // ── 条形图参数 ─────────────────────────────────────────
    int barX  = 20;                         // 条形图起始 X
    int barY  = y + 1;                      // 条形图起始 Y
    int barH  = 8;                          // 条形图高度
    int maxW  = OLED_SCREEN_W - barX - 32;  // 条形图最大宽度（留空间给数值）

    // 将 |speed| 映射到条形宽度（假设最大速度 300 mm/s）
    float absSpd = fabs(speed);
    int barW = (int)(absSpd / 300.0f * maxW);
    if (barW > maxW) barW = maxW;
    if (barW < 0)    barW = 0;

    // ── 绘制条形图 ─────────────────────────────────────────
    if (barW > 0) {
        display_.fillRect(barX, barY, barW, barH, SSD1306_WHITE);
    }

    // ── 显示数值 ───────────────────────────────────────────
    display_.setCursor(barX + maxW + 2, y);
    char buf[12];
    snprintf(buf, sizeof(buf), "%.0f", speed);
    display_.print(buf);
}
