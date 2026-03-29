#include "St7735.h"
#include <Adafruit_GFX.h> 
#include <Adafruit_ST7735.h>


// 引脚
#define TFT_SCLK  11   // SCL → 时钟
#define TFT_MOSI  12   // SDA → 数据 (MOSI)
#define TFT_CS   21   // Chip Select
#define TFT_DC   14   // Data/Command
#define TFT_RST  13  // Reset

// 使用软件 SPI 构造函数：(cs, dc, mosi, sclk, rst)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);



void St7735_Init() {
  Serial.println("Initializing ST7735 via Software SPI...");

  // 初始化屏幕（根据你的模块 PCB 颜色选择）
  // 常见选项：
  //   INITR_BLACKTAB   → 黑色 PCB（最常见）
  //   INITR_REDTAB     → 红色 PCB
  //   INITR_GREENTAB   → 绿色 PCB（部分需偏移）
  tft.initR(INITR_BLACKTAB);

  // 设置旋转方向（0~3）
  tft.setRotation(1);  // 横屏（128x160 → 宽度=160, 高度=128）

  // 清屏并显示信息
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(10, 10);
  tft.println("Software SPI");
  tft.println("ESP32 + ST7735");
  tft.println("SCLK=12, MOSI=13");

  Serial.println("Init OK!");
}

void St7735_test() {
  static int x = 0;
  static uint16_t color = ST7735_CYAN;

  // 擦除旧方块
  tft.fillRect(x, 50, 25, 25, ST7735_BLACK);
  
  // 移动位置
  x = (x + 3) % (tft.width() - 25);
  
  // 绘制新方块
  tft.fillRect(x, 50, 25, 25, color);

  // 切换颜色
  color = (color == ST7735_CYAN) ? ST7735_MAGENTA : ST7735_CYAN;

  delay(150);
}



