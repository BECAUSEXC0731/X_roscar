#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <WebServer.h>

// 配网数据结构：存到 NVS 闪存，掉电不丢
struct ConfigData {
    char ssid[32];       // 手机热点名称
    char password[64];   // 热点密码
    char agent_ip[16];   // 运行 micro_ros_agent 的电脑 IP
};

class ConfigManager {
public:
    ConfigManager();

    void begin();           // 初始化 NVS，读取已有配置
    bool isConfigured();    // 是否有有效配置
    ConfigData getConfig(); // 获取配置

    void startAP();         // 开启 AP 热点 + Web 配置页面
    void handleClient();    // 处理 Web 请求（放 loop 里）
    void clearConfig();     // 清除 NVS 中的配置

private:
    ConfigData config_;
    WebServer *server_;
    bool configMode_;

    void handleRoot();      // Web: 配置页
    void handleSave();      // Web: 保存配置
    void handleNotFound();  // Web: 404
    void saveToNVS();
    bool loadFromNVS();
};

extern ConfigManager configManager;

#endif
