/*
 * Copyright 2026 徐畅
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
    void setDefaults(const char *ssid, const char *pass, const char *ip); // NVS 为空时设置默认值

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
