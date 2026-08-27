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

#include "ConfigManager.h"
#include <nvs_flash.h>
#include <nvs.h>

ConfigManager configManager;  // 全局单例

ConfigManager::ConfigManager() : server_(nullptr), configMode_(false)
{
    memset(&config_, 0, sizeof(config_));
}

void ConfigManager::begin()
{
    // 初始化 NVS 闪存
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    loadFromNVS();
}

bool ConfigManager::isConfigured()
{
    return strlen(config_.ssid) > 0 && strlen(config_.agent_ip) > 0;
}

ConfigData ConfigManager::getConfig()
{
    return config_;
}

// ============================================================
// AP 配网模式：开启热点 + Web 服务器
// ============================================================
void ConfigManager::startAP()
{
    configMode_ = true;

    WiFi.mode(WIFI_AP);
    WiFi.softAP("CAR_CONFIG");

    IPAddress apIP = WiFi.softAPIP();
    Serial.println("\n========================================");
    Serial.println("配网模式已启动");
    Serial.println("手机连接热点: CAR_CONFIG（无密码）");
    Serial.print("  打开浏览器访问: http://");
    Serial.println(apIP);
    Serial.println("========================================\n");

    server_ = new WebServer(80);
    server_->on("/", std::bind(&ConfigManager::handleRoot, this));
    server_->on("/save", HTTP_POST, std::bind(&ConfigManager::handleSave, this));
    server_->onNotFound(std::bind(&ConfigManager::handleNotFound, this));
    server_->begin();
}

void ConfigManager::handleClient()
{
    if (server_) server_->handleClient();
}

// ============================================================
// Web 配置页面（内嵌 HTML）
// ============================================================
void ConfigManager::handleRoot()
{
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 配网</title>
<style>
body{font-family:Arial;margin:20px;background:#f0f0f0}
.card{background:white;padding:24px;border-radius:10px;max-width:400px;margin:auto;box-shadow:0 2px 10px rgba(0,0,0,0.1)}
h2{color:#333;text-align:center}
label{display:block;margin-top:12px;color:#555;font-size:14px}
input{width:100%;padding:10px;margin:4px 0 8px 0;border:1px solid #ddd;border-radius:5px;box-sizing:border-box;font-size:14px}
button{width:100%;padding:12px;background:#4CAF50;color:white;border:none;border-radius:5px;font-size:16px;cursor:pointer;margin-top:12px}
button:hover{background:#45a049}
.hint{color:#999;font-size:12px;margin-top:0}
.footer{text-align:center;margin-top:16px;color:#aaa;font-size:12px}
</style>
</head>
<body>
<div class="card">
<h2>🔧 小车配网配置</h2>
<form action="/save" method="POST">
<label>手机热点名称 (SSID):</label>
<input name="ssid" value=")rawliteral";
    html += config_.ssid;
    html += R"rawliteral(" required>

<label>热点密码:</label>
<input name="pass" type="password" value=")rawliteral";
    html += config_.password;
    html += R"rawliteral(">

<label>Agent IP (运行 micro_ros_agent 的电脑IP):</label>
<input name="ip" placeholder="192.168.x.x" value=")rawliteral";
    html += config_.agent_ip;
    html += R"rawliteral(" required>
<p class="hint">查看电脑连接手机热点后分配到的 IP</p>

<button type="submit">✅ 保存并重启</button>
</form>
<div class="footer">配置将保存到 ESP32 闪存，掉电不丢</div>
</div>
</body>
</html>
)rawliteral";
    server_->send(200, "text/html", html);
}

// ============================================================
// 保存配置 → 写入 NVS → 重启
// ============================================================
void ConfigManager::handleSave()
{
    String ssid = server_->arg("ssid");
    String pass = server_->arg("pass");
    String ip   = server_->arg("ip");

    if (ssid.length() == 0 || ip.length() == 0) {
        server_->send(200, "text/html",
            "<html><body style='font-family:Arial;margin:40px'>"
            "<h3>❌ SSID 和 Agent IP 不能为空</h3>"
            "<a href='/'>← 返回</a></body></html>");
        return;
    }

    strncpy(config_.ssid,     ssid.c_str(), sizeof(config_.ssid) - 1);
    strncpy(config_.password, pass.c_str(), sizeof(config_.password) - 1);
    strncpy(config_.agent_ip, ip.c_str(),   sizeof(config_.agent_ip) - 1);

    saveToNVS();

    server_->send(200, "text/html",
        "<html><body style='font-family:Arial;margin:40px;text-align:center'>"
        "<h3>✅ 配置已保存！正在重启...</h3>"
        "<p>小车将自动连接热点并运行</p></body></html>");
    delay(1000);
    ESP.restart();
}

void ConfigManager::handleNotFound()
{
    server_->send(404, "text/plain", "404 Not Found");
}

// ============================================================
// 设置默认配置（仅在 NVS 无配置时生效）
// ============================================================
void ConfigManager::setDefaults(const char *ssid, const char *pass, const char *ip)
{
    if (strlen(config_.ssid) == 0) {
        strncpy(config_.ssid,     ssid, sizeof(config_.ssid) - 1);
        strncpy(config_.password, pass, sizeof(config_.password) - 1);
        strncpy(config_.agent_ip, ip,   sizeof(config_.agent_ip) - 1);
        Serial.printf("📋 使用默认配置: SSID=%s, Agent IP=%s\n",
            config_.ssid, config_.agent_ip);
    } else {
        Serial.println("📂 使用 NVS 中保存的配置");
    }
}

// ============================================================
// NVS 读写
// ============================================================
void ConfigManager::saveToNVS()
{
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, "ssid", config_.ssid);
        nvs_set_str(nvs, "pass", config_.password);
        nvs_set_str(nvs, "ip",   config_.agent_ip);
        nvs_commit(nvs);
        nvs_close(nvs);
        Serial.println("✅ 配置已保存到 NVS 闪存");
    }
}

bool ConfigManager::loadFromNVS()
{
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READONLY, &nvs) != ESP_OK) return false;

    size_t len;
    len = sizeof(config_.ssid);
    nvs_get_str(nvs, "ssid", config_.ssid, &len);
    len = sizeof(config_.password);
    nvs_get_str(nvs, "pass", config_.password, &len);
    len = sizeof(config_.agent_ip);
    nvs_get_str(nvs, "ip",   config_.agent_ip, &len);

    nvs_close(nvs);

    // ⚠️ 确保字符串都以 '\0' 结尾，防止 printf/fromString 读到越界脏数据
    config_.ssid[sizeof(config_.ssid) - 1]         = '\0';
    config_.password[sizeof(config_.password) - 1] = '\0';
    config_.agent_ip[sizeof(config_.agent_ip) - 1] = '\0';

    if (strlen(config_.ssid) > 0) {
        Serial.printf("📂 从 NVS 读取配置: SSID=%s, Agent IP=%s\n",
            config_.ssid, config_.agent_ip);
        return true;
    }
    return false;
}

void ConfigManager::clearConfig()
{
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    memset(&config_, 0, sizeof(config_));
    Serial.println("🗑️ 配置已清除");
}
