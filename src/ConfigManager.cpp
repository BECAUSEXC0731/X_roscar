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
    loadKinematicsDefaults();
}

// 运动学默认值：与改造前的四轮差速模型完全一致，老固件升级后行为不变
void ConfigManager::loadKinematicsDefaults()
{
    config_.kin_type    = 0;        // 四轮差速
    config_.track       = 165.0f;   // 轮距 mm
    config_.wheel_base  = 0.0f;     // 轴距 mm（差速不用）
    config_.omega_scale = 1.0f;     // 不校准
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
    delay(100);

    bool ok = WiFi.softAP("CAR_CONFIG");
    if (!ok) {
        // 之前还处于 STA 连接中/驱动状态异常时，第一次起热点可能失败，重试一次
        Serial.println("⚠️ softAP 启动失败，重试...");
        WiFi.softAPdisconnect(true);
        delay(200);
        WiFi.mode(WIFI_AP);
        delay(100);
        ok = WiFi.softAP("CAR_CONFIG");
    }

    IPAddress apIP = WiFi.softAPIP();
    Serial.println("\n========================================");
    Serial.printf("配网模式已启动 (softAP: %s)\n", ok ? "OK" : "失败！");
    Serial.println("手机连接热点: CAR_CONFIG（无密码）");
    Serial.printf("  打开浏览器访问: http://%s\n", apIP.toString().c_str());
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
select{width:100%;padding:10px;margin:4px 0 8px 0;border:1px solid #ddd;border-radius:5px;box-sizing:border-box;font-size:14px;background:#fff}
hr{border:none;border-top:1px solid #eee;margin:16px 0}
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

<hr>
<label>底盘类型:</label>
<select name="kin_type">
)rawliteral";
    html += "<option value=\"0\"";
    if (config_.kin_type == 0) html += " selected";
    html += ">四轮差速</option>";
    html += "<option value=\"1\"";
    if (config_.kin_type == 1) html += " selected";
    html += ">麦克纳姆轮</option>";
    html += R"rawliteral(</select>

<label>轮距 (mm，左右轮中心距):</label>
<input name="track" type="number" step="1" min="50" max="1000" value=")rawliteral";
    html += String(config_.track, 1);
    html += R"rawliteral(">

<label>轴距 (mm，前后轮中心距，差速可留 0):</label>
<input name="wheel_base" type="number" step="1" min="0" max="1000" value=")rawliteral";
    html += String(config_.wheel_base, 1);
    html += R"rawliteral(">

<label>角速度校准系数 (1.00 = 不校准):</label>
<input name="omega_scale" type="number" step="0.01" min="0.5" max="2" value=")rawliteral";
    html += String(config_.omega_scale, 3);
    html += R"rawliteral(">
<p class="hint">原地旋转比指令转得多就调小，转不够就调大</p>

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

    // ── 运动学参数：逐项校验，任何一项不合法就整体不保存 ──
    //    避免出现"SSID 存了新值、轴距还是旧值"的半套配置
    int   kin_type    = server_->hasArg("kin_type")    ? server_->arg("kin_type").toInt()      : config_.kin_type;
    float track       = server_->hasArg("track")       ? server_->arg("track").toFloat()       : config_.track;
    float wheel_base  = server_->hasArg("wheel_base")  ? server_->arg("wheel_base").toFloat()  : config_.wheel_base;
    float omega_scale = server_->hasArg("omega_scale") ? server_->arg("omega_scale").toFloat() : config_.omega_scale;

    String err;
    if (kin_type != 0 && kin_type != 1)          err = "底盘类型只能是 0(差速) 或 1(麦克纳姆)";
    else if (track < 50 || track > 1000)         err = "轮距应在 50 ~ 1000 mm 之间";
    else if (wheel_base < 0 || wheel_base > 1000) err = "轴距应在 0 ~ 1000 mm 之间";
    else if (omega_scale < 0.5f || omega_scale > 2.0f) err = "角速度校准系数应在 0.5 ~ 2.0 之间";
    else if (kin_type == 1 && wheel_base <= 0)   err = "麦克纳姆轮必须填写轴距（前后轮中心距）";

    if (err.length() > 0) {
        server_->send(200, "text/html",
            "<html><body style='font-family:Arial;margin:40px'>"
            "<h3>❌ " + err + "</h3><a href='/'>← 返回</a></body></html>");
        return;
    }

    strncpy(config_.ssid,     ssid.c_str(), sizeof(config_.ssid) - 1);
    strncpy(config_.password, pass.c_str(), sizeof(config_.password) - 1);
    strncpy(config_.agent_ip, ip.c_str(),   sizeof(config_.agent_ip) - 1);
    config_.kin_type    = (uint8_t)kin_type;
    config_.track       = track;
    config_.wheel_base  = wheel_base;
    config_.omega_scale = omega_scale;

    bool saved = saveToNVS();

    // 把真正写进去的内容回显到网页上，避免"以为存好了其实没有"
    String msg = "<html><body style='font-family:Arial;margin:40px;text-align:center'>";
    if (saved) {
        msg += "<h3>✅ 配置已保存</h3><p>SSID: <b>" + ssid + "</b><br>密码长度: " +
               String(pass.length()) + "<br>Agent IP: <b>" + ip + "</b></p>" +
               "<p>底盘: <b>" + (kin_type == 1 ? String("麦克纳姆轮") : String("四轮差速")) +
               "</b><br>轮距: <b>" + String(track, 1) + " mm</b>" +
               "<br>轴距: <b>" + String(wheel_base, 1) + " mm</b>" +
               "<br>角速度系数: <b>" + String(omega_scale, 3) + "</b></p>" +
               "<p>重启后将连接这个热点<br>（必须是 <b>2.4GHz</b> 热点，ESP32 不支持 5GHz）</p>" +
               "<p>正在重启...</p>";
    } else {
        msg += "<h3>❌ 写入 NVS 失败</h3><p>配置没有保存，请重试</p><a href='/'>← 返回</a>";
    }
    msg += "</body></html>";
    server_->send(200, "text/html", msg);

    if (!saved) return;     // 没存进去就别重启，免得又回到配网

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
bool ConfigManager::saveToNVS()
{
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READWRITE, &nvs) != ESP_OK) {
        Serial.println("❌ NVS 打不开，配置保存失败");
        return false;
    }

    esp_err_t e_ssid = nvs_set_str(nvs, "ssid", config_.ssid);
    esp_err_t e_pass = nvs_set_str(nvs, "pass", config_.password);
    esp_err_t e_ip   = nvs_set_str(nvs, "ip",   config_.agent_ip);
    // 运动学参数：NVS 没有 float 类型，长度统一按 mm 取整、系数按 ×1000 存
    esp_err_t e_kin  = nvs_set_u8 (nvs, "kin_type",    config_.kin_type);
    esp_err_t e_trk  = nvs_set_i32(nvs, "track",       (int32_t)lroundf(config_.track));
    esp_err_t e_wb   = nvs_set_i32(nvs, "wheelbase",   (int32_t)lroundf(config_.wheel_base));
    esp_err_t e_wos  = nvs_set_i32(nvs, "omega_scale", (int32_t)lroundf(config_.omega_scale * 1000.0f));
    esp_err_t e_cmt  = nvs_commit(nvs);
    nvs_close(nvs);

    if (e_ssid != ESP_OK || e_pass != ESP_OK || e_ip != ESP_OK || e_cmt != ESP_OK ||
        e_kin != ESP_OK || e_trk != ESP_OK || e_wb != ESP_OK || e_wos != ESP_OK) {
        Serial.printf("❌ 配置写入失败: ssid=%d pass=%d ip=%d kin=%d trk=%d wb=%d wos=%d commit=%d\n",
                      (int)e_ssid, (int)e_pass, (int)e_ip, (int)e_kin,
                      (int)e_trk, (int)e_wb, (int)e_wos, (int)e_cmt);
        return false;
    }

    // 只打印密码长度，不打印密码本身
    Serial.printf("✅ 配置已保存到 NVS: SSID=%s, 密码长度=%u, Agent IP=%s\n",
                  config_.ssid, (unsigned)strlen(config_.password), config_.agent_ip);
    Serial.printf("   运动学: 模型=%u 轮距=%.1fmm 轴距=%.1fmm 角速度系数=%.3f\n",
                  (unsigned)config_.kin_type, config_.track, config_.wheel_base, config_.omega_scale);
    return true;
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

    // 运动学参数：老固件的 NVS 里没有这些 key，读失败就保留构造函数里的默认值
    uint8_t kin_type = 0;
    if (nvs_get_u8(nvs, "kin_type", &kin_type) == ESP_OK && kin_type <= 1)
        config_.kin_type = kin_type;

    int32_t iv = 0;
    if (nvs_get_i32(nvs, "track", &iv) == ESP_OK && iv >= 50 && iv <= 1000)
        config_.track = (float)iv;
    if (nvs_get_i32(nvs, "wheelbase", &iv) == ESP_OK && iv >= 0 && iv <= 1000)
        config_.wheel_base = (float)iv;
    if (nvs_get_i32(nvs, "omega_scale", &iv) == ESP_OK && iv >= 500 && iv <= 2000)
        config_.omega_scale = (float)iv / 1000.0f;

    nvs_close(nvs);

    // ⚠️ 确保字符串都以 '\0' 结尾，防止 printf/fromString 读到越界脏数据
    config_.ssid[sizeof(config_.ssid) - 1]         = '\0';
    config_.password[sizeof(config_.password) - 1] = '\0';
    config_.agent_ip[sizeof(config_.agent_ip) - 1] = '\0';

    if (strlen(config_.ssid) > 0) {
        Serial.printf("📂 从 NVS 读取配置: SSID=%s, Agent IP=%s\n",
            config_.ssid, config_.agent_ip);
        Serial.printf("   运动学: 模型=%u 轮距=%.1fmm 轴距=%.1fmm 角速度系数=%.3f\n",
            (unsigned)config_.kin_type, config_.track, config_.wheel_base, config_.omega_scale);
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
    loadKinematicsDefaults();
    Serial.println("🗑️ 配置已清除");
}
