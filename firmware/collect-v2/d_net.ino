/*
 * d_net.ino —— Wi-Fi 基础联网（只做联网，不接 AI / 不做对话）
 *
 * 硬约束：
 *   · **不阻塞主循环**：所有 WiFi 调用都在 netTask（core 0，优先级 1）里做，
 *     主循环只读几个原子状态。绝不出现 `while (WiFi.status() != WL_CONNECTED)`
 *     这种写法 —— 那会让 OBD 采集停摆。
 *   · 扫描用异步 `scanNetworks(true)` + 轮询 `scanComplete()`，不阻塞。
 *   · **密码永不进日志**：只记 SSID 和密码长度。
 *   · ESP32 只有 2.4 GHz，界面不提供 5 GHz 选项。
 *   · 断网自动重连，但重连失败不影响 BLE / SD。
 */
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <string.h>

#include "v2.h"

#define NET_MAX_SCAN 8

struct NetAp {
    char ssid[33];
    int8_t rssi;
    uint8_t ch;
};

static NetAp s_ap[NET_MAX_SCAN];
static int   s_apN = 0;

static char s_ssid[33] = {0};
static char s_pass[65] = {0};
static bool s_hasCred = false;

static volatile int  s_state = 0;      /* 0 off / 1 connecting / 2 connected / 3 failed / 4 scanning */
static volatile bool s_scanReq = false;
static volatile bool s_connReq = false;
static volatile bool s_discReq = false;
static volatile bool s_testReq = false;
static volatile bool s_autoReconn = true;
static char s_ip[20] = "0.0.0.0";
static char s_testResult[48] = "-";
static int  s_rssiNow = 0;
static uint32_t s_lastTry = 0;

static void credLoad()
{
    Preferences p;
    p.begin("sx4", true);
    String ss = p.getString("ssid", "");
    String pw = p.getString("pass", "");
    p.end();
    if (ss.length()) {
        snprintf(s_ssid, sizeof(s_ssid), "%s", ss.c_str());
        snprintf(s_pass, sizeof(s_pass), "%s", pw.c_str());
        s_hasCred = true;
    }
}

void netSetCred(const char *ssid, const char *pass)
{
    if (!ssid || !ssid[0]) {
        Serial.println(F("{\"ev\":\"err\",\"why\":\"empty ssid\"}"));
        return;
    }
    Preferences p;
    p.begin("sx4", false);
    p.putString("ssid", ssid);
    p.putString("pass", pass ? pass : "");
    p.end();
    snprintf(s_ssid, sizeof(s_ssid), "%s", ssid);
    snprintf(s_pass, sizeof(s_pass), "%s", pass ? pass : "");
    s_hasCred = true;
    /* ★ 只记 SSID 与密码长度，绝不记密码本身 */
    Serial.printf("{\"ev\":\"wifi-cred\",\"ssid\":\"%s\",\"pass_len\":%u,\"saved\":true}\n",
                  s_ssid, (unsigned)(pass ? strlen(pass) : 0));
    s_connReq = true;
}

bool netHasCred() { return s_hasCred; }

const char *netStateStr()
{
    switch (s_state) {
    case 0: return "off";
    case 1: return "connecting";
    case 2: return "connected";
    case 3: return "failed";
    default: return "scanning";
    }
}

const char *netIpStr() { return s_ip; }
int netRssi() { return s_rssiNow; }
int netScanCount() { return s_apN; }

void netScanList(char *out, size_t n)
{
    if (!out || !n) return;
    size_t o = 0;
    o += (size_t)snprintf(out + o, n - o, "[");
    for (int i = 0; i < s_apN && o + 48 < n; i++) {
        o += (size_t)snprintf(out + o, n - o, "%s{\"ssid\":\"%s\",\"rssi\":%d,\"ch\":%u}",
                              i ? "," : "", s_ap[i].ssid, s_ap[i].rssi, (unsigned)s_ap[i].ch);
    }
    snprintf(out + o, n - o, "]");
}

void netApGet(int i, char *ssid, size_t n, int *rssi, unsigned *ch)
{
    if (i < 0 || i >= s_apN) {
        if (ssid && n) ssid[0] = 0;
        if (rssi) *rssi = 0;
        if (ch) *ch = 0;
        return;
    }
    if (ssid && n) snprintf(ssid, n, "%s", s_ap[i].ssid);
    if (rssi) *rssi = s_ap[i].rssi;
    if (ch) *ch = s_ap[i].ch;
}

const char *netSsid() { return s_ssid; }

void netStatusJson()
{
    Serial.printf("{\"ev\":\"wifi\",\"state\":\"%s\",\"has_cred\":%d,\"ssid\":\"%s\","
                  "\"ip\":\"%s\",\"rssi\":%d,\"gateway\":\"%s\",\"ap\":%d,\"test\":\"%s\"}\n",
                  netStateStr(), s_hasCred ? 1 : 0, s_ssid, s_ip, s_rssiNow,
                  WiFi.status() == WL_CONNECTED ? WiFi.gatewayIP().toString().c_str() : "-",
                  s_apN, s_testResult);
}

/* ================= 任务 ================= */
static void netTask(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(120));

        if (s_scanReq) {
            s_scanReq = false;
            s_state = 4;
            Serial.println(F("{\"ev\":\"wifi-scan-begin\"}"));
            WiFi.scanDelete();
            const int r = WiFi.scanNetworks(true /*async*/, true /*show hidden*/);
            if (r == WIFI_SCAN_RUNNING) {
                uint32_t t0 = millis();
                int n = WIFI_SCAN_RUNNING;
                while (n == WIFI_SCAN_RUNNING && millis() - t0 < 12000) {
                    vTaskDelay(pdMS_TO_TICKS(150));
                    n = WiFi.scanComplete();
                }
                if (n >= 0) {
                    s_apN = (n > NET_MAX_SCAN) ? NET_MAX_SCAN : n;
                    for (int i = 0; i < s_apN; i++) {
                        snprintf(s_ap[i].ssid, sizeof(s_ap[i].ssid), "%s",
                                 WiFi.SSID(i).c_str());
                        s_ap[i].rssi = (int8_t)WiFi.RSSI(i);
                        s_ap[i].ch = (uint8_t)WiFi.channel(i);
                        Serial.printf("{\"ev\":\"ap\",\"i\":%d,\"ssid\":\"%s\",\"rssi\":%d,"
                                      "\"ch\":%u}\n", i, s_ap[i].ssid, s_ap[i].rssi,
                                      (unsigned)s_ap[i].ch);
                    }
                    Serial.printf("{\"ev\":\"wifi-scan-end\",\"n\":%d}\n", s_apN);
                } else {
                    Serial.printf("{\"ev\":\"wifi-scan-end\",\"n\":0,\"err\":%d}\n", n);
                }
            } else {
                Serial.printf("{\"ev\":\"wifi-scan-end\",\"n\":0,\"err\":%d}\n", r);
            }
            WiFi.scanDelete();
            s_state = (WiFi.status() == WL_CONNECTED) ? 2 : (s_hasCred ? 3 : 0);
        }

        if (s_discReq) {
            s_discReq = false;
            WiFi.disconnect(true, false);
            s_state = 0;
            snprintf(s_ip, sizeof(s_ip), "0.0.0.0");
            Serial.println(F("{\"ev\":\"wifi-off\"}"));
        }

        if (s_connReq) {
            s_connReq = false;
            if (!s_hasCred) {
                Serial.println(F("{\"ev\":\"err\",\"why\":\"no wifi cred\"}"));
            } else {
                s_state = 1;
                Serial.printf("{\"ev\":\"wifi-join\",\"ssid\":\"%s\"}\n", s_ssid);
                WiFi.begin(s_ssid, s_pass);          /* 非阻塞 */
                s_lastTry = millis();
            }
        }

        /* 状态轮询 */
        const wl_status_t st = WiFi.status();
        if (st == WL_CONNECTED) {
            if (s_state != 2) {
                snprintf(s_ip, sizeof(s_ip), "%s", WiFi.localIP().toString().c_str());
                s_rssiNow = WiFi.RSSI();
                s_state = 2;
                Serial.printf("{\"ev\":\"wifi-up\",\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,"
                              "\"gw\":\"%s\",\"ch\":%u}\n",
                              s_ssid, s_ip, s_rssiNow,
                              WiFi.gatewayIP().toString().c_str(), (unsigned)WiFi.channel());
            }
            s_rssiNow = WiFi.RSSI();
        } else {
            if (s_state == 2) {
                s_state = 3;
                Serial.printf("{\"ev\":\"wifi-down\",\"status\":%d}\n", (int)st);
            }
            /* 自动重连：间隔 15 秒，绝不 while 等待 */
            if (s_hasCred && s_autoReconn && s_state != 1 && millis() - s_lastTry > 15000) {
                s_lastTry = millis();
                Serial.println(F("{\"ev\":\"wifi-retry\"}"));
                WiFi.begin(s_ssid, s_pass);
                s_state = 1;
            }
        }

        if (s_testReq) {
            s_testReq = false;
            if (WiFi.status() != WL_CONNECTED) {
                snprintf(s_testResult, sizeof(s_testResult), "no-link");
            } else {
                /* 用网关的 80 端口做一次 3 秒 TCP 连通测试，不阻塞主循环 */
                WiFiClient c;
                const IPAddress gw = WiFi.gatewayIP();
                const bool ok = c.connect(gw, 80, 3000);
                c.stop();
                snprintf(s_testResult, sizeof(s_testResult), ok ? "tcp-gw:80 ok" : "tcp-gw:80 fail");
            }
            Serial.printf("{\"ev\":\"net-test\",\"result\":\"%s\",\"ip\":\"%s\"}\n",
                          s_testResult, s_ip);
        }
    }
}

/* ================= 对外 ================= */
void netBegin()
{
    credLoad();
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(true);        /* 让射频优先给 BLE，采集优先 */

    xTaskCreatePinnedToCore(netTask, "net", 4096, nullptr, 1, nullptr, 0);

    Serial.printf("{\"ev\":\"wifi-init\",\"has_cred\":%d,\"ssid\":\"%s\",\"band\":\"2.4GHz\"}\n",
                  s_hasCred ? 1 : 0, s_ssid);
    if (s_hasCred) s_connReq = true;   /* 有凭据才开机自动连 */
}

void netScan()     { s_scanReq = true; }
void netConnect()  { s_connReq = true; }
void netDisconnect(){ s_discReq = true; }
void netTest()     { s_testReq = true; }
void netTaskStart(){ /* 任务已在 netBegin 里创建，占位 */ }
