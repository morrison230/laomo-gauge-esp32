/*
 * collect v2 —— ESP32 OBD Gauge 第二版
 *
 * ★ 与 V1 的根本区别（针对 2026-11-12 实车审计）
 *   1. **开机永远 REAL**。删除 V1「开机 3 秒内 GPIO0 为低 → SIM」的判据。
 *      V1 在车上只插车载 USB 时该线不受控，45 次里有 16 次被误判成 SIM，
 *      于是 36,764 次"OBD 数据"全是模拟器注入的，真实应答 0 条。
 *      V2 的 SIM 只能由串口命令 `SIM` 在运行时启动，且全程打红条、逐行标 org=sim。
 *   2. **三态分离**：link / svc / ecu 各自独立记录，不再用"看起来连上了"冒充有数据。
 *   3. **设备绑定**：扫描列名 + 地址 + 地址类型 + RSSI，用户选定后存 NVS；
 *      以后只认这个地址，绝不因为别的设备信号强就连过去。
 *   4. **数据来源逐行标注**：每行 JSON 都有 "org":"real"|"sim" 与 "sess":N。
 *   5. **JSON 先校验再落盘**：jsonValidate() 不过就拒写并记 json-invalid。
 *      V1 的坏行曾由经纬度前导零（如 `"lon":0123.45`）造成。
 *
 * ★ 保留 V1 已验证的硬结论（不得改回去）
 *   · 初始化顺序：LCD init + 首屏 → SD.begin() → 开文件 → GPS → BLE
 *     （SD 先占 VSPI 再让 GFX 接管会 → open() 立刻 ENODEV(errno=19) 且屏幕黑）
 *   · 写卡走 POSIX：::write / ::fsync / ::close，每步查返回码
 *     （File::flush() 是 void，VFSFileImpl::flush() 丢弃 fflush/fsync 返回值）
 *   · 开文件用 O_CREAT|O_EXCL，绝不覆盖已有 trip 文件
 *
 * ★ 本版不做
 *   · 不引入 LVGL（本工程从来没有 LVGL，直绘通路已实屏验收，不为新页面推倒重来）
 *   · 不启用触摸（FT6336 从未在本工程验证过 I2C，贸然加入会拖累已通的 LCD 初始化）
 *   · 不开发 DeepSeek / 对话；只做 Wi-Fi 联网基础能力
 *   · 只发 Mode 01 只读查询，绝不向 ECU 写任何东西
 *
 * 刷写范围：只写 0x10000 应用分片。不动分区表 / NVS / otadata / eFuse。
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "TCA9554.h"
#include "bootkey_fsm.h"
#include <BLEDevice.h>
#include <BLEScan.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_system.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <mbedtls/sha256.h>

#include "v2.h"

/* ================= 全局定义 ================= */
V2Mode  g_mode = MODE_REAL;
V2Link  g_link = LINK_IDLE;
V2Svc   g_svc  = SVC_UNKNOWN;
V2Ecu   g_ecu  = ECU_NONE;
V2Page  g_page = PAGE_MAIN;
V2Sd    g_uisd = SD_NONE;

uint32_t g_sess   = 0;
uint32_t g_bootMs = 0;
char     g_resetWhy[20] = "?";

uint32_t g_realReq = 0, g_realOk = 0, g_realNoData = 0, g_realTo = 0;
uint32_t g_simReq  = 0, g_simOk  = 0;

static Preferences g_prefs;

/* ================= 复位原因 ================= */
static const char *resetReasonStr()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
    }
}

/* ================= 会话号（NVS 自增，用于把同一天多次上电区分开） ================= */
static uint32_t nextSession()
{
    g_prefs.begin("sx4", false);
    uint32_t s = g_prefs.getUInt("sess", 0) + 1;
    g_prefs.putUInt("sess", s);
    g_prefs.end();
    return s;
}

/* ================= 模式切换（只有串口能改） ================= */
static void setMode(V2Mode m, const char *why)
{
    if (g_mode == m) return;
    g_mode = m;
    Serial.printf("\n{\"v\":2,\"t\":%lu,\"src\":\"sys\",\"ev\":\"mode\",\"to\":\"%s\","
                  "\"by\":\"%s\",\"sess\":%lu}\n",
                  (unsigned long)millis(), m == MODE_SIM ? "sim" : "real", why,
                  (unsigned long)g_sess);
    if (m == MODE_SIM) {
        obdDisconnect();          /* SIM 期间不占 BLE，也不发任何真请求 */
        g_ecu = ECU_NONE;
    } else {
        g_ecu = ECU_NONE;
        g_link = LINK_IDLE;
    }
    uiDraw(true);
}

/* ================= SIM：合成数据（只有命令能进来） ================= */
static void simTick()
{
    static uint32_t last = 0;
    const uint32_t now = millis();
    if (now - last < 200) return;
    last = now;
    static int n = 0;
    n++;

    char line[340];

    /* 每 5 拍一条 GGA。坐标用十进制（V2 的 GPS 也是十进制，避免前导零） */
    if (n % 5 == 0) {
        const int q = (n > 20) ? 1 : 0;
        const int nsat = (n > 20) ? 9 : 0;
        snprintf(line, sizeof(line),
                 "{\"v\":2,\"t\":%lu,\"src\":\"gps\",\"ev\":\"gga\",\"org\":\"sim\",\"sess\":%lu,"
                 "\"q\":%d,\"sat\":%d,\"hdop\":0.9,\"alt_m\":35.2,"
                 "\"lat\":0.000000,\"lon\":0.000000,\"utc\":\"000000.00\"}",
                 (unsigned long)now, (unsigned long)g_sess, q, nsat);
        logJson(line);
        gpsSimSet(q, nsat);
    }

    /* 合成 OBD：五个 PID 轮转，含 010C/010D（V1 的 SIM 只模拟了 0105/0142） */
    static const char *cyc[5] = { "0105", "0142", "010C", "010D", "0104" };
    const char *cmd = cyc[n % 5];
    String raw;
    if (!strcmp(cmd, "0105")) {
        raw = (n % 8 == 0) ? "7E9 03 41 05 5B\r\n7E8 03 41 05 5A\r\n>"
                           : "7E8 03 41 05 5A\r\n7E9 03 41 05 5B\r\n>";
    } else if (!strcmp(cmd, "0142")) {
        const char *a = (n % 20 == 0) ? "7E8 04 41 42 36 D8" : "7E8 04 41 42 30 70";
        raw = (String(a) + "\r\n7E9 04 41 42 2E E0\r\n>");
    } else if (!strcmp(cmd, "010C")) {
        /* 800 rpm 怠速：0x0C80 → A=0x0C,B=0x80 → (12*256+128)/4 = 800 */
        raw = "7E8 04 41 0C 0C 80\r\n7E9 04 41 0C 0C 00\r\n>";
    } else if (!strcmp(cmd, "010D")) {
        raw = "7E8 03 41 0D 00\r\n7E9 03 41 0D 00\r\n>";
    } else {
        raw = "7E8 03 41 04 40\r\n7E9 03 41 04 3C\r\n>";
    }
    g_simReq++;
    obdFeedRaw(cmd, raw, 40 + (n % 7), /*isSim=*/true);
}

/* ================= 串口命令 ================= */
static void printHelp()
{
    Serial.println(F("\n--- collect v2 commands ---"));
    Serial.println(F("HELP | STATUS | DIAG | PHASE | PIDS | GPS?"));
    Serial.println(F("SIM | REAL                     (SIM 只能这样进)"));
    Serial.println(F("BLE SCAN | BLE LIST | BLE PICK <n> | BLE CONNECT | BLE FORGET"));
    Serial.println(F("WIFI SCAN | WIFI SET <ssid> <pass> | WIFI CONNECT | WIFI OFF | WIFI STATUS | NET TEST"));
    Serial.println(F("TOUCH SCAN | TOUCH CALCLEAR | TOUCH LOG | TOUCH MARK ON|OFF | TOUCH DIAG"));
    Serial.println(F("SAVE | REBOOT"));
    Serial.println(F("SD PING | SD INFO | SD LS /trips | SD GET /trips/trip-NNN.jsonl | SD CANCEL"));
    Serial.println(F("BOOT key: short=next page, long(>0.8s)=action on that page"));
    Serial.println(F("---"));
}

static void cmdStatus()
{
    Serial.printf("\n{\"ev\":\"status\",\"ver\":\"%s\",\"sess\":%lu,\"uptime_ms\":%lu,"
                  "\"mode\":\"%s\",\"page\":%u,\"link\":\"%s\",\"svc\":\"%s\",\"ecu\":\"%s\","
                  "\"real\":{\"req\":%lu,\"ok\":%lu,\"nodata\":%lu,\"to\":%lu},"
                  "\"sim\":{\"req\":%lu,\"ok\":%lu},"
                  "\"stale_ms\":%lu,\"sd\":%u,\"rec\":%d,\"lines\":%lu,"
                  "\"cand\":%d,\"saved\":%d,\"wifi\":\"%s\",\"ip\":\"%s\"}\n",
                  V2_APP_VERSION, (unsigned long)g_sess, (unsigned long)(millis() - g_bootMs),
                  g_mode == MODE_SIM ? "SIM" : "REAL", (unsigned)g_page,
                  obdLinkStr(), obdSvcStr(), obdEcuStr(),
                  (unsigned long)g_realReq, (unsigned long)g_realOk,
                  (unsigned long)g_realNoData, (unsigned long)g_realTo,
                  (unsigned long)g_simReq, (unsigned long)g_simOk,
                  (unsigned long)obdStaleMs(), (unsigned)g_uisd,
                  storeIsRecording() ? 1 : 0, (unsigned long)storeLines(),
                  obdCandCount(), obdSavedBound() ? 1 : 0,
                  netStateStr(), netIpStr());
}

static void handleCmd(char *line)
{
    /* 去掉首尾空白 */
    while (*line == ' ' || *line == '\t') line++;
    size_t L = strlen(line);
    while (L && (line[L - 1] == '\r' || line[L - 1] == '\n' || line[L - 1] == ' ')) line[--L] = 0;
    if (!L) return;

    /* 命令统一大写做比较，但参数保持原样（SSID/密码区分大小写） */
    char up[24] = {0};
    const char *sp = strchr(line, ' ');
    size_t cmdLen = sp ? (size_t)(sp - line) : L;
    for (size_t i = 0; i < cmdLen && i < sizeof(up) - 1; i++) up[i] = (char)toupper(line[i]);

    if (!strcmp(up, "HELP")) { printHelp(); return; }
    if (!strcmp(up, "STATUS")) { cmdStatus(); return; }
    if (!strcmp(up, "SD")) { tripExportCommand(sp ? sp + 1 : ""); return; }
    if (!strcmp(up, "DIAG")) { obdDiagJson(); touchDiagJson(); cmdStatus(); netStatusJson(); return; }
    if (!strcmp(up, "PIDS")) {
        for (int i = 0; i < obdPidCount(); i++) {
            const PidRec *p = obdPid(i);
            bool fresh = false;
            obdVal(i, &fresh, nullptr);
            const char *ecu = !p->hasVal ? "null" : p->lastEcu == 0x7E8 ? "\"7E8\"" :
                              p->lastEcu == 0x7E9 ? "\"7E9\"" : "null";
            Serial.printf("{\"ev\":\"pid\",\"cmd\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\","
                          "\"req\":%lu,\"ok\":%lu,\"nodata\":%lu,\"bad\":%lu,\"to\":%lu,"
                          "\"supported\":%u,\"has\":%d,\"val\":%.2f,\"age_ms\":%lu,"
                          "\"ecu\":%s,\"result\":\"%s\",\"fresh\":%s}\n",
                          p->cmd, p->label, p->unit,
                          (unsigned long)p->req, (unsigned long)p->ok,
                          (unsigned long)p->nodata, (unsigned long)p->badframe,
                          (unsigned long)p->to, (unsigned)p->supported, p->hasVal ? 1 : 0,
                          p->lastVal,
                          p->lastOkMs ? (unsigned long)(millis() - p->lastOkMs) : 999999UL,
                          ecu, obdPidResultStr(*p), fresh ? "true" : "false");
        }
        return;
    }
    if (!strcmp(up, "GPS?")) { gpsStr(nullptr, 0); return; }

    /* ★ r2 新增：阶段快照。台上/车上任何时刻都能一眼看到
       「现在走到第几阶段、结果是什么、三条 AT 各自如何、0100 结论、绑定状态」。 */
    if (!strcmp(up, "PHASE")) {
        char pid[26];
        obdLastPidStr(pid, sizeof(pid));
        Serial.printf("{\"ev\":\"phase\",\"sess\":%lu,\"ph\":%u,\"of\":%u,\"code\":\"%s\","
                      "\"st\":\"%s\",\"ms\":%lu,\"note\":\"%s\",\"conn\":\"%s\","
                      "\"pid_busy\":%d,\"pid_cmd\":\"%s\","
                      "\"atz\":\"%s\",\"ate0\":\"%s\",\"ath1\":\"%s\","
                      "\"p0100\":\"%s\",\"nfy\":%lu,\"reg\":%d,\"cccd\":\"UNKNOWN\","
                      "\"fail\":%u,\"fail_note\":\"%s\",\"fail1\":%u,\"fail1_note\":\"%s\","
                      "\"bound\":%d,\"last_pid\":\"%s\",\"events\":%lu}\n",
                      (unsigned long)g_sess, (unsigned)obdPhaseNo(), (unsigned)obdPhaseCount(),
                      obdPhaseCode(), obdPhaseResultCode(), (unsigned long)obdPhaseMs(),
                      obdLastNote(), obdConnStepStr(), obdPidBusy() ? 1 : 0, obdSentCmd(),
                      obdAtResultStr(0), obdAtResultStr(1), obdAtResultStr(2),
                      obdProbeStr(), (unsigned long)obdNotifyCount(), obdNotifyFlag() ? 1 : 0,
                      (unsigned)obdLastFailPhase(), obdLastFailNote(),
                      (unsigned)obdFirstFailPhase(), obdFirstFailNote(),
                      obdSavedBound() ? 1 : 0, pid, (unsigned long)obdStageEvents());
        return;
    }

    if (!strcmp(up, "TOUCH")) {
        const char *arg = sp ? sp + 1 : "";
        if (!strncmp(arg, "SCAN", 4) || !arg[0]) {
            char list[200] = {0};
            const int n = touchScanBus(list, sizeof(list));
            Serial.printf("{\"ev\":\"i2c-scan\",\"n\":%d,\"addrs\":%s}\n", n, list);
            touchDiagJson();
            return;
        }
        if (!strncmp(arg, "CALCLEAR", 8)) { touchForgetCal(); return; }
        if (!strncmp(arg, "LOG", 3))      { touchLogDump(); return; }
        if (!strncmp(arg, "MARK", 4)) {
            const char *v = arg + 4;
            while (*v == ' ') v++;
            const bool on = (v[0] == '1' || v[0] == 'O' || v[0] == 'o');
            uiMarkSet(on);
            Serial.printf("{\"ev\":\"touch-mark\",\"on\":%d}\n", on ? 1 : 0);
            return;
        }
        if (!strncmp(arg, "DIAG", 4))     { touchDiagJson(); return; }
        Serial.println(F("{\"ev\":\"err\",\"why\":\"TOUCH SCAN|CALCLEAR|DIAG\"}"));
        return;
    }

    if (!strcmp(up, "SIM")) {
        if (g_mode == MODE_SIM) { Serial.println(F("{\"ev\":\"ack\",\"note\":\"already SIM\"}")); return; }
        setMode(MODE_SIM, "serial");
        Serial.println(F("{\"ev\":\"ack\",\"note\":\"SIM ON - data tagged org=sim, real stats untouched\"}"));
        return;
    }
    if (!strcmp(up, "REAL")) {
        setMode(MODE_REAL, "serial");
        Serial.println(F("{\"ev\":\"ack\",\"note\":\"REAL ON\"}"));
        return;
    }

    /* --- BLE --- */
    if (!strcmp(up, "BLE")) {
        const char *arg = sp ? sp + 1 : "";
        if (!strncmp(arg, "SCAN", 4))       { obdRequestScan(); return; }
        if (!strncmp(arg, "LIST", 4)) {
            for (int i = 0; i < obdCandCount(); i++) {
                const BleCand *c = obdCand(i);
                Serial.printf("{\"ev\":\"cand\",\"i\":%d,\"name\":\"%s\",\"addr\":\"%s\","
                              "\"atype\":%u,\"rssi\":%d,\"ffe0\":%d}\n",
                              i, c->name, c->addr, (unsigned)c->atype, c->rssi, c->hasFfe0 ? 1 : 0);
            }
            if (!obdCandCount()) Serial.println(F("{\"ev\":\"cand\",\"note\":\"empty, run BLE SCAN\"}"));
            return;
        }
        if (!strncmp(arg, "PICK", 4)) {
            char *endp = nullptr;
            long n = strtol(arg + 4, &endp, 10);
            if (endp == arg + 4) { Serial.println(F("{\"ev\":\"err\",\"why\":\"need index\"}")); return; }
            obdPickToIndex((int)n);
            obdSelectCurrent();
            return;
        }
        if (!strncmp(arg, "CONNECT", 7))    { obdConnectSaved(); return; }
        if (!strncmp(arg, "FORGET", 6))     { obdForget(); return; }
        if (!strncmp(arg, "OFF", 3))        { obdDisconnect(); return; }
        Serial.println(F("{\"ev\":\"err\",\"why\":\"unknown BLE subcommand\"}"));
        return;
    }

    /* --- WIFI --- */
    if (!strcmp(up, "WIFI")) {
        const char *arg = sp ? sp + 1 : "";
        if (!strncmp(arg, "SCAN", 4))    { netScan(); return; }
        if (!strncmp(arg, "STATUS", 6))  { netStatusJson(); return; }
        if (!strncmp(arg, "CONNECT", 7)) { netConnect(); return; }
        if (!strncmp(arg, "OFF", 3))     { netDisconnect(); return; }
        if (!strncmp(arg, "SET", 3)) {
            /* WIFI SET <ssid> <pass>  —— 密码只在内存与 NVS，日志绝不出现 */
            char *p = (char *)arg + 3;
            while (*p == ' ') p++;
            char *ssidEnd = strchr(p, ' ');
            if (!ssidEnd) { Serial.println(F("{\"ev\":\"err\",\"why\":\"need ssid and pass\"}")); return; }
            *ssidEnd = 0;
            const char *pass = ssidEnd + 1;
            netSetCred(p, pass);
            return;
        }
        Serial.println(F("{\"ev\":\"err\",\"why\":\"unknown WIFI subcommand\"}"));
        return;
    }

    if (!strcmp(up, "NET")) {
        const char *arg = sp ? sp + 1 : "";
        if (!strncmp(arg, "TEST", 4)) { netTest(); return; }
        Serial.println(F("{\"ev\":\"err\",\"why\":\"unknown NET subcommand\"}"));
        return;
    }

    if (!strcmp(up, "SAVE")) {
        if (storeIsRecording()) { storeFinishAndSave(); }
        else Serial.println(F("{\"ev\":\"ack\",\"note\":\"nothing to save\"}"));
        return;
    }
    if (!strcmp(up, "REBOOT")) {
        Serial.println(F("{\"ev\":\"ack\",\"note\":\"rebooting\"}"));
        Serial.flush(); delay(120); ESP.restart();
    }

    Serial.printf("{\"ev\":\"err\",\"why\":\"unknown command\",\"got\":\"%s\"}\n", line);
}

static void pollSerial()
{
    static char buf[160];
    static size_t len = 0;
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\n' || c == '\r') {
            buf[len] = 0;
            if (len) handleCmd(buf);
            len = 0;
        } else if (len + 1 < sizeof(buf)) {
            buf[len++] = (char)c;
        } else {
            len = 0;   /* 超长直接丢弃，防止越界 */
        }
    }
}

/* ================= BOOT 键：短按翻页 / 长按执行 =================
   判定逻辑在 bootkey_fsm.h 里（纯状态机，可在 PC 上做模型测试）。
   旧实现没有消抖：一次 LOW、下一次 HIGH 就算短按 → 没人碰也会自己翻页。
   新规则：开机 3 秒屏蔽 + 40ms 稳定电平 + 长按毛刺必须走稳定释放判定。
   短按阈值 400ms、长按阈值 800ms 与旧实现完全相同，操作习惯不变。 */
static BootKeyFsm s_bk;

static void bootKeyTick()
{
    const int ev = s_bk.tick(digitalRead(PIN_BOOT) == LOW, millis());
    if (ev == 1)      uiNextPage();   /* 短按：换页（<400ms） */
    else if (ev == 2) uiAction();     /* 长按：当前页动作（>=800ms） */
}

/* ================= setup / loop ================= */
void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(150);

    g_bootMs = millis();
    snprintf(g_resetWhy, sizeof(g_resetWhy), "%s", resetReasonStr());
    g_sess = nextSession();

    pinMode(PIN_BOOT, INPUT_PULLUP);
    s_bk.begin(millis());          /* 开机屏蔽从此刻起算 3 秒 */

    /* ★ 开机永远 REAL。V1 在这里读 GPIO0 判 SIM，是假数据的根源，已删除。 */
    g_mode = MODE_REAL;

    Serial.println();
    logJsonf("{\"v\":2,\"t\":%lu,\"src\":\"sys\",\"ev\":\"boot\",\"ver\":\"%s\","
             "\"mode\":\"REAL\",\"sess\":%lu,\"reset\":\"%s\",\"order\":\"lcd-first\"}",
             (unsigned long)(millis() - g_bootMs), V2_APP_VERSION,
             (unsigned long)g_sess, g_resetWhy);

    /* ★ A. 先 LCD（实测结论：LCD 必须早于 SD） */
    if (!uiBegin()) {
        Serial.println(F("{\"ev\":\"lcd-missing\"}"));
    }

    /* ★ A2. 触摸（I2C 已由 uiBegin 建好；失败不影响采集） */
    if (!touchBegin()) {
        Serial.println(F("{\"ev\":\"touch-absent\",\"note\":\"按键仍可用\"}"));
    }

    /* ★ B. 再挂 SD → 恢复检查 → 开新行程文件 */
    if (storeMount()) {
        storeRecoverCheck();      /* 用 .open.json 判断上次是否异常结束 */
        storeOpenTrip();
    } else {
        Serial.println(F("{\"ev\":\"sd-absent\",\"note\":\"collection continues without card\"}"));
    }
    uiDraw(true);

    /* C. GPS */
    gpsBegin();

    /* D. BLE：只起任务骨架，扫描必须由用户/命令触发 */
    obdBegin();

    /* E. Wi-Fi：只初始化，不自动连（凭据存在才连） */
    netBegin();

    uiDraw(true);

    logJsonf("{\"ev\":\"ready\",\"sess\":%lu,\"sd\":%d,\"path\":\"%s\",\"mode\":\"REAL\","
             "\"saved_dev\":%d,\"wifi_cred\":%d}",
             (unsigned long)g_sess, storeIsRecording() ? 1 : 0, storePathVfs(),
             obdSavedBound() ? 1 : 0, netHasCred() ? 1 : 0);
    if (obdSavedBound()) obdConnectSaved();   /* 有绑定才自动重连，没有就等命令 */
}

void loop()
{
    /* 1. 采集（最高优先级，任何网络动作都不能挡在这里） */
    gpsTick();
    if (g_mode == MODE_SIM) simTick();
    obdTick();

    /* 2. 串口命令 */
    pollSerial();

    /* 3. 落盘 */
    storeTick();
    tripExportTick();

    /* 4. 每 5 秒一条 stat（体积可忽略，随车保留） */
    {
        static uint32_t statAt = 0;
        const uint32_t now = millis();
        if (now - statAt >= 5000) {
            statAt = now;
            char st[560];
            snprintf(st, sizeof(st),
                     "{\"v\":2,\"t\":%lu,\"src\":\"sys\",\"ev\":\"stat\",\"org\":\"%s\","
                     "\"sess\":%lu,\"mode\":\"%s\",\"link\":\"%s\",\"svc\":\"%s\",\"ecu\":\"%s\","
                     "\"real_req\":%lu,\"real_ok\":%lu,\"real_nodata\":%lu,\"real_to\":%lu,"
                     "\"sim_req\":%lu,\"sim_ok\":%lu,\"stale_ms\":%lu,"
                     "\"gps_q\":%d,\"gps_sat\":%d,\"lines\":%lu,\"werr\":%lu,\"rec\":%s,"
                     "\"ph\":%u,\"of\":%u,\"ph_code\":\"%s\",\"ph_st\":\"%s\",\"ph_ms\":%lu,"
                     "\"nfy\":%lu,\"p0100\":\"%s\",\"bound\":%d,\"fail\":%u}",
                     (unsigned long)now, g_mode == MODE_SIM ? "sim" : "real",
                     (unsigned long)g_sess,
                     g_mode == MODE_SIM ? "SIM" : "REAL",
                     obdLinkStr(), obdSvcStr(), obdEcuStr(),
                     (unsigned long)g_realReq, (unsigned long)g_realOk,
                     (unsigned long)g_realNoData, (unsigned long)g_realTo,
                     (unsigned long)g_simReq, (unsigned long)g_simOk,
                     (unsigned long)obdStaleMs(),
                     gpsFixQ(), gpsSats(), (unsigned long)storeLines(),
                     (unsigned long)storeErrWrites(), storeIsRecording() ? "true" : "false",
                     (unsigned)obdPhaseNo(), (unsigned)obdPhaseCount(),
                     obdPhaseCode(), obdPhaseResultCode(), (unsigned long)obdPhaseMs(),
                     (unsigned long)obdNotifyCount(), obdProbeStr(),
                     obdSavedBound() ? 1 : 0, (unsigned)obdLastFailPhase());
            logJson(st);
            uiDrawStats();     /* 绘制计数只走串口，不落卡 */
        }
    }

    /* 5. 触摸 → 页面动作（点一下就走，不阻塞）
       ★ 每次点击都打一行 touch 事件：原始坐标 / 换算坐标 / 点击时页面 / 最终命中 ID
          这样"屏幕画在哪、实际点到哪"可以直接对账，不用再靠猜。 */
    {
        int16_t tx = 0, ty = 0;
        if (touchTick(&tx, &ty)) {
            int16_t rx = 0, ry = 0;
            touchLastRaw(&rx, &ry);
            /* ★ 十字坐标必须在 uiTouch() 之前落地：uiTouch 内部就会触发整页重画，
               放到后面会让红叉晚一帧出现（看起来像没生效）。 */
            uiMarkMove(tx, ty);
            const uint8_t pg = (uint8_t)g_page;      /* 动作会改页，先记下点击时的页 */
            const uint8_t hit = uiTouch(tx, ty);
            touchLogAdd(rx, ry, tx, ty, pg, hit);
            Serial.printf("{\"ev\":\"touch\",\"page\":%u,\"raw\":[%d,%d],\"scr\":[%d,%d],"
                          "\"hitid\":%u,\"hit\":\"%s\"}\n",
                          (unsigned)pg, rx, ry, tx, ty, (unsigned)hit, uiHitName(hit));
        }
    }

    /* 6. 屏幕 */
    uiTick();

    /* 7. BOOT 键（备用操作，保留） */
    bootKeyTick();
    static V2Page lastPage = PAGE_MAIN;
    if (g_page != lastPage) {
        logJsonf("{\"v\":2,\"t\":%lu,\"src\":\"ui\",\"ev\":\"page\",\"sess\":%lu,"
                 "\"from\":%u,\"page\":%u,\"link\":\"%s\"}",
                 (unsigned long)millis(), (unsigned long)g_sess,
                 (unsigned)lastPage, (unsigned)g_page, obdLinkStr());
        lastPage = g_page;
    }
}
