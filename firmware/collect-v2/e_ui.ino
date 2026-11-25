/*
 * e_ui.ino —— 中文触摸界面（第三版：分页 + 中文 + 按钮命中可核对）
 *
 * 本次改动范围（严格受限）：
 *   · 继续用 Arduino_GFX 直绘，**不引入 LVGL**、不改显示驱动
 *   · 页面改为五页：主界面 / 蓝牙设备 / 无线网络 / 系统诊断 / 高级诊断
 *   · 触摸校准收进「高级诊断」，不再占用一个独立页面
 *   · 所有按钮的**绘制与命中判定共用同一张 BtnDef 表**，从结构上杜绝"画在这、点在那"
 *   · 数据链路（BLE/OBD/PID/ECU/GPS/SD/REAL-SIM）一行未动
 *
 * 中文：屏幕只有 ASCII 字库，所以自带 16x16 点阵（cjk_font.h）。
 *      字模由 tools/cjk_font_gen.py 从本文件实际用到的字符串生成，
 *      并有审计项保证"界面里出现的每个汉字都在字模里"，不会再有空白字。
 */
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "TCA9554.h"
#include <string.h>

#include "v2.h"
#include "cjk_font.h"

/* ================= 配色（沿用原有风格，不新增） ================= */
#define C_BG       0x18E3
#define C_CARD     0x2945
#define C_TEXT     0xFFFF
#define C_DIM      0x9CD3
#define C_OK       0x07E0
#define C_WARN     0xFD20
#define C_OFF      0x7BEF
#define C_ERR      0xF800
#define C_GREEN_BG 0x0320
#define C_LINE     0x2104
#define C_SEL      0x3AEB

/* ================= 版面常量（单点定义，绘制与命中都引用这里） ================= */
#define SCR_W        480
#define SCR_H        320
#define TOP_H        38
#define LIST_Y0      42
#define LIST_ROWS    4
#define LIST_ROW_H   38
#define LIST_ROW_GAP 41
#define PGT_Y        206      /* 分页控制行 */
#define PGT_H        32
#define HINT_Y       246      /* 列表页底部提示行 */
#define BOT_Y        272
#define BOT_BTN_Y    278
#define BOT_BTN_W    152
#define BOT_BTN_H    36

#define COL1_X       6
#define COL2_X       164
#define COL3_X       322

/* ================= 诊断页版面（r2：给「阶段区」腾出位置） =================
   顶部栏 0..38 不动；阶段区固定两行；8 项状态行压缩到 20px 行距；
   底部两行放「最近失败」与「最近一次 PID 轮询」。
   行距从 25 改到 20 只影响诊断页，其它页与按钮几何完全不变。 */
#define DIAG_Y_PH    42          /* 阶段行：阶段号/名称/结果/耗时 */
#define DIAG_Y_NTF   58          /* 通知行：注册调用/回调观察/CCCD */
#define DIAG_Y0      78          /* 8 项状态行起始 */
#define DIAG_ROW_H   20
#define DIAG_Y_FAIL  238         /* 最近失败 */
#define DIAG_Y_PID   254         /* 最近一次 PID 轮询 */

/* 高级诊断页底部三行技术进度（ASCII，字号 1 = 8px） */
#define ADV_Y_PH     232
#define ADV_Y_PRB    243
#define ADV_Y_AT     254

static TCA9554 TCA(0x20);
static Arduino_DataBus *g_bus = new Arduino_ESP32SPI(PIN_LCD_DC, PIN_LCD_CS,
                                                     PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO);
static Arduino_GFX *gfx = new Arduino_ST7796(g_bus, GFX_NOT_DEFINED, 1 /* rot=1 */, true);
static bool s_lcd = false;

static char     s_toast[28] = {0};
static uint32_t s_toastUntil = 0;
static uint32_t s_lastDraw = 0;

/* 列表分页与选中 */
static int      s_devPage = 0;
static int      s_devCur  = 0;      /* 绝对索引 */
static int      s_apPage  = 0;
static int      s_advPidPage = 0;
static constexpr int kAdvPidRows = 6;
static int      s_apCur   = 0;      /* 绝对索引 */

/* 触摸校准（二级功能，不再是页面） */
static uint8_t  s_calStep = 0;      /* 0 未开始 / 1 等左上 / 2 等右下 */
static int16_t  s_calTX0 = 0, s_calTY0 = 0;

/* 触摸十字标记（现场调试） */
static bool     s_mark = false;
static int16_t  s_markX = -1, s_markY = -1;

/* ---------- 前向声明 ---------- */
static int  cjkText(int x, int y, const char *s, uint16_t color, int scale);
static void cjkCenter(int cx, int y, const char *s, uint16_t color, int scale);
static bool hit(int16_t x, int16_t y, int rx, int ry, int rw, int rh);
static void drawBtns(const BtnDef *arr, int n);
static uint8_t hitBtns(const BtnDef *arr, int n, int16_t x, int16_t y);
static void rowRect(int i, int16_t *x, int16_t *y, int16_t *w, int16_t *h);

/* ================= 中文/ASCII 混排 ================= */
static int utf8Next(const char **p)
{
    const unsigned char *s = (const unsigned char *)*p;
    if (!s || !*s) return 0;
    if (s[0] < 0x80) { *p += 1; return s[0]; }
    if ((s[0] & 0xE0) == 0xC0 && s[1]) { *p += 2; return ((s[0] & 0x1F) << 6) | (s[1] & 0x3F); }
    if ((s[0] & 0xF0) == 0xE0 && s[1] && s[2]) {
        *p += 3;
        return ((s[0] & 0x0F) << 12) | ((s[1] & 0x3F) << 6) | (s[2] & 0x3F);
    }
    *p += 1;
    return 0;
}

static int cjkIndex(uint32_t code)
{
    int lo = 0, hi = CJK_N - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (cjk_code[mid] == code) return mid;
        if (cjk_code[mid] < code) lo = mid + 1; else hi = mid - 1;
    }
    return -1;
}

static void cjkGlyph(int x, int y, int idx, uint16_t color, int scale)
{
    const uint8_t *g = cjk_glyph[idx];
    for (int row = 0; row < 16; row++) {
        const uint16_t bits = ((uint16_t)g[row * 2] << 8) | g[row * 2 + 1];
        int c = 0;
        while (c < 16) {
            if (bits & (0x8000 >> c)) {
                int run = 1;
                while (c + run < 16 && (bits & (0x8000 >> (c + run)))) run++;
                gfx->fillRect(x + c * scale, y + row * scale, run * scale, scale, color);
                c += run;
            } else {
                c++;
            }
        }
    }
}

static int cjkText(int x, int y, const char *s, uint16_t color, int scale)
{
    if (!s) return 0;
    int cx = x;
    const char *p = s;
    while (*p) {
        const char *before = p;
        const int code = utf8Next(&p);
        if (code == 0) break;
        if (code < 0x80) {
            gfx->setTextSize(scale > 1 ? 2 : 1);
            gfx->setTextColor(color);
            gfx->setCursor(cx, y + (scale > 1 ? 1 : 4));
            gfx->write((uint8_t)code);
            cx += 6 * (scale > 1 ? 2 : 1);
        } else {
            const int idx = cjkIndex((uint32_t)code);
            if (idx >= 0) cjkGlyph(cx, y, idx, color, scale);
            cx += 16 * scale;
        }
        if (p == before) break;
    }
    return cx - x;
}

static int cjkWidth(const char *s, int scale)
{
    int w = 0;
    const char *p = s;
    while (*p) {
        const char *before = p;
        const int code = utf8Next(&p);
        if (code == 0) break;
        w += (code < 0x80) ? 6 * (scale > 1 ? 2 : 1) : 16 * scale;
        if (p == before) break;
    }
    return w;
}

static void cjkCenter(int cx, int y, const char *s, uint16_t color, int scale)
{
    cjkText(cx - cjkWidth(s, scale) / 2, y, s, color, scale);
}

/* ================= 通用控件 ================= */
static bool hit(int16_t x, int16_t y, int rx, int ry, int rw, int rh)
{
    return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

/* 按钮外观：绘制用这张表的坐标，命中判定也用同一张表 */
/* 分页按钮的"可用/置灰"由数据边界决定。
   ★ 只影响绘制，不影响命中矩形：几何仍来自同一张 BtnDef 表，
     置灰状态下点它也不会翻页（uiAct 里有同样的边界守卫）。 */
static bool btnDisabled(uint8_t id)
{
    switch (id) {
    case HIT_DEV_PREV:  return s_devPage <= 0;
    case HIT_DEV_NEXT:  return (s_devPage + 1) * LIST_ROWS >= obdCandCount();
    case HIT_WIFI_PREV: return s_apPage <= 0;
    case HIT_WIFI_NEXT: return (s_apPage + 1) * LIST_ROWS >= netScanCount();
    default:            return false;
    }
}

static void drawBtns(const BtnDef *arr, int n)
{
    for (int i = 0; i < n; i++) {
        const BtnDef *b = &arr[i];
        uint16_t bg = C_BG, fg = C_TEXT, bd = C_LINE;
        if (b->style == 1) { bg = C_GREEN_BG; fg = C_OK;   bd = C_OK;   }
        if (b->style == 2) { bg = C_ERR;      fg = C_TEXT; bd = C_ERR;  }
        if (btnDisabled(b->id)) { bg = C_BG;  fg = C_OFF;  bd = C_LINE; }
        gfx->fillRect(b->x, b->y, b->w, b->h, bg);
        gfx->drawRect(b->x, b->y, b->w, b->h, bd);
        cjkCenter(b->x + b->w / 2, b->y + (b->h - 16) / 2, b->label, fg, 1);
    }
}

static uint8_t hitBtns(const BtnDef *arr, int n, int16_t x, int16_t y)
{
    for (int i = 0; i < n; i++) {
        const BtnDef *b = &arr[i];
        if (hit(x, y, b->x, b->y, b->w, b->h)) return b->id;
    }
    return HIT_NONE;
}

static void rowRect(int i, int16_t *x, int16_t *y, int16_t *w, int16_t *h)
{
    *x = 6;
    *y = LIST_Y0 + i * LIST_ROW_GAP;
    *w = 468;
    *h = LIST_ROW_H;
}

const char *uiHitName(uint8_t id)
{
    switch (id) {
    case HIT_MAIN_DEV:     return "主界面/蓝牙设备";
    case HIT_MAIN_WIFI:    return "主界面/无线网络";
    case HIT_MAIN_DIAG:    return "主界面/系统诊断";
    case HIT_DEV_PREV:     return "蓝牙/上一页";
    case HIT_DEV_NEXT:     return "蓝牙/下一页";
    case HIT_DEV_RESCAN:   return "蓝牙/重新扫描";
    case HIT_DEV_CONNECT:  return "蓝牙/连接所选";
    case HIT_DEV_BACK:     return "蓝牙/返回";
    case HIT_WIFI_PREV:    return "网络/上一页";
    case HIT_WIFI_NEXT:    return "网络/下一页";
    case HIT_WIFI_SCAN:    return "网络/扫描";
    case HIT_WIFI_CONNECT: return "网络/连接";
    case HIT_WIFI_BACK:    return "网络/返回";
    case HIT_DIAG_ADV:     return "诊断/高级诊断";
    case HIT_DIAG_BACK:    return "诊断/返回";
    case HIT_ADV_CAL:      return "高级/触摸校准";
    case HIT_ADV_PIDS:     return "PID";
    case HIT_ADV_BACK:     return "高级/返回";
    case HIT_CAL_START:    return "校准/重新开始";
    case HIT_CAL_CANCEL:   return "校准/取消";
    case HIT_DIAG_SAVE:    return "诊断/保存行程";
    default:
        if (id >= HIT_ROW_BASE && id <= HIT_ROW_BASE + 3) return "列表行";
        return "未命中";
    }
}

void uiMarkSet(bool on) { s_mark = on; uiDraw(true); }

/* 90e6 的缺陷：s_markX/s_markY 从未被赋值，永远是 -1，所以 TOUCH MARK 打开也不画十字。
   这里补上赋值的入口，主循环每次点击都调一次。 */
void uiMarkMove(int16_t x, int16_t y) { s_markX = x; s_markY = y; }

void uiToast(const char *msg, uint16_t ms)
{
    snprintf(s_toast, sizeof(s_toast), "%s", msg ? msg : "");
    s_toastUntil = millis() + ms;
}

/* ================= 首次初始化（必须在 SD 之前） ================= */
bool uiBegin()
{
    Wire.begin(21, 22);
    TCA.begin();
    TCA.pinMode1(0, OUTPUT); TCA.pinMode1(1, OUTPUT);
    TCA.write1(0, 1); TCA.write1(1, 1); delay(10);
    TCA.write1(0, 0); TCA.write1(1, 0); delay(10);
    TCA.write1(0, 1); TCA.write1(1, 1); delay(200);

    if (!gfx->begin()) {
        Serial.println(F("{\"ev\":\"lcd-fail\",\"begin\":0}"));
        return false;
    }
    gfx->fillScreen(C_BG);
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);
    s_lcd = true;

    gfx->setTextSize(3);
    gfx->setTextColor(C_WARN);
    gfx->setCursor(24, 88);
    gfx->print("LAOMO GAUGE");
    cjkText(24, 150, "启动中", C_TEXT, 1);

    Serial.println(F("{\"ev\":\"lcd-ok\",\"order\":\"lcd-first\"}"));
    return true;
}

/* ================= 顶部栏 ================= */
static const char *pageTitle(int p)
{
    switch (p) {
    case PAGE_MAIN: return "主界面";
    case PAGE_DEV:  return "蓝牙设备";
    case PAGE_WIFI: return "无线网络";
    case PAGE_DIAG: return "系统诊断";
    default:        return "高级诊断";
    }
}

static void drawTopBar()
{
    const bool sim = (g_mode == MODE_SIM);
    gfx->fillRect(0, 0, SCR_W, TOP_H, sim ? C_ERR : C_CARD);
    gfx->drawFastHLine(0, TOP_H - 1, SCR_W, C_LINE);

    cjkText(10, 11, pageTitle(g_page), sim ? C_TEXT : C_WARN, 1);

    /* 页码：右对齐到模式徽标左侧，宽度按实际字符算，不会越界 */
    char b[12];
    snprintf(b, sizeof(b), "%d/%d", (int)g_page + 1, (int)PAGE_N);
    const int pw = 6 * (int)strlen(b);
    gfx->setTextSize(1);
    gfx->setTextColor(C_DIM);
    gfx->setCursor(412 - pw, 15);
    gfx->print(b);

    /* 无线网络页：把连接状态放在标题右侧空白区，列表就能从同一行高开始 */
    if (g_page == PAGE_WIFI) {
        const bool up = (strcmp(netStateStr(), "connected") == 0);
        char st[40];
        if (up) snprintf(st, sizeof(st), "已连接 %s", netIpStr());
        else    snprintf(st, sizeof(st), "%s", netHasCred() ? "未连接" : "未配置");
        cjkText(88, 11, st, up ? C_OK : C_DIM, 1);
    }

    /* 模式徽标：中文两字，52x26，右边界 470 < 480 */
    const char *m = sim ? "模拟" : "真实";
    gfx->fillRect(418, 6, 52, 26, sim ? C_ERR : C_GREEN_BG);
    gfx->drawRect(418, 6, 52, 26, sim ? C_ERR : C_OK);
    cjkCenter(418 + 26, 11, m, sim ? C_TEXT : C_OK, 1);
}

/* ================= 主界面 ================= */
static const char *pidCn(const char *cmd)
{
    if (!strcmp(cmd, "010C")) return "发动机转速";
    if (!strcmp(cmd, "010D")) return "车速";
    if (!strcmp(cmd, "0105")) return "发动机水温";
    if (!strcmp(cmd, "0142")) return "车辆电压";
    if (!strcmp(cmd, "0104")) return "发动机负荷";
    if (!strcmp(cmd, "0100")) return "支持位图";
    return "";
}

/* 单位：水温用 °C，用点阵渲染（° 在字模里），其余 ASCII */
static void drawUnit(int xRight, int y, const char *unit)
{
    if (!unit || !unit[0]) return;
    if ((uint8_t)unit[0] < 0x80) {
        gfx->setTextSize(1);
        gfx->setTextColor(C_DIM);
        gfx->setCursor(xRight - 6 * (int)strlen(unit), y);
        gfx->print(unit);
    } else {
        cjkText(xRight - cjkWidth(unit, 1), y, unit, C_DIM, 1);
    }
}

static void drawCell(int x, int y, int w, int h, const char *label, const char *unit,
                     float val, bool has, bool stale, int dec)
{
    gfx->fillRect(x, y, w, h, C_CARD);
    gfx->drawRect(x, y, w, h, C_LINE);

    /* 中文名称（scale 1，16px 高） */
    cjkText(x + 8, y + 5, label, C_DIM, 1);

    /* 数值 */
    char buf[16];
    if (!has || stale) snprintf(buf, sizeof(buf), "--");
    else if (dec > 0)  snprintf(buf, sizeof(buf), "%.1f", val);
    else               snprintf(buf, sizeof(buf), "%.0f", val);

    gfx->setTextSize(3);
    gfx->setTextColor((!has || stale) ? C_OFF : C_OK);
    gfx->setCursor(x + 8, y + 24);
    gfx->print(buf);

    /* 右下角：正常显示单位，超时显示「数据暂不可用」 */
    if (stale && has) {
        cjkText(x + w - 6 - cjkWidth("数据暂不可用", 1), y + h - 20, "数据暂不可用", C_WARN, 1);
    } else {
        drawUnit(x + w - 8, y + h - 20, unit);
    }
}

static void statusRow(int y, const char *label, const char *value, uint16_t color)
{
    gfx->fillCircle(14, y + 8, 4, color);
    cjkText(26, y, label, C_TEXT, 1);
    cjkText(130, y, value, color, 1);
}

/* ---- 主页拆成 7 个可独立重画的部件（局部刷新的基础） ---- */
static void drawMainCell(int i)
{
    bool has = false, isSim = false;
    const float v = obdVal(i, &has, &isSim);
    const PidRec *p = obdPid(i);
    const bool stale = (obdStaleMs() > STALE_MS);
    const int x = 8 + (i % 2) * 236;
    const int y = 42 + (i / 2) * 60;
    /* 水温的 PID 表单位是 "C"，界面上要显示 °C（° 需要字模，故在此显式给出字面量） */
    const char *unitDisp = strcmp(p->cmd, "0105") == 0 ? "°C" : p->unit;
    drawCell(x, y, 228, 56, pidCn(p->cmd), unitDisp, v, has, stale && !isSim,
             strcmp(p->cmd, "0142") == 0 ? 1 : 0);
}

static void drawMainVeh()
{
    /* 车辆 */
    const char *t; uint16_t c;
    if (g_mode == MODE_SIM)                                        { t = "模拟数据 非车辆"; c = C_WARN; }
    else if (g_link == LINK_CONNECTED && g_ecu == ECU_VALID)        { t = "已连接 收到车辆应答"; c = C_OK; }
    else if (g_link == LINK_CONNECTED)                              { t = "已连接 无车辆应答"; c = C_WARN; }
    else if (g_link == LINK_SCANNING || g_link == LINK_CONNECTING)  { t = "正在连接"; c = C_WARN; }
    else if (g_link == LINK_FAILED)                                 { t = "连接失败"; c = C_ERR; }
    else                                                            { t = "未连接"; c = C_OFF; }
    statusRow(166, "车辆", t, c);
}

static void drawMainGps()
{
    /* 定位 */
    char r[48];
    const int q = gpsFixQ();
    if (q >= 1) {
        snprintf(r, sizeof(r), "已定位 卫星%d", gpsSats() > 0 ? gpsSats() : 0);
        statusRow(188, "定位", r, C_OK);
    } else {
        statusRow(188, "定位", q < 0 ? "无报文" : "未定位", C_OFF);
    }
}

static void drawMainStore()
{
    /* 存储：把行数含义写清楚 */
    char r[48];
    const char *t2; uint16_t c2;
    switch (g_uisd) {
    case SD_REC:     t2 = "记录中"; c2 = C_OK;   break;
    case SD_SAVED:   t2 = "已保存"; c2 = C_OK;   break;
    case SD_VERIFY:  t2 = "核验中"; c2 = C_WARN; break;
    case SD_ERROR:   t2 = "存储错误"; c2 = C_ERR; break;
    case SD_MOUNTED: t2 = "已挂载未记录"; c2 = C_WARN; break;
    case SD_RECOVER: t2 = "断电恢复"; c2 = C_WARN; break;
    default:         t2 = "无卡"; c2 = C_ERR;   break;
    }
    gfx->fillCircle(14, 218, 4, c2);
    cjkText(26, 210, "存储", C_TEXT, 1);
    cjkText(130, 210, t2, c2, 1);
    /* 已写入行数：明确标出"行" */
    snprintf(r, sizeof(r), "%lu", (unsigned long)storeLines());
    /* 「已写」两字 = 32px，数字从 334 起，避免压到字上 */
    cjkText(296, 214, "已写", C_DIM, 1);
    gfx->setTextSize(2);
    gfx->setTextColor(C_DIM);
    gfx->setCursor(334, 211); gfx->print(r);
    cjkText(334 + 12 * (int)strlen(r) + 6, 214, "行", C_DIM, 1);
}

static void drawMainNet()
{
    /* 网络 */
    const bool up = (strcmp(netStateStr(), "connected") == 0);
    if (up) {
        gfx->fillCircle(14, 240, 4, C_OK);
        cjkText(26, 232, "网络", C_TEXT, 1);
        cjkText(130, 232, "已连接", C_OK, 1);
        gfx->setTextSize(1); gfx->setTextColor(C_DIM);
        gfx->setCursor(210, 236); gfx->print(netIpStr());
    } else {
        statusRow(232, "网络", netHasCred() ? "已配置 未连接" : "未配置", C_OFF);
    }
}

static void drawMain()
{
    for (int i = 0; i < 4; i++) drawMainCell(i);
    drawMainVeh();
    drawMainGps();
    drawMainStore();
    drawMainNet();
}

/* ================= 蓝牙设备页 ================= */
static void trimTail(char *s)
{
    int n = (int)strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = 0;
}

static void drawDevRow(int i, int idx)
{
    int16_t x, y, w, h;
    rowRect(i, &x, &y, &w, &h);
    const BleCand *c = obdCand(idx);
    const bool sel = (idx == s_devCur);

    gfx->fillRect(x, y, w, h, sel ? C_SEL : C_CARD);
    gfx->drawRect(x, y, w, h, sel ? C_WARN : C_LINE);

    char nm[24];
    nm[0] = 0;
    if (c->name[0]) { snprintf(nm, sizeof(nm), "%.11s", c->name); trimTail(nm); }

    if (nm[0]) {
        gfx->setTextSize(2);
        gfx->setTextColor(C_TEXT);
        gfx->setCursor(x + 10, y + 3);
        gfx->print(nm);
    } else {
        /* 设备没广播名称：中文必须走点阵，gfx->print 只有 ASCII 字库 */
        cjkText(x + 10, y + 3, "未命名设备", C_TEXT, 2);
    }

    /* 第二行：地址尾号 + 信号强度 + FFE0 */
    const int L = (int)strlen(c->addr);
    const char *tail = (L > 8) ? (c->addr + L - 8) : c->addr;

    gfx->setTextSize(1);
    gfx->setTextColor(C_DIM);
    gfx->setCursor(x + 10, y + 22);
    gfx->print("..");
    gfx->print(tail);

    char rr[16];
    snprintf(rr, sizeof(rr), "%d dBm", c->rssi);
    gfx->setCursor(x + 170, y + 22);
    gfx->print(rr);

    gfx->setTextColor(c->hasFfe0 ? C_OK : C_OFF);
    gfx->setCursor(x + 268, y + 22);
    gfx->print(c->hasFfe0 ? "FFE0" : "----");

    if (sel) {
        gfx->setTextColor(C_WARN);
        gfx->setCursor(x + w - 22, y + 10);
        gfx->print(">");
    }
}

static void drawPageIndicator(int page, int perPage, int total, int y)
{
    char b[40];      /* 三个 %d 最坏各 11 字符，缓冲区不能太小，否则编译器报截断风险 */
    const int start = page * perPage + 1;
    int end = (page + 1) * perPage;
    if (end > total) end = total;
    if (total <= 0) snprintf(b, sizeof(b), "0 / 0");
    else            snprintf(b, sizeof(b), "%d-%d / %d", start, end, total);
    gfx->setTextSize(2);
    gfx->setTextColor(C_TEXT);
    /* 居中到屏幕中线：两个分页按钮分别在 6..110 与 370..474，中间 192..288 是安全区 */
    const int w = 12 * (int)strlen(b);        /* size 2 每字符 12px */
    gfx->setCursor(SCR_W / 2 - w / 2, y);
    gfx->print(b);
}

/* ---- 分页按钮表：必须在 drawDevFooter/drawWifiFooter 之前定义 ----
/* 分页行：上一页在左、下一页在右，页码居中在两者之间（x≈192..288），互不重叠 */
static const BtnDef kDevPageBtns[2] = {
    { 6,   PGT_Y, 104, PGT_H, HIT_DEV_PREV, 0, "上一页" },
    { 370, PGT_Y, 104, PGT_H, HIT_DEV_NEXT, 0, "下一页" },
};
static const BtnDef kWifiPageBtns[2] = {
    { 6,   PGT_Y, 104, PGT_H, HIT_WIFI_PREV, 0, "上一页" },
    { 370, PGT_Y, 104, PGT_H, HIT_WIFI_NEXT, 0, "下一页" },
};

/* 分页行 + 底部提示：整页绘制与局部重画都只调这一个函数，
   避免"整页有按钮、局部刷新后按钮消失"这类两处实现走偏的问题。 */
static void drawDevFooter()
{
    const int n = obdCandCount();

    /* 分页按钮：绘制与命中同用 kDevPageBtns，几何天然一致 */
    drawBtns(kDevPageBtns, 2);
    drawPageIndicator(s_devPage, LIST_ROWS, n, PGT_Y + 8);

    /* 底部提示：当前选中与连接状态 */
    /* 分页行占到 238，提示两行整体上移 6px：已选 240..255，link 258..265，留 3px 行距 */
    const int hy = HINT_Y - 6;
    if (n > 0 && s_devCur < n) {
        const BleCand *c = obdCand(s_devCur);
        const int hx = cjkText(12, hy, "已选: ", C_DIM, 1);
        if (c->name[0]) {
            gfx->setTextSize(1);
            gfx->setTextColor(C_DIM);
            gfx->setCursor(hx + 4, hy + 4);
            gfx->print(c->name);
        } else {
            cjkText(hx + 4, hy, "未命名设备", C_DIM, 1);
        }
    } else {
        cjkText(12, hy, "请先扫描并选择设备", C_DIM, 1);
    }
    gfx->setTextSize(1);
    gfx->setTextColor(C_DIM);
    gfx->setCursor(12, hy + 18);   /* 已选那行 16px 高，+14 会压 2px，推到 +16 */
    gfx->print("link=");  gfx->print(obdLinkStr());
    gfx->print("  svc="); gfx->print(obdSvcStr());
    gfx->print("  ecu="); gfx->print(obdEcuStr());
}

static void drawDev()
{
    const int n = obdCandCount();

    if (n <= 0) {
        cjkText(16, 56,  "未发现设备", C_WARN, 2);
        cjkText(16, 100, "点下方 重新扫描 开始搜索", C_DIM, 1);
        cjkText(16, 126, "手机请先断开盒子的连接", C_DIM, 1);
        cjkText(16, 152, "同一盒子只能被一台设备连接", C_DIM, 1);
    } else {
        const int start = s_devPage * LIST_ROWS;
        for (int i = 0; i < LIST_ROWS; i++) {
            const int idx = start + i;
            if (idx >= n) break;
            drawDevRow(i, idx);
        }
    }
    drawDevFooter();
}

/* ================= 无线网络页 ================= */
static void drawWifiRow(int i, int idx)
{
    int16_t x, y, w, h;
    rowRect(i, &x, &y, &w, &h);
    const bool sel = (idx == s_apCur);

    gfx->fillRect(x, y, w, h, sel ? C_SEL : C_CARD);
    gfx->drawRect(x, y, w, h, sel ? C_WARN : C_LINE);

    char ssid[34];
    netApGet(idx, ssid, sizeof(ssid), nullptr, nullptr);

    gfx->setTextSize(2);
    gfx->setTextColor(C_TEXT);
    gfx->setCursor(x + 10, y + 3);
    gfx->print(ssid);

    int rssi = 0; unsigned ch = 0;
    netApGet(idx, nullptr, 0, &rssi, &ch);
    char rr[24];
    snprintf(rr, sizeof(rr), "%d dBm  信道%u", rssi, ch);
    cjkText(x + 10, y + 22, rr, C_DIM, 1);

    /* 已保存过密码的网络打个标记 */
    if (netHasCred() && !strcmp(ssid, netSsid())) {
        cjkText(x + w - 60, y + 22, "已配置", C_OK, 1);
    }
}

/* 分页行 + 底部提示（与 BLE 页同一套做法） */
static void drawWifiFooter()
{
    const int n = netScanCount();

    drawBtns(kWifiPageBtns, 2);                       /* 绘制与命中同用 kWifiPageBtns */
    drawPageIndicator(s_apPage, LIST_ROWS, n, PGT_Y + 8);

    /* 关键：明确告诉用户密码怎么给，而不是只给个列表 */
    cjkText(12, HINT_Y - 8,  "密码暂无屏幕键盘，请用串口设置:", C_WARN, 1);
    cjkText(12, HINT_Y + 10, "WIFI SET <网络名> <密码>", C_TEXT, 1);
}

static void drawWifi()
{
    const int n = netScanCount();

    if (n <= 0) {
        cjkText(16, 60, "按下方 扫描网络 搜索附近网络", C_DIM, 1);
        cjkText(16, 90, "只支持 2.4GHz", C_DIM, 1);
    } else {
        const int start = s_apPage * LIST_ROWS;
        for (int i = 0; i < LIST_ROWS; i++) {
            const int idx = start + i;
            if (idx >= n) break;
            drawWifiRow(i, idx);
        }
    }
    drawWifiFooter();
}

/* ================= 系统诊断（中文） ================= */
/* 诊断页：8 行状态，可单行重画 */
static void drawDiagRow(int i)
{
    const int y = DIAG_Y0 + DIAG_ROW_H * i;
    uint16_t c;
    switch (i) {
    case 0: {
        const bool sim = (g_mode == MODE_SIM);
        statusRow(y, "采集模式", sim ? "模拟测试" : "真实采集", sim ? C_ERR : C_OK);
        break;
    }
    case 1: {
        const char *bt;
        switch (g_link) {
        case LINK_CONNECTED:  bt = "已连接"; c = C_OK;   break;
        case LINK_CONNECTING: bt = "连接中"; c = C_WARN; break;
        case LINK_SCANNING:   bt = "扫描中"; c = C_WARN; break;
        case LINK_FAILED:     bt = "连接失败"; c = C_ERR; break;
        default:              bt = "未连接"; c = C_OFF;  break;
        }
        statusRow(y, "蓝牙连接", bt, c);
        break;
    }
    case 2: {
        const char *sv;
        switch (g_svc) {
        case SVC_FOUND:   sv = "已识别"; c = C_OK;  break;
        case SVC_MISSING: sv = "未识别"; c = C_ERR; break;
        default:          sv = "未识别"; c = C_OFF; break;
        }
        statusRow(y, "OBD 服务", sv, c);
        break;
    }
    case 3: {
        const char *ec;
        switch (g_ecu) {
        case ECU_VALID:  ec = "正常收到"; c = C_OK;  break;
        case ECU_NODATA: ec = "无有效应答"; c = C_WARN; break;
        default:         ec = "尚未收到"; c = C_OFF; break;
        }
        statusRow(y, "ECU 数据", ec, c);
        break;
    }
    case 4:
        statusRow(y, "设备绑定", obdSavedBound() ? "已绑定" : "未绑定",
                  obdSavedBound() ? C_OK : C_OFF);
        break;
    case 5:
        statusRow(y, "触摸屏", touchReady() ? "正常" : "异常", touchReady() ? C_OK : C_ERR);
        break;
    case 6: {
        const int q = gpsFixQ();
        if (q >= 1)      statusRow(y, "卫星定位", "已定位", C_OK);
        else if (q < 0)  statusRow(y, "卫星定位", "无报文", C_OFF);
        else             statusRow(y, "卫星定位", "未定位", C_OFF);
        break;
    }
    default: {
        const char *sd;
        switch (g_uisd) {
        case SD_REC:     sd = "记录中"; c = C_OK;   break;
        case SD_SAVED:   sd = "已保存"; c = C_OK;   break;
        case SD_VERIFY:  sd = "核验中"; c = C_WARN; break;
        case SD_ERROR:   sd = "异常";   c = C_ERR;  break;
        case SD_MOUNTED: sd = "已挂载"; c = C_WARN; break;
        case SD_RECOVER: sd = "断电恢复"; c = C_WARN; break;
        default:         sd = "无卡";   c = C_ERR;  break;
        }
        statusRow(y, "SD 卡", sd, c);
        break;
    }
    }
}

/* 阶段结果的显示颜色：成功绿 / 失败红 / 已发送已收到黄 / 其它白 */
static uint16_t resColor(const char *code)
{
    if (!strcmp(code, "ok"))  return C_OK;
    if (!strcmp(code, "timeout") || !strcmp(code, "unparse") ||
        !strcmp(code, "search")  || !strcmp(code, "nodata") ||
        !strcmp(code, "sendfail")) return C_ERR;
    if (!strcmp(code, "sent") || !strcmp(code, "recv")) return C_WARN;
    return C_TEXT;
}

/* ================= 诊断页「阶段区」 =================
   4 个独立小区域，只在这些内容变化时重画（uiPhasePaint），不整屏重画。
   内容与源码事件的对应关系见交付报告第三节，绝不出现「已发送」显示成「成功」。 */
static const int16_t kDiagPhRect[4][4] = {
    { 0, DIAG_Y_PH,   SCR_W, 16 },
    { 0, DIAG_Y_NTF,  SCR_W, 16 },
    { 0, DIAG_Y_FAIL, SCR_W, 16 },
    { 0, DIAG_Y_PID,  SCR_W, 16 },
};

static void drawDiagPhase()
{
    for (int i = 0; i < 4; i++) {
        gfx->fillRect(kDiagPhRect[i][0], kDiagPhRect[i][1],
                      kDiagPhRect[i][2], kDiagPhRect[i][3], C_BG);
    }

    char s[80];
    const uint32_t pm = obdPhaseMs();
    const char *rc = obdPhaseResultCode();
    const unsigned sec = (unsigned)(pm / 1000);
    const unsigned d10 = (unsigned)((pm % 1000) / 100);

    /* ---- 1) 当前阶段 / 结果 / 阶段耗时 ---- */
    const int rl = obdRxLen();
    char extra[20] = "";
    /* 「已找到」这一步顺带显示候选数量，让「找到并选中」这件事在诊断页也说得清 */
    if (!strcmp(obdPhaseCode(), "found")) snprintf(extra, sizeof(extra), " 共%d台", obdCandCount());
    if (!strcmp(rc, "recv") && rl > 0) {
        snprintf(s, sizeof(s), "阶段 %u/%u %s%s %s %dB %u.%us",
                 (unsigned)obdPhaseNo(), (unsigned)obdPhaseCount(),
                 obdPhaseName(), extra, obdPhaseResultStr(), rl, sec, d10);
    } else {
        snprintf(s, sizeof(s), "阶段 %u/%u %s%s %s %u.%us",
                 (unsigned)obdPhaseNo(), (unsigned)obdPhaseCount(),
                 obdPhaseName(), extra, obdPhaseResultStr(), sec, d10);
    }
    cjkText(10, DIAG_Y_PH, s, resColor(rc), 1);

    /* ---- 2) 通知注册 / 回调观察（两件事分开说，CCCD 一律标未知） ---- */
    {
        const uint32_t nfy = obdNotifyCount();
        if (nfy > 0) {
            snprintf(s, sizeof(s), "通知 注册%s 回调%lu次 CCCD未知",
                     obdNotifyFlag() ? "已调用" : "未调用", (unsigned long)nfy);
        } else {
            snprintf(s, sizeof(s), "通知 注册%s 回调未观察 CCCD未知",
                     obdNotifyFlag() ? "已调用" : "未调用");
        }
        /* 回调一次都没观察到 → 用暗色，避免被读成「订阅失败」（这是 UNKNOWN） */
        cjkText(10, DIAG_Y_NTF, s, (nfy > 0) ? C_OK : C_DIM, 1);
    }

    /* ---- 3) 最近一次失败（不会被笼统的"未连接"覆盖） ---- */
    if (obdLastFailPhase()) {
        snprintf(s, sizeof(s), "最近失败 %u/%u %s",
                 (unsigned)obdLastFailPhase(), (unsigned)obdPhaseCount(), obdLastFailNote());
        cjkText(10, DIAG_Y_FAIL, s, C_ERR, 1);
    } else {
        cjkText(10, DIAG_Y_FAIL, "最近失败 无", C_DIM, 1);
    }

    /* ---- 4) 最近一次 PID 轮询：有请求在飞就显示「等待中」，否则显示最近结果 ---- */
    {
        char pid[26];
        if (obdPidBusy()) {
            /* ★「轮询正在进行」用这一行表达：它不是阶段（阶段是连接/握手进度），
               但必须看得见，所以和最近结果共用一行、用「等待中」区分。 */
            snprintf(s, sizeof(s), "轮询 %s 等待中", obdSentCmd());
            cjkText(10, DIAG_Y_PID, s, C_WARN, 1);
        } else {
            obdLastPidStr(pid, sizeof(pid));
            snprintf(s, sizeof(s), "轮询 %s", pid);
            cjkText(10, DIAG_Y_PID, s, strcmp(pid, "无") ? C_TEXT : C_DIM, 1);
        }
    }
}

static void drawDiag()
{
    drawDiagPhase();
    for (int i = 0; i < 8; i++) drawDiagRow(i);
}

/* ================= 高级诊断（PID 表 + 技术字段 + 校准入口） ================= */
/* 高级诊断：6 行 PID + 4 行技术字段，均可单行重画 */
static void drawAdvPidRow(int i)
{
    const int index = s_advPidPage * kAdvPidRows + i;
    if (index >= obdPidCount()) return;
    const int y = 64 + 17 * i;

    const PidRec *p = obdPid(index);
    char line[48];
    const char *name = pidCn(p->cmd);
    snprintf(line, sizeof(line), "%s %s", p->cmd, *name ? name : p->label);
    cjkText(12, y, line, C_TEXT, 1);

    bool fresh = false, sim = false;
    const float reading = obdVal(index, &fresh, &sim);
    const bool mask = p->nbytes == 4;
    char value[32];
    if (fresh && !mask) snprintf(value, sizeof(value), "%.2f %s", reading, p->unit);
    else if (fresh) snprintf(value, sizeof(value), "RX");
    else snprintf(value, sizeof(value), "%s", p->supported == 2 ? "unsupported" : "--");
    cjkText(160, y, value, fresh ? C_OK : C_OFF, 1);

    const char *ecu = sim ? "SIM" : p->hasVal && p->lastEcu == 0x7E8 ? "7E8" :
                      p->hasVal && p->lastEcu == 0x7E9 ? "7E9" : "--";
    char result[32];
    snprintf(result, sizeof(result), "%s %s", ecu, obdPidResultStr(*p));
    cjkText(249, y, result, C_DIM, 1);

    char st[28];
    if (!sim && p->hasVal) {
        snprintf(st, sizeof(st), "%lu.%lus", (unsigned long)((millis() - p->lastOkMs) / 1000),
                 (unsigned long)(((millis() - p->lastOkMs) / 100) % 10));
        gfx->setTextSize(1);
        gfx->setTextColor(fresh ? C_OK : C_OFF);
        gfx->setCursor(330, y + 4);
        gfx->print(st);
    } else {
        cjkText(330, y, "未收到", C_OFF, 1);
    }

    char cnt[24];
    snprintf(cnt, sizeof(cnt), "%lu/%lu", (unsigned long)p->ok, (unsigned long)p->req);
    gfx->setTextColor(C_DIM);
    gfx->setCursor(410, y + 4);
    gfx->print(cnt);
}

static void drawAdvTech(int k)
{
    const int y = 186 + 11 * k;
    char l[64];
    gfx->setTextSize(1);
    gfx->setTextColor(C_DIM);
    switch (k) {
    case 0:
        snprintf(l, sizeof(l), "link=%s svc=%s ecu=%s", obdLinkStr(), obdSvcStr(), obdEcuStr());
        break;
    case 1:
        snprintf(l, sizeof(l), "req=%lu ok=%lu nodata=%lu to=%lu", (unsigned long)g_realReq,
                 (unsigned long)g_realOk, (unsigned long)g_realNoData, (unsigned long)g_realTo);
        break;
    case 2:
        snprintf(l, sizeof(l), "cand=%d bound=%d sess=%lu up=%lus", obdCandCount(),
                 obdSavedBound() ? 1 : 0, (unsigned long)g_sess,
                 (unsigned long)((millis() - g_bootMs) / 1000));
        break;
    default:
        snprintf(l, sizeof(l), "reset=%s touch=%s cal=%s taps=%lu", g_resetWhy,
                 touchReady() ? "OK" : "NO", touchCalValid() ? "Y" : "N",
                 (unsigned long)touchTaps());
        break;
    }
    gfx->setCursor(12, y); gfx->print(l);
}

/* ================= 高级诊断页「阶段区」 =================
   三行 ASCII，字号 1：① 阶段/结果 ② 0100 与通知 ③ 三条 AT 结果与失败阶段。
   ★ 只刷新这三行，不整屏重画。 */
static const int16_t kAdvPhRect[3][4] = {
    { 0, ADV_Y_PH,  SCR_W, 11 },
    { 0, ADV_Y_PRB, SCR_W, 11 },
    { 0, ADV_Y_AT,  SCR_W, 10 },
};

static void drawAdvPhase()
{
    for (int i = 0; i < 3; i++) {
        gfx->fillRect(kAdvPhRect[i][0], kAdvPhRect[i][1],
                      kAdvPhRect[i][2], kAdvPhRect[i][3], C_BG);
    }
    char d[80];
    gfx->setTextSize(1);

    const uint32_t pm = obdPhaseMs();
    snprintf(d, sizeof(d), "ph=%u/%u %s st=%s %lu.%lus",
             (unsigned)obdPhaseNo(), (unsigned)obdPhaseCount(),
             obdPhaseCode(), obdPhaseResultCode(),
             (unsigned long)(pm / 1000), (unsigned long)((pm % 1000) / 100));
    gfx->setTextColor(resColor(obdPhaseResultCode()));
    gfx->setCursor(12, ADV_Y_PH);   gfx->print(d);

    snprintf(d, sizeof(d), "p0100=%s nfy=%lu reg=%d fb=%lu",
             obdProbeStr(), (unsigned long)obdNotifyCount(),
             obdNotifyFlag() ? 1 : 0, (unsigned long)obdFirstByteMs());
    /* 回调没观察到 → 暗色：只表示没看见，不表示订阅失败 */
    gfx->setTextColor(obdNotifyCount() ? C_OK : C_DIM);
    gfx->setCursor(12, ADV_Y_PRB);  gfx->print(d);

    snprintf(d, sizeof(d), "%s=%s %s=%s %s=%s fail=%u/%u f1=%u",
             obdAtCmdName(0), obdAtResultStr(0),
             obdAtCmdName(1), obdAtResultStr(1),
             obdAtCmdName(2), obdAtResultStr(2),
             (unsigned)obdLastFailPhase(), (unsigned)obdPhaseCount(),
             (unsigned)obdFirstFailPhase());
    gfx->setTextColor(obdLastFailPhase() ? C_ERR : C_DIM);
    gfx->setCursor(12, ADV_Y_AT);   gfx->print(d);
}

static void drawAdv()
{
    cjkText(12, 44, "主要数据项", C_WARN, 1);
    char page[24];
    snprintf(page, sizeof(page), "%d/%d", s_advPidPage + 1,
             (obdPidCount() + kAdvPidRows - 1) / kAdvPidRows);
    cjkText(420, 44, page, C_DIM, 1);
    for (int i = 0; i < kAdvPidRows; i++) drawAdvPidRow(i);
    cjkText(12, 168, "技术字段", C_WARN, 1);
    for (int k = 0; k < 4; k++) drawAdvTech(k);
    drawAdvPhase();
}

/* ================= 触摸校准（高级诊断的二级功能） ================= */
/* 角标画在**真实角落**上：校准存的就是这两点的原始坐标，
   若把方框画在屏幕中间，"点方框中心"会被当成点(0,0)，反而引入偏差。 */
static void drawCornerMark(bool right, bool bottom)
{
    const int m = 44;
    const int x = right  ? (SCR_W - m) : 0;
    const int y = bottom ? (BOT_Y - m) : TOP_H;
    gfx->drawRect(x, y, m, m, C_WARN);
    gfx->drawFastHLine(x, y + m / 2, m, C_WARN);
    gfx->drawFastVLine(x + m / 2, y, m, C_WARN);
}

static void drawCal()
{
    cjkText(16, 48, "触摸校准", C_WARN, 2);

    if (s_calStep == 1) {
        cjkText(16, 88,  "请点屏幕左上角角标", C_TEXT, 1);
        cjkText(16, 112, "实体按键那一侧为上", C_DIM, 1);
        cjkText(16, 134, "点在角标十字附近即可", C_DIM, 1);
        drawCornerMark(false, false);
    } else {
        cjkText(16, 88,  "请点屏幕右下角角标", C_TEXT, 1);
        cjkText(16, 112, "这次点对角", C_DIM, 1);
        cjkText(16, 134, "点在角标十字附近即可", C_DIM, 1);
        drawCornerMark(true, true);
    }
    cjkText(16, 236, "共两次点击，结果自动保存", C_DIM, 1);
}

/* ================= 按钮表（绘制与命中共用） ================= */
static const BtnDef kMainBot[3] = {
    { COL1_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_MAIN_DEV,  0, "蓝牙设备" },
    { COL2_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_MAIN_WIFI, 0, "无线网络" },
    { COL3_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_MAIN_DIAG, 0, "系统诊断" },
};

static const BtnDef kDevBot[3] = {
    { COL1_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_DEV_RESCAN,  0, "重新扫描" },
    { COL2_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_DEV_CONNECT, 1, "连接所选" },
    { COL3_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_DEV_BACK,    0, "返回" },
};

static const BtnDef kWifiBot[3] = {
    { COL1_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_WIFI_SCAN,    0, "扫描网络" },
    { COL2_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_WIFI_CONNECT, 1, "连接" },
    { COL3_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_WIFI_BACK,    0, "返回" },
};

static const BtnDef kDiagBot[3] = {
    { COL1_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_DIAG_ADV,  1, "高级诊断" },
    { COL2_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_DIAG_SAVE, 1, "保存行程" },
    { COL3_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_DIAG_BACK, 0, "返回" },
};

static const BtnDef kAdvBot[3] = {
    { COL1_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_ADV_CAL,  0, "触摸校准" },
    { COL2_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_ADV_PIDS, 0, "PID" },
    { COL3_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_ADV_BACK, 0, "返回" },
};

static const BtnDef kCalBot[2] = {
    { COL1_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_CAL_START,  1, "重新开始" },
    { COL3_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H, HIT_CAL_CANCEL, 0, "取消" },
};

static void drawBottomBar()
{
    gfx->fillRect(0, BOT_Y, SCR_W, SCR_H - BOT_Y, C_CARD);
    gfx->drawFastHLine(0, BOT_Y, SCR_W, C_LINE);

    if (s_calStep > 0) { drawBtns(kCalBot, 2); return; }

    switch (g_page) {
    case PAGE_MAIN: drawBtns(kMainBot, 3); break;
    case PAGE_DEV:  drawBtns(kDevBot, 3);  break;
    case PAGE_WIFI: drawBtns(kWifiBot, 3); break;
    case PAGE_DIAG: drawBtns(kDiagBot, 3); break;
    default:        drawBtns(kAdvBot, 3);  break;
    }
}

/* ================= 绘制入口 ================= */
/* ------------------------------------------------------------
   两层绘制：
     ① uiDraw(true)  → 整屏清除 + 整页重画。只在**结构变化**时发生
                       （开机、切页、模式变、列表翻页、扫描结果变、提示、校准）。
     ② uiTick()      → 只重画"数据变了的那一小块"（band）。
   旧实现把 storeLines() / g_realOk / obdStaleMs()/1000 放进"是否重画"的签名里，
   而 GGA 每秒落盘一行、OBD 每次应答都在动 → 签名每秒都变 → 每秒一次 fillScreen，
   这就是"整屏每秒闪一下"的真正原因。这三个量现在只影响各自的 band。
   ------------------------------------------------------------ */
static uint32_t s_nFull = 0;   /* fillScreen + 整页重画次数 */
static uint32_t s_nSig  = 0;   /* 结构签名变化触发的整页重画次数 */
static uint32_t s_nBand = 0;   /* 局部区域重画次数 */
static uint32_t s_nPage = 0;   /* 页面切换次数 */

#define BAND_MAX 12
static char    s_bandSig[BAND_MAX][48];
static uint8_t s_shownPage = 0xFF;
static char    s_structSig[96];

static uint8_t pageBands(uint8_t pg)
{
    switch (pg) {
    case PAGE_MAIN: return 7;
    case PAGE_DEV:  return 5;
    case PAGE_WIFI: return 5;
    case PAGE_DIAG: return 8;
    default:        return 10;      /* PAGE_ADV：6 行 PID + 4 行技术字段 */
    }
}

/* 结构签名：只有这些变化才允许整屏重画。★ 不含每秒都在变的量 */
static void structSig(char *o, size_t n)
{
    snprintf(o, n, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d",
             (int)g_page, (int)g_mode, (int)g_link, (int)g_svc, (int)g_ecu,
             (int)g_uisd, (int)s_calStep, obdCandCount(),
             netScanCount(), s_devPage, s_devCur, s_apPage, s_apCur,
             ((s_toast[0] && millis() < s_toastUntil) ? 1 : 0) + (s_mark ? 2 : 0));
}

/* 每个 band 的"内容指纹"，只放它自己显示的东西 */
static void bandSig(uint8_t i, char *o, size_t n)
{
    switch (g_page) {
    case PAGE_MAIN:
        if (i < 4) {
            bool has = false, isSim = false;
            const float v = obdVal(i, &has, &isSim);
            const bool stale = (obdStaleMs() > STALE_MS);
            snprintf(o, n, "c|%.1f|%d|%d", (double)v, has ? 1 : 0, (stale && !isSim) ? 1 : 0);
        } else if (i == 4) {
            snprintf(o, n, "v|%d|%d|%d", (int)g_mode, (int)g_link, (int)g_ecu);
        } else if (i == 5) {
            snprintf(o, n, "g|%d|%d", gpsFixQ(), gpsSats());
        } else {
            snprintf(o, n, "s|%d|%lu|%s|%s", (int)g_uisd, (unsigned long)storeLines(),
                     netStateStr(), netIpStr());
        }
        break;

    case PAGE_DEV: {
        const int nCand = obdCandCount();
        if (i < LIST_ROWS) {
            const int idx = s_devPage * LIST_ROWS + i;
            const BleCand *c = (idx < nCand) ? obdCand(idx) : nullptr;
            if (!c) snprintf(o, n, "r|e|%d|%d", idx, nCand);
            else    snprintf(o, n, "r|%s|%d|%d|%d", c->addr, c->rssi,
                             c->hasFfe0 ? 1 : 0, (idx == s_devCur) ? 1 : 0);
        } else {
            snprintf(o, n, "f|%d|%d|%d|%d|%d|%d", nCand, s_devPage, s_devCur,
                     (int)g_link, (int)g_svc, (int)g_ecu);
        }
        break;
    }

    case PAGE_WIFI: {
        const int nAp = netScanCount();
        if (i < LIST_ROWS) {
            const int idx = s_apPage * LIST_ROWS + i;
            char ssid[34]; int rssi = 0; unsigned ch = 0;
            if (idx < nAp) { netApGet(idx, ssid, sizeof(ssid), &rssi, &ch); }
            else           { ssid[0] = 0; }
            snprintf(o, n, "a|%s|%d|%u|%d|%d", ssid, rssi, ch,
                     (idx == s_apCur) ? 1 : 0, nAp);
        } else {
            snprintf(o, n, "w|%d|%d|%d|%d", nAp, s_apPage, s_apCur,
                     netHasCred() ? 1 : 0);
        }
        break;
    }

    case PAGE_DIAG:
        switch (i) {
        case 0: snprintf(o, n, "d0|%d", (int)g_mode); break;
        case 1: snprintf(o, n, "d1|%d", (int)g_link); break;
        case 2: snprintf(o, n, "d2|%d", (int)g_svc);  break;
        case 3: snprintf(o, n, "d3|%d", (int)g_ecu);  break;
        case 4: snprintf(o, n, "d4|%d", obdSavedBound() ? 1 : 0); break;
        case 5: snprintf(o, n, "d5|%d", touchReady() ? 1 : 0);    break;
        case 6: snprintf(o, n, "d6|%d", gpsFixQ());   break;
        default: snprintf(o, n, "d7|%d", (int)g_uisd); break;
        }
        break;

    default: /* PAGE_ADV */
        if (i < kAdvPidRows) {
            const int index = s_advPidPage * kAdvPidRows + i;
            const int np = obdPidCount();
            if (index >= np) { snprintf(o, n, "p|none|%d", np); }
            else {
                const PidRec *p = obdPid(index);
                bool fresh = false, sim = false;
                const float value = obdVal(index, &fresh, &sim);
                /* 毫秒值量化到 100ms，避免每个循环都判成"变了" */
                const unsigned long ms = p->lastOkMs
                    ? (unsigned long)((millis() - p->lastOkMs) / 100) : 0xFFFFFFFFUL;
                snprintf(o, n, "p|%lu|%lu|%lu|%u|%u|%d|%.2f", (unsigned long)p->ok,
                         (unsigned long)p->req, ms, (unsigned)p->supported,
                         (unsigned)p->lastResult, fresh ? 1 : 0, sim ? value : 0.f);
            }
        } else {
            const int k = i - 6;
            switch (k) {
            case 0: snprintf(o, n, "t0|%s|%s|%s", obdLinkStr(), obdSvcStr(), obdEcuStr()); break;
            case 1: snprintf(o, n, "t1|%lu|%lu|%lu|%lu", (unsigned long)g_realReq,
                             (unsigned long)g_realOk, (unsigned long)g_realNoData,
                             (unsigned long)g_realTo); break;
            case 2: snprintf(o, n, "t2|%d|%d|%lu|%lu", obdCandCount(),
                             obdSavedBound() ? 1 : 0, (unsigned long)g_sess,
                             (unsigned long)((millis() - g_bootMs) / 1000)); break;
            default: snprintf(o, n, "t3|%s|%d|%d|%lu", g_resetWhy,
                              touchReady() ? 1 : 0, touchCalValid() ? 1 : 0,
                              (unsigned long)touchTaps()); break;
            }
        }
        break;
    }
}

static void bandRect(uint8_t i, int16_t *x, int16_t *y, int16_t *w, int16_t *h)
{
    switch (g_page) {
    case PAGE_MAIN:
        if (i < 4) { *x = 8 + (i % 2) * 236; *y = 42 + (i / 2) * 60; *w = 228; *h = 56; }
        else if (i == 4) { *x = 0; *y = 166; *w = SCR_W; *h = 16; }
        else if (i == 5) { *x = 0; *y = 188; *w = SCR_W; *h = 16; }
        else             { *x = 0; *y = 206; *w = SCR_W; *h = 46; }
        break;
    case PAGE_DEV:
    case PAGE_WIFI:
        if (i < LIST_ROWS) rowRect(i, x, y, w, h);
        else { *x = 0; *y = 204; *w = SCR_W; *h = 68; }
        break;
    case PAGE_DIAG:
        *x = 0; *y = (int16_t)(DIAG_Y0 + DIAG_ROW_H * i); *w = SCR_W; *h = 16;
        break;
    default: /* PAGE_ADV */
        if (i < 6) { *x = 0; *y = (int16_t)(64 + 17 * i); *w = SCR_W; *h = 16; }
        else       { *x = 0; *y = (int16_t)(186 + 11 * (i - 6)); *w = SCR_W; *h = 11; }
        break;
    }
}

/* 局部重画：先按 band 自己的底色清掉（这正是"数字变长不留残影"的关键） */
static void bandPaint(uint8_t i)
{
    int16_t x, y, w, h;
    bandRect(i, &x, &y, &w, &h);

    switch (g_page) {
    case PAGE_MAIN:
        if (i < 4) { drawMainCell(i); return; }        /* 卡片自带底色与边框 */
        gfx->fillRect(x, y, w, h, C_BG);
        if (i == 4)      drawMainVeh();
        else if (i == 5) drawMainGps();
        else { drawMainStore(); drawMainNet(); }
        return;

    case PAGE_DEV: {
        const int nCand = obdCandCount();
        if (i < LIST_ROWS) {
            const int idx = s_devPage * LIST_ROWS + i;
            if (idx < nCand) {
                drawDevRow(i, idx);
            } else {
                gfx->fillRect(x, y, w, h, C_BG);
                if (idx == 0 && nCand == 0) {          /* 空态提示只画一次 */
                    cjkText(16, 56,  "未发现设备", C_WARN, 2);
                    cjkText(16, 100, "点下方 重新扫描 开始搜索", C_DIM, 1);
                    cjkText(16, 126, "手机请先断开盒子的连接", C_DIM, 1);
                    cjkText(16, 152, "同一盒子只能被一台设备连接", C_DIM, 1);
                }
            }
        } else {
            /* 页脚（含分页按钮）整块重画：清底后交给与整页同一个函数，
               保证"局部刷新后分页按钮不会消失" */
            gfx->fillRect(x, y, w, h, C_BG);
            drawDevFooter();
        }
        return;
    }

    case PAGE_WIFI: {
        const int nAp = netScanCount();
        if (i < LIST_ROWS) {
            const int idx = s_apPage * LIST_ROWS + i;
            if (idx < nAp) {
                drawWifiRow(i, idx);
            } else {
                gfx->fillRect(x, y, w, h, C_BG);
                if (idx == 0 && nAp == 0) {
                    cjkText(16, 60, "按下方 扫描网络 搜索附近网络", C_DIM, 1);
                    cjkText(16, 90, "只支持 2.4GHz", C_DIM, 1);
                }
            }
        } else {
            gfx->fillRect(x, y, w, h, C_BG);
            drawWifiFooter();
        }
        return;
    }

    case PAGE_DIAG:
        gfx->fillRect(x, y, w, h, C_BG);
        drawDiagRow(i);
        return;

    default: /* PAGE_ADV */
        gfx->fillRect(x, y, w, h, C_BG);
        if (i < 6) drawAdvPidRow(i);
        else       drawAdvTech(i - 6);
        return;
    }
}

/* ================= 阶段区：签名 + 按区刷新 =================
   只有「阶段区里显示的东西」变了才重画那几块，绝不为了刷新进度整屏重画。
   ★ 由两处调用：① c_obd.emitStage()（阶段/结果变化，可能紧跟一次阻塞写，所以先画）
                  ② uiTick()（时间在走，让「耗时」跟着涨） */
static char s_phaseSig[280];

static void phaseSig(char *o, size_t n)
{
    char pid[26];
    obdLastPidStr(pid, sizeof(pid));
    snprintf(o, n, "%d|%u|%s|%s|%u|%s|%d|%lu|%d|%lu|%u|%s|%u|%s|%s|%s|%s|%d|%d|%s",
             (int)g_page,
             (unsigned)obdPhaseNo(), obdPhaseCode(), obdPhaseResultCode(),
             (unsigned)(obdPhaseMs() / 100), obdPhaseResultStr(), obdRxLen(),
             (unsigned long)obdNotifyCount(), obdNotifyFlag() ? 1 : 0,
             (unsigned long)obdFirstByteMs(),
             (unsigned)obdLastFailPhase(), obdLastFailNote(),
             (unsigned)obdFirstFailPhase(),
             obdAtResultStr(0), obdAtResultStr(1), obdAtResultStr(2),
             pid, obdCandCount(),
             obdPidBusy() ? 1 : 0, obdSentCmd());
}

void uiPhasePaint()
{
    if (!s_lcd) return;
    if (s_calStep > 0) return;
    if (g_page != PAGE_DIAG && g_page != PAGE_ADV) return;

    char s[280];
    phaseSig(s, sizeof(s));
    if (!strcmp(s, s_phaseSig)) return;
    snprintf(s_phaseSig, sizeof(s_phaseSig), "%s", s);

    if (g_page == PAGE_DIAG) drawDiagPhase();
    else                     drawAdvPhase();
}

void uiDraw(bool force)
{
    if (!s_lcd) return;
    if (!force) { uiTick(); return; }

    s_nFull++;
    s_lastDraw = millis();

    gfx->fillScreen(C_BG);
    drawTopBar();

    if (s_calStep > 0) {
        drawCal();
    } else {
        switch (g_page) {
        case PAGE_MAIN: drawMain(); break;
        case PAGE_DEV:  drawDev();  break;
        case PAGE_WIFI: drawWifi(); break;
        case PAGE_DIAG: drawDiag(); break;
        default:        drawAdv();  break;
        }
    }
    drawBottomBar();

    if (g_uisd == SD_ERROR && s_calStep == 0) {
        gfx->fillRect(0, 252, SCR_W, 18, C_ERR);
        cjkText(10, 253, "存储错误 请检查卡", C_TEXT, 1);
    }

    if (s_toast[0] && millis() < s_toastUntil) {
        const int w = cjkWidth(s_toast, 1) + 28;
        const int x = (SCR_W - w) / 2;
        gfx->fillRect(x, 130, w, 40, C_CARD);
        gfx->drawRect(x, 130, w, 40, C_WARN);
        cjkCenter(SCR_W / 2, 142, s_toast, C_WARN, 1);
    }

    if (s_mark && s_markX >= 0) {
        gfx->drawFastHLine(s_markX - 12, s_markY, 25, C_ERR);
        gfx->drawFastVLine(s_markX, s_markY - 12, 25, C_ERR);
    }

    /* 整页画完 = 当前状态全部落地，把影子重置，避免紧接着又重画一遍 */
    s_shownPage = (uint8_t)g_page;
    structSig(s_structSig, sizeof(s_structSig));
    phaseSig(s_phaseSig, sizeof(s_phaseSig));
    const uint8_t nb = pageBands((uint8_t)g_page);
    for (uint8_t i = 0; i < nb && i < BAND_MAX; i++) bandSig(i, s_bandSig[i], sizeof(s_bandSig[i]));
}

void uiTick()
{
    if (!s_lcd) return;
    if (s_calStep > 0) return;                     /* 校准页静态，等点击 */
    if (millis() - s_lastDraw < 150) return;
    s_lastDraw = millis();

    if ((uint8_t)g_page != s_shownPage) { s_nPage++; uiDraw(true); return; }

    char ss[96];
    structSig(ss, sizeof(ss));
    if (strcmp(ss, s_structSig)) { s_nSig++; uiDraw(true); return; }

    const uint8_t nb = pageBands((uint8_t)g_page);
    for (uint8_t i = 0; i < nb && i < BAND_MAX; i++) {
        char bs[48];
        bandSig(i, bs, sizeof(bs));
        if (strcmp(bs, s_bandSig[i])) {
            snprintf(s_bandSig[i], sizeof(s_bandSig[i]), "%s", bs);
            bandPaint(i);
            s_nBand++;
        }
    }

    /* 阶段区：阶段号/结果/耗时一变就只重画那几行 */
    uiPhasePaint();
}

/* 绘制统计：串口每 5 秒一行，只走串口不落卡（不进 SD 日志） */
void uiDrawStats()
{
    Serial.printf("{\"ev\":\"draw\",\"t\":%lu,\"page\":%u,\"full\":%lu,\"sig\":%lu,"
                  "\"band\":%lu,\"switch\":%lu}\n",
                  (unsigned long)millis(), (unsigned)g_page,
                  (unsigned long)s_nFull, (unsigned long)s_nSig,
                  (unsigned long)s_nBand, (unsigned long)s_nPage);
}

void uiDrawStatsReset()
{
    s_nFull = s_nSig = s_nBand = s_nPage = 0;
}

/* ================= 动作执行 ================= */
static void uiAct(uint8_t id)
{
    switch (id) {
    case HIT_MAIN_DEV:
        g_page = PAGE_DEV;
        if (obdCandCount() == 0 && g_link != LINK_CONNECTED) {
            obdRequestScan();
            uiToast("正在扫描", 2000);
        }
        break;
    case HIT_MAIN_WIFI: g_page = PAGE_WIFI; break;
    case HIT_MAIN_DIAG: g_page = PAGE_DIAG; break;

    case HIT_DEV_PREV:
        if (s_devPage > 0) { s_devPage--; s_devCur = s_devPage * LIST_ROWS; }
        break;
    case HIT_DEV_NEXT:
        if ((s_devPage + 1) * LIST_ROWS < obdCandCount()) {
            s_devPage++;
            s_devCur = s_devPage * LIST_ROWS;
        }
        break;
    case HIT_DEV_RESCAN:
        s_devPage = 0; s_devCur = 0;
        obdRequestScan();
        uiToast("正在扫描", 2000);
        break;
    case HIT_DEV_CONNECT: {
        const int n = obdCandCount();
        if (n == 0)        { uiToast("请先扫描", 1500); break; }
        if (s_devCur >= n) { uiToast("请先选择设备", 1500); break; }
        obdPickToIndex(s_devCur);
        obdSelectCurrent();
        uiToast("正在连接", 2500);
        break;
    }
    case HIT_DEV_BACK: g_page = PAGE_MAIN; break;

    case HIT_WIFI_PREV:
        if (s_apPage > 0) { s_apPage--; s_apCur = s_apPage * LIST_ROWS; }
        break;
    case HIT_WIFI_NEXT:
        if ((s_apPage + 1) * LIST_ROWS < netScanCount()) {
            s_apPage++;
            s_apCur = s_apPage * LIST_ROWS;
        }
        break;
    case HIT_WIFI_SCAN:
        s_apPage = 0; s_apCur = 0;
        netScan();
        uiToast("正在扫描网络", 2000);
        break;
    case HIT_WIFI_CONNECT: {
        const int n = netScanCount();
        if (n == 0)       { uiToast("请先扫描网络", 1500); break; }
        if (s_apCur >= n) { uiToast("请先选择网络", 1500); break; }
        char ssid[34];
        netApGet(s_apCur, ssid, sizeof(ssid), nullptr, nullptr);
        if (!netHasCred() || strcmp(ssid, netSsid()) != 0) {
            uiToast("密码需串口设置", 2500);
            break;
        }
        netConnect();
        uiToast("正在连接", 2000);
        break;
    }
    case HIT_WIFI_BACK: g_page = PAGE_MAIN; break;

    case HIT_DIAG_ADV:  g_page = PAGE_ADV; break;
    case HIT_DIAG_BACK: g_page = PAGE_MAIN; break;
    case HIT_DIAG_SAVE:
        /* ★ 诊断版：屏幕直接封存行程，无需串口。
           与串口 SAVE 走同一个 storeFinishAndSave()。 */
        if (storeIsRecording()) { uiToast("保存中...", 2500); uiDraw(true); storeFinishAndSave(); }
        else                    { uiToast("未在记录", 1200); }
        break;

    case HIT_ADV_CAL:
        s_calStep = 1;
        uiToast("请点屏幕左上角", 2500);
        break;
    case HIT_ADV_BACK: g_page = PAGE_DIAG; break;
    case HIT_ADV_PIDS:
        s_advPidPage = (s_advPidPage + 1) % ((obdPidCount() + kAdvPidRows - 1) / kAdvPidRows);
        break;

    case HIT_CAL_START:  s_calStep = 1; uiToast("请点屏幕左上角", 2500); break;
    case HIT_CAL_CANCEL: s_calStep = 0; break;

    default: break;
    }

    uiDraw(true);
}

/* ================= 触摸入口：返回命中的 ID ================= */
uint8_t uiTouch(int16_t x, int16_t y)
{
    uint8_t id = HIT_NONE;

    if (s_calStep > 0) {
        id = hitBtns(kCalBot, 2, x, y);
        if (id != HIT_NONE) { uiAct(id); return id; }

        /* 其余点击都当作校准点 */
        /* 两个校准点按"原始坐标"原样保存，方向由这两点反解，不再猜轴序 */
        int16_t rx = 0, ry = 0;
        touchLastRaw(&rx, &ry);
        if (s_calStep == 1) {
            s_calTX0 = rx; s_calTY0 = ry;
            s_calStep = 2;
            uiToast("请点右下角", 2500);
        } else {
            touchSaveCalRaw(s_calTX0, s_calTY0, rx, ry);
            s_calStep = 0;
            uiToast("校准完成", 2500);
        }
            uiDraw(true);
        return HIT_CAL_START;      /* 记为校准点，日志里能看出来 */
    }

    switch (g_page) {
    case PAGE_MAIN:
        id = hitBtns(kMainBot, 3, x, y);
        break;

    case PAGE_DEV: {
        id = hitBtns(kDevPageBtns, 2, x, y);
        if (id == HIT_NONE) id = hitBtns(kDevBot, 3, x, y);
        if (id == HIT_NONE) {
            const int n = obdCandCount();
            for (int i = 0; i < LIST_ROWS; i++) {
                const int idx = s_devPage * LIST_ROWS + i;
                if (idx >= n) break;
                int16_t rx, ry, rw, rh;
                rowRect(i, &rx, &ry, &rw, &rh);
                if (hit(x, y, rx, ry, rw, rh)) {
                    s_devCur = idx;
                    id = (uint8_t)(HIT_ROW_BASE + i);
                    break;
                }
            }
        }
        break;
    }

    case PAGE_WIFI: {
        id = hitBtns(kWifiPageBtns, 2, x, y);
        if (id == HIT_NONE) id = hitBtns(kWifiBot, 3, x, y);
        if (id == HIT_NONE) {
            const int n = netScanCount();
            for (int i = 0; i < LIST_ROWS; i++) {
                const int idx = s_apPage * LIST_ROWS + i;
                if (idx >= n) break;
                int16_t rx, ry, rw, rh;
                rowRect(i, &rx, &ry, &rw, &rh);
                if (hit(x, y, rx, ry, rw, rh)) {
                    s_apCur = idx;
                    id = (uint8_t)(HIT_ROW_BASE + i);
                    break;
                }
            }
        }
        break;
    }

    case PAGE_DIAG:
        id = hitBtns(kDiagBot, 3, x, y);
        break;

    default: /* PAGE_ADV */
        id = hitBtns(kAdvBot, 3, x, y);
        break;
    }

    if (id != HIT_NONE) uiAct(id);
    else {
            uiDraw(true);
    }
    return id;
}

/* ================= BOOT 键（备用操作，保留） ================= */
void uiNextPage()
{
    if (s_calStep > 0) { s_calStep = 0; }
    else { g_page = (V2Page)(((int)g_page + 1) % (int)PAGE_N); }
    uiDraw(true);
}

void uiAction()
{
    switch (g_page) {
    case PAGE_MAIN: uiAct(HIT_MAIN_DEV); break;
    case PAGE_DEV:  uiAct(HIT_DEV_CONNECT); break;
    case PAGE_WIFI: uiAct(HIT_WIFI_CONNECT); break;
    case PAGE_DIAG: uiAct(HIT_DIAG_ADV); break;
    default:
        if (s_calStep == 0) uiAct(HIT_ADV_CAL);
        else                uiAct(HIT_CAL_START);
        break;
    }
}
