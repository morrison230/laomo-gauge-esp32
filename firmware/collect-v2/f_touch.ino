/*
 * f_touch.ino —— FT6336 电容触摸（最小实现，直接读寄存器）
 *
 * 为什么自己写而不直接用 SensorLib 的 TouchDrvFT6X36：
 *   1. 它的 begin() 走 SensorComm/Hal 抽象层，底层用了哪个 Wire 实例不直观，
 *      一旦和 TCA9554 抢总线会很难查；
 *   2. 我们只需要"读一个点"，寄存器协议就 6 个字节，自己写反而可控；
 *   3. 不引入新的库依赖，编译风险最小（今天已经上板跑通了，不能为触摸把它搞坏）。
 *
 * 硬件事实（以本工程实测为准，不凭型号猜）：
 *   · I2C 总线已经由 uiBegin() 里的 Wire.begin(21,22) 建立，TCA9554 在 0x20
 *   · FT6336 从地址 0x38（SensorLib 的 REG/FT6X36Constants.h 也是这个值）
 *   · 复位：本板 TCA9554 的 P0/P1 在 lcdInit() 里已经做过复位时序，
 *     触摸不再单独拉 RST（避免多一处可能把 LCD 弄黑的动作）
 *   · INT 不使用：直接轮询 0x02 状态寄存器，少一根引脚依赖
 *
 * 寄存器（FT6x36 系列通用）：
 *   0x02 TD_STATUS 低 4 位 = 触点数
 *   0x03..0x06       = XH,XL,YH,YL（各 12 位有效）
 *   0xA8 芯片 ID（FT6336 通常 0x64），只用于识别日志，不作判据
 *
 * 坐标映射：面板原生是竖屏，显示用 rot=1 转了 90°。所以触摸必须做变换。
 * **不猜方向**：提供两点校准（左上、右下），由实测决定轴序与镜像，存 NVS。
 */
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

#include "v2.h"

#define FT_ADDR      0x38
#define FT_REG_STATUS 0x02
#define FT_REG_POINT  0x03
#define FT_REG_CHIPID 0xA8

static bool     s_touchOk   = false;
static uint8_t  s_touchAddr = 0;
static uint8_t  s_chipId    = 0;
static int      s_points    = 0;

/* 原始坐标（最近一次按下） */
static int16_t  s_rawX = 0, s_rawY = 0;
static bool     s_down = false;
static int16_t  s_downX = 0, s_downY = 0;

/* 校准：存"屏幕左上角"和"屏幕右下角"这两个物理点对应的**原始坐标**。
   把方向信息全部交给实测点，代码里不再出现"要不要取反"的假设。
     (s_calRX0, s_calRY0) = 物理左上角（实体按键一侧为上方）上报的 (raw 轴0, raw 轴1)
     (s_calRX1, s_calRY1) = 物理右下角上报的同一对原始值                            */
static bool     s_calValid = false;
static int16_t  s_calRX0 = 0, s_calRY0 = 0, s_calRX1 = 0, s_calRY1 = 0;

/* 统计，供诊断页显示 */
static uint32_t s_irqCount = 0, s_badRead = 0;
static int16_t  s_lastSX = -1, s_lastSY = -1;

/* ================= 底层 I2C ================= */
static bool ftWriteReg(uint8_t reg)
{
    Wire.beginTransmission(s_touchAddr);
    Wire.write(reg);
    return Wire.endTransmission(false) == 0;
}

static int ftRead(uint8_t reg, uint8_t *buf, size_t n)
{
    if (!ftWriteReg(reg)) return -1;
    size_t got = Wire.requestFrom((int)s_touchAddr, (int)n);
    if (got != n) return -1;
    for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return (int)n;
}

/* 扫一遍总线，把发现的地址打到串口 —— 这就是"实际识别结果"的证据 */
int touchScanBus(char *out, size_t n)
{
    char tmp[200];
    size_t o = 0;
    int found = 0;
    o += (size_t)snprintf(tmp + o, sizeof(tmp) - o, "[");
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            found++;
            if (o + 8 < sizeof(tmp)) {
                o += (size_t)snprintf(tmp + o, sizeof(tmp) - o, "%s\"0x%02X\"", found > 1 ? "," : "", a);
            }
        }
    }
    snprintf(tmp + o, sizeof(tmp) - o, "]");
    if (out && n) snprintf(out, n, "%s", tmp);
    return found;
}

/* ================= 初始化 ================= */
bool touchBegin()
{
    char list[200] = {0};
    const int n = touchScanBus(list, sizeof(list));
    Serial.printf("\n{\"v\":2,\"ev\":\"i2c-scan\",\"n\":%d,\"addrs\":%s}\n", n, list);

    if (n == 0) {
        Serial.println(F("{\"v\":2,\"ev\":\"touch-absent\",\"why\":\"bus-empty\"}"));
        return false;
    }

    /* 先试 0x38；不行再试 0x15（FT6206 兼容地址） */
    const uint8_t cands[2] = { FT_ADDR, 0x15 };
    for (int i = 0; i < 2 && !s_touchOk; i++) {
        s_touchAddr = cands[i];
        uint8_t id = 0;
        if (ftRead(FT_REG_CHIPID, &id, 1) == 1) {
            s_chipId = id;
            uint8_t st = 0;
            if (ftRead(FT_REG_STATUS, &st, 1) == 1) {
                s_touchOk = true;
                Serial.printf("{\"v\":2,\"ev\":\"touch-ok\",\"addr\":\"0x%02X\",\"chip\":\"0x%02X\"}\n",
                              s_touchAddr, s_chipId);
            }
        }
    }
    if (!s_touchOk) {
        Serial.printf("{\"v\":2,\"ev\":\"touch-absent\",\"why\":\"no-ack\",\"tried\":\"0x38,0x15\"}\n");
        return false;
    }

    touchLoadCal();
    Serial.printf("{\"v\":2,\"ev\":\"touch-cal\",\"valid\":%d,"
                  "\"tl_raw\":[%d,%d],\"br_raw\":[%d,%d]}\n",
                  s_calValid ? 1 : 0, s_calRX0, s_calRY0, s_calRX1, s_calRY1);
    return true;
}

bool touchReady()    { return s_touchOk; }
bool touchCalValid() { return s_calValid; }
uint32_t touchTaps()  { return (uint32_t)s_irqCount; }

/* ================= 校准存取（NVS） ================= */
/* v2 格式：tch_v2=1 时才认，存的是左上/右下两点的**原始**坐标。
   旧格式（tch_ok 无 tch_v2）记录的是"映射后的值 + swap 标志"，
   那套记录无法表达取反方向，直接作废回落到内置默认映射。 */
void touchLoadCal()
{
    Preferences p;
    p.begin("sx4", true);
    const bool v2 = p.getBool("tch_v2", false);
    s_calValid = v2;
    if (v2) {
        s_calRX0 = (int16_t)p.getShort("tch_rx0", 0);
        s_calRY0 = (int16_t)p.getShort("tch_ry0", 479);
        s_calRX1 = (int16_t)p.getShort("tch_rx1", 319);
        s_calRY1 = (int16_t)p.getShort("tch_ry1", 0);
    }
    p.end();
}

void touchSaveCalRaw(int16_t rxTL, int16_t ryTL, int16_t rxBR, int16_t ryBR)
{
    Preferences p;
    p.begin("sx4", false);
    p.putBool("tch_v2", true);
    p.putShort("tch_rx0", rxTL); p.putShort("tch_ry0", ryTL);
    p.putShort("tch_rx1", rxBR); p.putShort("tch_ry1", ryBR);
    p.end();
    s_calValid = true;
    s_calRX0 = rxTL; s_calRY0 = ryTL; s_calRX1 = rxBR; s_calRY1 = ryBR;
    Serial.printf("{\"v\":2,\"ev\":\"touch-cal-saved\",\"tl_raw\":[%d,%d],\"br_raw\":[%d,%d]}\n",
                  rxTL, ryTL, rxBR, ryBR);
}

void touchForgetCal()
{
    Preferences p;
    p.begin("sx4", false);
    p.remove("tch_v2"); p.remove("tch_ok"); p.remove("tch_swap");
    p.remove("tch_x0"); p.remove("tch_y0"); p.remove("tch_x1"); p.remove("tch_y1");
    p.remove("tch_rx0"); p.remove("tch_ry0"); p.remove("tch_rx1"); p.remove("tch_ry1");
    p.end();
    s_calValid = false;
    Serial.println(F("{\"v\":2,\"ev\":\"touch-cal-cleared\"}"));
}

/* ================= 读点 ================= */
/* 返回触点数；pts>0 时 rx、ry 是原始坐标 */
static int ftPoll(int16_t *rx, int16_t *ry)
{
    if (!s_touchOk) return 0;
    uint8_t st = 0;
    if (ftRead(FT_REG_STATUS, &st, 1) != 1) { s_badRead++; return -1; }
    const int pts = st & 0x0F;
    s_points = pts;
    if (pts == 0) return 0;

    uint8_t b[4];
    if (ftRead(FT_REG_POINT, b, 4) != 4) { s_badRead++; return -1; }
    *rx = (int16_t)(((b[0] & 0x0F) << 8) | b[1]);
    *ry = (int16_t)(((b[2] & 0x0F) << 8) | b[3]);
    return pts;
}

/* 原始 → 屏幕（480x320）
   ============================================================
   统一物理方向（2026-11-12 用户确认）：**实体按键所在的一侧就是屏幕上方**。
   面板原生竖屏 320x480，横屏 480x320 后两轴互换：
     屏幕 X ← raw 轴1（0..479，方向不取反）
     屏幕 Y ← raw 轴0（0..319，方向取反）

   ★ 免校准默认值来自 2026-11-12 23:30 的**六点受控采样**（不是猜的）：
       点位置            屏幕坐标     上报 raw
       底部蓝牙设备中心   ( 82,296)   ( 11, 79)
       底部无线网络中心   (240,296)   ( 26,252)
       底部系统诊断中心   (398,296)   ( 21,404)
       屏幕正中           (240,160)   (165,236)
       左上数值卡中心     (122, 70)   (235,105)
       右上数值卡中心     (358, 70)   (241,353)
     六点最小二乘解出 sx/sy 的交叉项都≈0（0.068 / 0.030），说明两轴独立；
     再按 raw 量程对齐（轴1 恰好 0..479 = 屏宽、轴0 恰好 0..319 = 屏高）得到：
        sx = ry          sy = 319 - rx
     三个底部按钮回代：raw(11,79)→(79,308)、raw(26,252)→(252,293)、
     raw(21,404)→(404,298)，分别落在「蓝牙设备 / 无线网络 / 系统诊断」内。

   历史教训（两次都错，记录在此防止再犯）：
     · 一次实测只验证"四角对四角"，方向是假设的 → 得不出方向；
     · 一次拿用户口述的一个锚点反推，锚点位置本身记错了 → 得出 sx=479-ry, sy=rx（全反）。
     结论：方向只能由**受控的已知屏幕点**决定，不能由"我以为他点的是那里"决定。
   ============================================================ */
static void mapToScreen(int16_t rx, int16_t ry, int16_t *sx, int16_t *sy)
{
    int32_t px, py;

    if (s_calValid) {
        const int32_t spanX = (int32_t)s_calRY1 - s_calRY0;   /* 屏幕 X 来自 raw 轴1 */
        const int32_t spanY = (int32_t)s_calRX1 - s_calRX0;   /* 屏幕 Y 来自 raw 轴0 */
        px = (spanX != 0) ? (int32_t)((int32_t)ry - s_calRY0) * 479 / spanX : 0;
        py = (spanY != 0) ? (int32_t)((int32_t)rx - s_calRX0) * 319 / spanY : 0;
    } else {
        px = (int32_t)ry;
        py = 319 - (int32_t)rx;
    }

    if (px < 0) px = 0; if (px > 479) px = 479;
    if (py < 0) py = 0; if (py > 319) py = 319;
    *sx = (int16_t)px;
    *sy = (int16_t)py;
}

/* 主循环调用：检测"按下→抬起"产生一次点击。
   返回 true 时 sx、sy 是屏幕坐标。 */
bool touchTick(int16_t *sx, int16_t *sy)
{
    if (!s_touchOk) return false;

    int16_t rx = 0, ry = 0;
    const int pts = ftPoll(&rx, &ry);
    if (pts < 0) return false;

    if (pts > 0) {
        if (!s_down) {
            s_down = true;
            s_downX = rx; s_downY = ry;
            s_irqCount++;
        }
        s_rawX = rx; s_rawY = ry;
        return false;                 /* 按下不触发，抬起才算一次点击 */
    }

    if (s_down) {
        s_down = false;
        int16_t mx, my;
        mapToScreen(s_downX, s_downY, &mx, &my);
        s_lastSX = mx; s_lastSY = my;
        if (sx) *sx = mx;
        if (sy) *sy = my;
        return true;                  /* 抬起的瞬间，用按下位置判定 */
    }
    return false;
}

/* 供校准页使用：拿最近一次原始坐标 */
void touchLastRaw(int16_t *rx, int16_t *ry)
{
    if (rx) *rx = s_rawX;
    if (ry) *ry = s_rawY;
}

/* ================= 触摸日志（现场核对命中用） ================= */
#define TLOG_N 24
struct TouchEv {
    uint32_t ms;
    int16_t  rx, ry;      /* 原始坐标 */
    int16_t  sx, sy;      /* 换算后屏幕坐标 */
    uint8_t  page;        /* 点击发生时的页面 */
    uint8_t  hit;         /* 最终命中的 HitId，0 = 未命中 */
};
static TouchEv s_tlog[TLOG_N];
static uint8_t s_tlogHead = 0;
static uint8_t s_tlogCnt  = 0;

void touchLogAdd(int16_t rx, int16_t ry, int16_t sx, int16_t sy, uint8_t page, uint8_t hit)
{
    TouchEv &e = s_tlog[s_tlogHead];
    e.ms = millis();
    e.rx = rx; e.ry = ry;
    e.sx = sx; e.sy = sy;
    e.page = page; e.hit = hit;
    s_tlogHead = (uint8_t)((s_tlogHead + 1) % TLOG_N);
    if (s_tlogCnt < TLOG_N) s_tlogCnt++;
}

void touchLogDump()
{
    Serial.printf("{\"ev\":\"tap-log\",\"n\":%u}\n", (unsigned)s_tlogCnt);
    for (uint8_t i = 0; i < s_tlogCnt; i++) {
        const uint8_t idx = (uint8_t)((s_tlogHead + TLOG_N - s_tlogCnt + i) % TLOG_N);
        const TouchEv &e = s_tlog[idx];
        Serial.printf("{\"ev\":\"tap\",\"i\":%u,\"ms\":%lu,\"raw\":[%d,%d],\"scr\":[%d,%d],"
                      "\"page\":%u,\"hitid\":%u,\"hit\":\"%s\"}\n",
                      (unsigned)i, (unsigned long)e.ms, e.rx, e.ry, e.sx, e.sy,
                      (unsigned)e.page, (unsigned)e.hit, uiHitName(e.hit));
    }
}

/* 供诊断输出 */
void touchDiagJson()
{
    Serial.printf("{\"ev\":\"touch-diag\",\"ok\":%d,\"addr\":\"0x%02X\",\"chip\":\"0x%02X\","
                  "\"pts\":%d,\"taps\":%lu,\"bad\":%lu,\"cal\":%d,"
                  "\"last\":[%d,%d],\"raw\":[%d,%d]}\n",
                  s_touchOk ? 1 : 0, s_touchAddr, s_chipId, s_points,
                  (unsigned long)s_irqCount, (unsigned long)s_badRead,
                  s_calValid ? 1 : 0,
                  s_lastSX, s_lastSY, s_rawX, s_rawY);
}
