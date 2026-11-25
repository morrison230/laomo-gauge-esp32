/*
 * b_gps.ino —— PG-131F NMEA → 合法 JSON
 *
 * V1 的坏行根因：把带前导零的 NMEA 原文 `0123.45` 当成 JSON 数字直接拼进字符串，
 * 前导零使整行非法（2,764 条坏行，占 5.4%）。
 *
 * V2 做法：
 *   · 不再"挑字段原文塞进去"，而是把 ddmm.mmmm 换算成十进制度数（double），
 *     用 %.6f 输出 —— 数值天然不会出现前导零。
 *   · 无效值统一输出 null，不再是空字符串。
 *   · 输出前仍过 jsonValidate()（双保险）。
 */
#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "v2.h"

static uint32_t s_nmea = 0, s_bad = 0;
static int s_fixQ = -1, s_sats = -1;         /* REAL */
static int s_simQ = -1, s_simSats = -1;      /* SIM（只影响 UI 显示，不计入 REAL 统计） */
static char s_utc[18] = "--";

void gpsBegin()
{
    Serial2.setRxBufferSize(2048);
    Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, -1);
    Serial2.setTimeout(0);
    Serial.printf("{\"v\":2,\"ev\":\"gps-init\",\"rx_pin\":%d,\"baud\":%d}\n",
                  PIN_GPS_RX, GPS_BAUD);
}

int gpsFixQ()  { return (g_mode == MODE_SIM) ? s_simQ : s_fixQ; }
int gpsSats()  { return (g_mode == MODE_SIM) ? s_simSats : s_sats; }

void gpsSimSet(int q, int sat)
{
    s_simQ = q;
    s_simSats = sat;
}

/* NMEA 校验：$ 与 * 之间所有字符异或 */
static bool nmeaOk(const char *s)
{
    if (!s || s[0] != '$') return false;
    const char *star = strchr(s, '*');
    if (!star || !star[1] || !star[2]) return false;
    uint8_t x = 0;
    for (const char *p = s + 1; p < star; p++) x ^= (uint8_t)*p;
    char h[3] = { star[1], star[2], 0 };
    return (uint8_t)strtoul(h, nullptr, 16) == x;
}

static bool nmeaField(const char *s, int idx, char *out, size_t n)
{
    out[0] = 0;
    const char *p = s;
    for (int i = 0; i < idx; i++) {
        p = strchr(p, ',');
        if (!p) return false;
        p++;
    }
    const char *e = p;
    while (*e && *e != ',' && *e != '*') e++;
    size_t len = (size_t)(e - p);
    if (len >= n) len = n - 1;
    memcpy(out, p, len);
    out[len] = 0;
    return len > 0;
}

/* ddmm.mmmm / dddmm.mmmm → 十进制度数。非法返回 NAN。 */
static double nmeaToDeg(const char *raw, bool isLon)
{
    if (!raw || !raw[0]) return NAN;
    char *endp = nullptr;
    const double v = strtod(raw, &endp);
    if (endp == raw || !isfinite(v) || v < 0.0) return NAN;

    const double d = floor(v / 100.0);
    const double m = v - d * 100.0;
    if (m < 0.0 || m >= 60.0) return NAN;
    const double dec = d + m / 60.0;
    if (isLon ? (dec < 0.0 || dec > 180.0) : (dec < 0.0 || dec > 90.0)) return NAN;
    return dec;
}

static void handleGga(const char *line)
{
    char utc[16] = {0}, latRaw[20] = {0}, ns[4] = {0}, lonRaw[20] = {0}, ew[4] = {0};
    char q[8] = {0}, sat[8] = {0}, hdop[12] = {0}, alt[16] = {0};

    nmeaField(line, 1, utc,    sizeof(utc));
    nmeaField(line, 2, latRaw, sizeof(latRaw));
    nmeaField(line, 3, ns,     sizeof(ns));
    nmeaField(line, 4, lonRaw, sizeof(lonRaw));
    nmeaField(line, 5, ew,     sizeof(ew));
    nmeaField(line, 6, q,      sizeof(q));
    nmeaField(line, 7, sat,    sizeof(sat));
    nmeaField(line, 8, hdop,   sizeof(hdop));
    nmeaField(line, 9, alt,    sizeof(alt));

    const int quality = q[0]   ? atoi(q) : -1;
    const int nsat    = sat[0] ? atoi(sat) : -1;

    double lat = nmeaToDeg(latRaw, false);
    double lon = nmeaToDeg(lonRaw, true);
    /* 南纬/西经取负 */
    if (ns[0] == 'S' || ns[0] == 's') { if (isfinite(lat)) lat = -lat; }
    if (ew[0] == 'W' || ew[0] == 'w') { if (isfinite(lon)) lon = -lon; }

    /* 无定位时坐标必然是空的 → null */
    if (quality <= 0 || !isfinite(lat) || !isfinite(lon)) { lat = NAN; lon = NAN; }
    if (nsat < 0) { /* 保持 -1，表示字段缺失 */ }

    s_fixQ = quality;
    s_sats = nsat;
    if (utc[0]) snprintf(s_utc, sizeof(s_utc), "%.15s", utc);

    double altM = NAN;
    if (alt[0]) {
        char *e2 = nullptr;
        const double a = strtod(alt, &e2);
        if (e2 != alt && isfinite(a)) altM = a;
    }
    double hd = NAN;
    if (hdop[0]) {
        char *e3 = nullptr;
        const double h = strtod(hdop, &e3);
        if (e3 != hdop && isfinite(h)) hd = h;
    }

    /* ---- 组装：数值用 %.6f / %d，缺失用 null ---- */
    char latS[24], lonS[24], altS[24], hdS[24];
    if (isfinite(lat)) snprintf(latS, sizeof(latS), "%.6f", lat); else snprintf(latS, sizeof(latS), "null");
    if (isfinite(lon)) snprintf(lonS, sizeof(lonS), "%.6f", lon); else snprintf(lonS, sizeof(lonS), "null");
    if (isfinite(altM)) snprintf(altS, sizeof(altS), "%.1f", altM); else snprintf(altS, sizeof(altS), "null");
    if (isfinite(hd))  snprintf(hdS, sizeof(hdS), "%.1f", hd);  else snprintf(hdS, sizeof(hdS), "null");

    char body[340];
    snprintf(body, sizeof(body),
             "{\"v\":2,\"t\":%lu,\"src\":\"gps\",\"ev\":\"gga\",\"org\":\"real\",\"sess\":%lu,"
             "\"ok\":%s,\"q\":%d,\"sat\":%d,\"hdop\":%s,\"alt_m\":%s,"
             "\"lat\":%s,\"lon\":%s,\"utc\":\"%s\"}",
             (unsigned long)millis(), (unsigned long)g_sess,
             (quality > 0 && isfinite(lat) && isfinite(lon)) ? "true" : "false",
             quality, nsat, hdS, altS, latS, lonS,
             utc[0] ? utc : "--");

    logJson(body);                      /* 内部还会再校验一次 */
}

void gpsTick()
{
    static char line[160];
    static size_t len = 0;
    while (Serial2.available() > 0) {
        const int c = Serial2.read();
        if (c < 0) break;
        if (c == '\n') {
            line[len] = 0;
            if (len && line[len - 1] == '\r') line[--len] = 0;
            if (len) {
                if (nmeaOk(line)) {
                    s_nmea++;
                    if (!strncmp(line + 3, "GGA", 3)) handleGga(line);
                } else {
                    s_bad++;
                }
            }
            len = 0;
        } else if (len + 1 < sizeof(line)) {
            line[len++] = (char)c;
        } else {
            len = 0;
        }
    }
}

/* 供 STATUS / GPS? 使用。out==nullptr 时直接打串口 */
void gpsStr(char *out, size_t n)
{
    char body[260];
    snprintf(body, sizeof(body),
             "{\"ev\":\"gps\",\"org\":\"%s\",\"q\":%d,\"sat\":%d,\"nmea\":%lu,\"bad\":%lu,"
             "\"utc\":\"%s\"}",
             g_mode == MODE_SIM ? "sim" : "real",
             gpsFixQ(), gpsSats(), (unsigned long)s_nmea, (unsigned long)s_bad, s_utc);
    if (out && n) snprintf(out, n, "%s", body);
    else Serial.println(body);
}
