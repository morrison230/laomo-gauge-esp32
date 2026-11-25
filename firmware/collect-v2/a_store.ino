/*
 * a_store.ino —— SD 写入 / 核验 / 断电标记 / JSON 校验
 *
 * V1 的两个缺陷在这里修掉：
 *   1. `trip-end.bytes` 是 flush 前快照，比真实文件小 111~917 字节。
 *      V2：写 trip-end 之前**先 flush**，并加 `self_len` 字段，使
 *          `bytes + self_len + 1 == 文件大小` 成为一个可验算的恒等式。
 *   2. GPS JSON 的经纬度前导零（2,764 条坏行）。
 *      V2：所有输出行先过 jsonValidate()，不合法拒写并记 json-invalid。
 *
 * 断电：
 *   · 周期 flush 收紧到 500 ms
 *   · 开文件时写 /trips/.open.json，正常保存时删除
 *   · 下次启动读它 → 判断"上次是不是异常结束"，并在新文件里记一行 prev-abnormal
 *   · 永远只用 O_APPEND 语义追加，绝不重写旧文件
 */
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <mbedtls/sha256.h>

#include "v2.h"

static bool     s_ok = false;
static bool     s_recording = false;
static bool     s_finished = false;
static int      s_fd = -1;
static char     s_pathVfs[40] = {0};
static char     s_pathPosix[48] = {0};
static const char *MARKER_VFS   = TRIPS_VFS "/.open.json";
static const char *MARKER_POSIX = SD_MOUNT TRIPS_VFS "/.open.json";

static char     s_buf[FLUSH_BYTES];
static size_t   s_bufLen = 0;
static uint32_t s_lastFlush = 0;
static uint32_t s_lines = 0;
static uint32_t s_bytesW = 0;
static uint32_t s_errWrites = 0;
static uint32_t s_lostBytes = 0;
static uint32_t s_jsonBad = 0;

/* 上一轮的异常结束信息：发生在开新文件**之前**，当时还没有文件可写，
   所以先存起来，等新文件打开后作为首条 sess-begin 落卡。 */
static bool   s_prevFound = false;
static char   s_prevPath[48] = {0};
static long   s_prevSize = -1;
static int    s_prevTrunc = -1;

static mbedtls_sha256_context s_sha;
static bool     s_shaActive = false;
static char     s_expSha[65] = {0};
static char     s_actSha[65] = {0};

/* ================= 小工具 ================= */
static void shaHex(const unsigned char *dg, char *out)
{
    static const char *H = "0123456789abcdef";
    for (int i = 0; i < 32; i++) {
        out[i * 2]     = H[dg[i] >> 4];
        out[i * 2 + 1] = H[dg[i] & 0x0F];
    }
    out[64] = 0;
}

/* ================= JSON 校验器 =================
 * 目标：在**写卡之前**拦住非法行，而不是回家分析时才发现。
 * 规则（覆盖 V1 实际踩到的全部情况）：
 *   · 必须是单个 {} 对象，括号配平
 *   · 字符串必须闭合，字符串内不得出现未转义控制字符
 *   · 数值不得有前导零（"lon":0123.45 / "sat":00）
 *   · 数值不得以 '+' 开头
 */
bool jsonValidate(const char *s)
{
    if (!s || s[0] != '{') return false;
    int depth = 0;
    bool inStr = false, esc = false;
    for (const char *p = s; *p; p++) {
        const unsigned char c = (unsigned char)*p;
        if (inStr) {
            if (esc) { esc = false; continue; }
            if (c == '\\') { esc = true; continue; }
            if (c == '"')  { inStr = false; continue; }
            if (c < 0x20) return false;            /* 未转义控制字符 */
            continue;
        }
        if (c == '"') { inStr = true; continue; }
        if (c == '{') { depth++; continue; }
        if (c == '}') { depth--; if (depth < 0) return false; continue; }
        if (c < 0x20) return false;

        /* 找前一个非空格字符，判断是不是"值的开头" */
        const char *q = p - 1;
        while (q >= s && *q == ' ') q--;
        const char prev = (q >= s) ? *q : 0;
        const bool valueStart = (prev == ':' || prev == ',' || prev == '[');

        if (valueStart) {
            if (c == '+') return false;
            if (c == '0' && p[1] >= '0' && p[1] <= '9') return false;   /* 前导零 */
        }
    }
    return (!inStr) && (depth == 0);
}

/* ================= 写入 ================= */
static bool writeAll(int fd, const uint8_t *p, size_t n, size_t *outWritten)
{
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::write(fd, p + off, n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            *outWritten = off;
            return false;
        }
        if (w == 0) { errno = EIO; *outWritten = off; return false; }
        off += (size_t)w;
    }
    *outWritten = off;
    return true;
}

bool storeFlush()
{
    if (!s_recording || s_bufLen == 0) return true;

    const size_t want = s_bufLen;
    size_t wrote = 0;
    errno = 0;
    const bool wOk = writeAll(s_fd, (const uint8_t *)s_buf, want, &wrote);
    const int wErr = errno;

    if (wrote > 0) {
        if (s_shaActive) mbedtls_sha256_update(&s_sha, (const uint8_t *)s_buf, wrote);
        s_bytesW += (uint32_t)wrote;
    }
    s_bufLen = 0;
    s_lastFlush = millis();

    if (!wOk || wrote != want) {
        s_errWrites++;
        s_lostBytes += (uint32_t)(want - wrote);
        g_uisd = SD_ERROR;
        Serial.printf("{\"v\":2,\"ev\":\"sd-write-short\",\"want\":%u,\"got\":%u,\"errno\":%d}\n",
                      (unsigned)want, (unsigned)wrote, wErr);
        return false;
    }

    /* ★ fsync 的返回值 —— File::flush() 会吞掉的就是这一步 */
    errno = 0;
    const int rc = ::fsync(s_fd);
    if (rc != 0) {
        const int e = errno;
        s_errWrites++;
        g_uisd = SD_ERROR;
        Serial.printf("{\"v\":2,\"ev\":\"sd-sync-fail\",\"rc\":%d,\"errno\":%d}\n", rc, e);
        return false;
    }
    return true;
}

void logJson(const char *line)
{
    /* ★ 先打串口再写卡：现场诊断只能看串口，V1 就是这样双写的。
       少了这一句，stat/value/rx 在串口上完全看不到，台上就成了瞎子。 */
    Serial.println(line);

    if (!s_recording) return;

    if (!jsonValidate(line)) {
        s_jsonBad++;
        Serial.printf("{\"v\":2,\"ev\":\"json-invalid\",\"len\":%u,\"n\":%lu}\n",
                      (unsigned)strlen(line), (unsigned long)s_jsonBad);
        return;                      /* 拒写非法行，宁缺不错 */
    }

    const size_t n = strlen(line);
    if (n + 2 >= sizeof(s_buf)) {
        Serial.println(F("{\"v\":2,\"ev\":\"line-too-long\"}"));
        return;
    }
    if (s_bufLen + n + 2 > sizeof(s_buf)) {
        if (!storeFlush()) return;
    }
    memcpy(s_buf + s_bufLen, line, n);
    s_bufLen += n;
    s_buf[s_bufLen++] = '\n';
    s_lines++;
    if (s_bufLen >= FLUSH_BYTES - 400) storeFlush();
}

void logJsonf(const char *fmt, ...)
{
    char body[460];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    logJson(body);
}

/* ================= 挂载 ================= */
static const char *cardTypeName(uint8_t t)
{
    switch (t) {
    case CARD_MMC:  return "MMC";
    case CARD_SD:   return "SDSC";
    case CARD_SDHC: return "SDHC";
    case CARD_NONE: return "NONE";
    default:        return "UNKNOWN";
    }
}

bool storeMount()
{
    Serial.printf("{\"v\":2,\"ev\":\"sd-init\",\"cs\":%d,\"sck\":%d,\"miso\":%d,\"mosi\":%d}\n",
                  PIN_SD_CS, PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SD_CS);

    errno = 0;
    /* 第 6 个参数 false = 挂载失败绝不格式化 */
    const bool ok = SD.begin(PIN_SD_CS, SPI, SD_FREQ_HZ, SD_MOUNT, 5, false);
    const int mountErrno = errno;
    Serial.printf("{\"v\":2,\"ev\":\"sd-mount\",\"ret\":%d,\"errno\":%d}\n",
                  ok ? 1 : 0, mountErrno);
    if (!ok) {
        g_uisd = SD_NONE;
        Serial.printf("{\"v\":2,\"ev\":\"sd-fail\",\"why\":\"mount\",\"formatted\":false,"
                      "\"errno\":%d}\n", mountErrno);
        return false;
    }

    const uint8_t t = SD.cardType();
    if (t == CARD_NONE) {
        g_uisd = SD_NONE;
        Serial.println(F("{\"v\":2,\"ev\":\"sd-fail\",\"why\":\"no-card\"}"));
        return false;
    }
    Serial.printf("{\"v\":2,\"ev\":\"sd-ok\",\"type\":\"%s\",\"card_mb\":%llu,"
                  "\"total_mb\":%llu,\"used_mb\":%llu,\"free_mb\":%llu}\n",
                  cardTypeName(t), SD.cardSize() / (1024ULL * 1024ULL),
                  SD.totalBytes() / (1024ULL * 1024ULL),
                  SD.usedBytes() / (1024ULL * 1024ULL),
                  (SD.totalBytes() - SD.usedBytes()) / (1024ULL * 1024ULL));

    /* 挂载成功 ≠ 可用：还要能打开目录（MODE1 的教训） */
    errno = 0;
    File d = SD.open(TRIPS_VFS, FILE_READ);
    if (!d) {
        g_uisd = SD_NONE;
        Serial.printf("{\"v\":2,\"ev\":\"sd-fail\",\"why\":\"dirprobe\",\"errno\":%d}\n", errno);
        return false;
    }
    d.close();
    s_ok = true;
    g_uisd = SD_MOUNTED;
    return true;
}

/* ================= 断电标记 ================= */
static void markerWrite()
{
    (void)MARKER_VFS;      /* VFS 路径给 File API 用；这里统一走 POSIX 全路径 */
    int fd = ::open(MARKER_POSIX, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        Serial.printf("{\"v\":2,\"ev\":\"marker-fail\",\"errno\":%d}\n", errno);
        return;
    }
    char body[140];
    int n = snprintf(body, sizeof(body),
                     "{\"sess\":%lu,\"path\":\"%s\",\"t\":%lu}\n",
                     (unsigned long)g_sess, s_pathVfs, (unsigned long)millis());
    size_t w = 0;
    writeAll(fd, (const uint8_t *)body, (size_t)n, &w);
    ::fsync(fd);
    ::close(fd);
}

static void markerClear()
{
    if (::unlink(MARKER_POSIX) != 0) {
        Serial.printf("{\"v\":2,\"ev\":\"marker-clear-fail\",\"errno\":%d}\n", errno);
    }
}

/* 启动时判断上次是否异常结束；不改动任何旧文件 */
void storeRecoverCheck()
{
    struct stat st;
    if (::stat(MARKER_POSIX, &st) != 0) {
        return;                      /* 没有标记 = 上次是正常结束 */
    }
    int fd = ::open(MARKER_POSIX, O_RDONLY);
    if (fd < 0) return;
    char buf[192] = {0};
    ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
    ::close(fd);
    if (n <= 0) return;

    /* 从标记里取上一次的文件路径 */
    char prevPath[48] = {0};
    const char *k = strstr(buf, "\"path\":\"");
    if (k) {
        k += 8;
        const char *e = strchr(k, '"');
        if (e && (size_t)(e - k) < sizeof(prevPath)) {
            memcpy(prevPath, k, (size_t)(e - k));
        }
    }
    char prevPosix[64] = {0};
    snprintf(prevPosix, sizeof(prevPosix), "%s%s", SD_MOUNT, prevPath);

    /* 检查上一次的文件末字节是否为换行（判断有无半行截断） */
    int truncated = -1;
    struct stat ps;
    if (prevPath[0] && ::stat(prevPosix, &ps) == 0) {
        truncated = 0;
        if (ps.st_size > 0) {
            int f2 = ::open(prevPosix, O_RDONLY);
            if (f2 >= 0) {
                if (::lseek(f2, ps.st_size - 1, SEEK_SET) == ps.st_size - 1) {
                    uint8_t last = 0;
                    if (::read(f2, &last, 1) == 1 && last != '\n') truncated = 1;
                }
                ::close(f2);
            }
        }
    }

    s_prevFound = true;
    snprintf(s_prevPath, sizeof(s_prevPath), "%s", prevPath);
    s_prevSize = prevPath[0] ? (long)ps.st_size : -1L;
    s_prevTrunc = truncated;
    Serial.printf("{\"v\":2,\"ev\":\"prev-abnormal\",\"prev\":\"%s\",\"prev_size\":%ld,"
                  "\"truncated\":%d,\"marker\":\"found\"}\n",
                  s_prevPath, s_prevSize, s_prevTrunc);
    g_uisd = SD_RECOVER;

    /* 标记清掉，避免每次都报；信息已经进新文件 */
    markerClear();
}

/* ================= 开新行程 ================= */
bool storeOpenTrip()
{
    if (!s_ok) return false;
    if (!SD.exists(TRIPS_VFS)) SD.mkdir(TRIPS_VFS);

    for (int i = 1; i <= 999; i++) {
        char vfs[40], posix[48];
        snprintf(vfs, sizeof(vfs), TRIPS_VFS "/trip-%03d.jsonl", i);
        snprintf(posix, sizeof(posix), "%s%s", SD_MOUNT, vfs);

        errno = 0;
        int fd = ::open(posix, O_WRONLY | O_CREAT | O_EXCL, 0666);
        if (fd < 0) {
            if (errno == EEXIST) continue;
            Serial.printf("{\"v\":2,\"ev\":\"sd-fail\",\"why\":\"open\",\"path\":\"%s\","
                          "\"errno\":%d}\n", vfs, errno);
            g_uisd = SD_ERROR;
            return false;
        }
        snprintf(s_pathVfs, sizeof(s_pathVfs), "%s", vfs);
        snprintf(s_pathPosix, sizeof(s_pathPosix), "%s", posix);
        s_fd = fd;
        s_recording = true;
        g_uisd = SD_REC;
        s_bufLen = 0;
        s_lines = 0;
        s_bytesW = 0;
        s_lastFlush = millis();

        mbedtls_sha256_init(&s_sha);
        mbedtls_sha256_starts(&s_sha, 0);
        s_shaActive = true;
        s_expSha[0] = 0;

        logJsonf("{\"v\":2,\"ev\":\"sd-open\",\"path\":\"%s\",\"create\":\"excl\","
                 "\"sess\":%lu}", s_pathVfs, (unsigned long)g_sess);

        /* ★ 卡里第一行就是"这一趟是谁、为什么启动、上一趟怎么结束的" */
        logJsonf("{\"v\":2,\"t\":0,\"src\":\"sys\",\"ev\":\"sess-begin\",\"sess\":%lu,"
                 "\"reset\":\"%s\",\"prev\":\"%s\",\"prev_size\":%ld,\"prev_truncated\":%d,"
                 "\"prev_marker\":%s}",
                 (unsigned long)g_sess, g_resetWhy,
                 s_prevFound ? s_prevPath : "", s_prevFound ? s_prevSize : -1L,
                 s_prevFound ? s_prevTrunc : -1,
                 s_prevFound ? "true" : "false");

        markerWrite();          /* ★ 断电标记：正常保存时才删 */
        return true;
    }
    Serial.println(F("{\"v\":2,\"ev\":\"sd-fail\",\"why\":\"no-free-name\"}"));
    return false;
}

/* ================= 结束并保存 ================= */
static bool verifySavedFile(long *outSize, uint32_t *outLines, bool *outHashOk)
{
    struct stat st;
    if (::stat(s_pathPosix, &st) != 0) {
        Serial.printf("{\"v\":2,\"ev\":\"verify-fail\",\"why\":\"stat\",\"errno\":%d}\n", errno);
        return false;
    }
    int fd = ::open(s_pathPosix, O_RDONLY);
    if (fd < 0) {
        Serial.printf("{\"v\":2,\"ev\":\"verify-fail\",\"why\":\"reopen\",\"errno\":%d}\n", errno);
        return false;
    }

    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);

    uint8_t buf[512];
    uint32_t total = 0, nl = 0;
    bool readErr = false;
    for (;;) {
        ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n < 0) { readErr = true; break; }
        if (n == 0) break;
        mbedtls_sha256_update(&c, buf, (size_t)n);
        for (ssize_t i = 0; i < n; i++) if (buf[i] == '\n') nl++;
        total += (uint32_t)n;
    }
    errno = 0;
    const int rcClose = ::close(fd);

    unsigned char dg[32];
    mbedtls_sha256_finish(&c, dg);
    mbedtls_sha256_free(&c);
    shaHex(dg, s_actSha);

    const bool sizeOk  = !readErr && (total == s_bytesW) && ((uint32_t)st.st_size == s_bytesW);
    const bool linesOk = (nl == s_lines);
    const bool hashOk  = (strcmp(s_actSha, s_expSha) == 0);
    const bool closeOk = (rcClose == 0);

    if (outSize)   *outSize   = (long)st.st_size;
    if (outLines)  *outLines  = nl;
    if (outHashOk) *outHashOk = hashOk;

    logJsonf("{\"v\":2,\"ev\":\"verify\",\"path\":\"%s\",\"size\":%u,\"expect_bytes\":%lu,"
             "\"st_size\":%ld,\"lines\":%lu,\"expect_lines\":%lu,\"sha\":\"%s\","
             "\"expect_sha\":\"%s\",\"size_ok\":%s,\"lines_ok\":%s,\"hash_ok\":%s,"
             "\"close_ok\":%s}",
                  s_pathVfs, (unsigned)total, (unsigned long)s_bytesW, (long)st.st_size,
                  (unsigned long)nl, (unsigned long)s_lines, s_actSha, s_expSha,
                  sizeOk ? "true" : "false", linesOk ? "true" : "false",
                  hashOk ? "true" : "false", closeOk ? "true" : "false");

    return sizeOk && linesOk && hashOk && closeOk;
}

void storeFinishAndSave()
{
    if (!s_recording) return;

    /* ★ 先 flush，让下面 stats 里的 bytes 就是"此刻真实落盘字节数"。
       V1 少了这一步，导致 trip-end.bytes 比文件小 111~917 字节。 */
    const bool preFlushOk = storeFlush();
    const uint32_t bytesBefore = s_bytesW;
    const uint32_t linesBefore = s_lines;

    /* trip-end 自身长度参与格式化，迭代到自洽 */
    char line[300];
    int selfLen = 0;
    for (int it = 0; it < 4; it++) {
        int n = snprintf(line, sizeof(line),
                         "{\"v\":2,\"t\":%lu,\"src\":\"sys\",\"ev\":\"trip-end\",\"org\":\"%s\","
                         "\"sess\":%lu,\"path\":\"%s\",\"lines\":%lu,\"bytes\":%lu,"
                         "\"self_len\":%d,\"write_errors\":%lu,\"lost_bytes\":%lu,"
                         "\"json_invalid\":%lu}",
                         (unsigned long)millis(), g_mode == MODE_SIM ? "sim" : "real",
                         (unsigned long)g_sess, s_pathVfs,
                         (unsigned long)linesBefore, (unsigned long)bytesBefore,
                         selfLen, (unsigned long)s_errWrites,
                         (unsigned long)s_lostBytes, (unsigned long)s_jsonBad);
        if (n == selfLen) break;
        selfLen = n;
    }
    logJson(line);

    const bool flushOk = storeFlush();

    errno = 0;
    const int rcSync = ::fsync(s_fd);
    const int syncErrno = errno;
    if (rcSync != 0) s_errWrites++;
    Serial.printf("{\"v\":2,\"ev\":\"sd-final-sync\",\"rc\":%d,\"errno\":%d}\n", rcSync, syncErrno);

    errno = 0;
    const int rcClose = ::close(s_fd);
    const int closeErrno = errno;
    s_fd = -1;
    if (rcClose != 0) s_errWrites++;
    Serial.printf("{\"v\":2,\"ev\":\"sd-final-close\",\"rc\":%d,\"errno\":%d}\n",
                  rcClose, closeErrno);

    s_recording = false;
    s_finished = true;

    if (s_shaActive) {
        unsigned char dg[32];
        mbedtls_sha256_finish(&s_sha, dg);
        mbedtls_sha256_free(&s_sha);
        s_shaActive = false;
        shaHex(dg, s_expSha);
    }

    g_uisd = SD_VERIFY;
    uiDraw(true);

    long fsize = -1;
    uint32_t flines = 0;
    bool hashOk = false;
    const bool verified = verifySavedFile(&fsize, &flines, &hashOk);

    /* 恒等式验算：bytes + self_len + 1 == 文件大小 */
    const bool acctOk = (fsize > 0) &&
                        ((long)(bytesBefore + (uint32_t)selfLen + 1) == fsize);

    if (s_errWrites == 0 && preFlushOk && flushOk && verified && acctOk) {
        g_uisd = SD_SAVED;
        markerClear();                       /* 只有到这里才算"正常结束" */
        logJsonf("{\"v\":2,\"ev\":\"saved\",\"path\":\"%s\",\"lines\":%lu,\"bytes\":%ld,"
                 "\"sha\":\"%s\",\"verified\":true,\"acct_ok\":true,\"sess\":%lu}",
                      s_pathVfs, (unsigned long)flines, fsize, s_actSha,
                      (unsigned long)g_sess);
    } else {
        g_uisd = SD_ERROR;
        logJsonf("{\"v\":2,\"ev\":\"sd-error-final\",\"path\":\"%s\",\"lines\":%lu,"
                 "\"bytes\":%ld,\"write_errors\":%lu,\"lost_bytes\":%lu,"
                 "\"flush_ok\":%s,\"verified\":%s,\"acct_ok\":%s}",
                      s_pathVfs, (unsigned long)flines, fsize,
                      (unsigned long)s_errWrites, (unsigned long)s_lostBytes,
                      flushOk ? "true" : "false", verified ? "true" : "false",
                      acctOk ? "true" : "false");
    }
    uiDraw(true);
}

/* ================= 周期落盘 ================= */
void storeTick()
{
    if (s_recording && (millis() - s_lastFlush >= FLUSH_MS)) {
        storeFlush();
    }
}

/* ================= getters ================= */
uint32_t storeLines()      { return s_lines; }
uint32_t storeBytes()      { return s_bytesW; }
uint32_t storeErrWrites()  { return s_errWrites + s_jsonBad; }
const char *storePathVfs() { return s_pathVfs; }
bool storeIsRecording()    { return s_recording; }
bool storeOk()             { return s_ok; }
