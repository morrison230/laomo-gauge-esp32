/*
 * c_obd.ino —— BLE 设备选择 / 绑定 / ELM327 握手 / PID 轮询与解码
 *
 * 针对 2026-11-12 实车审计的三条硬要求：
 *   1. 不再"扫到名字里带 OBD 就连"。候选进池 → 用户选 → 连 → 验服务 → 验真实 ECU 响应 → 才存 NVS。
 *   2. 有绑定后**只认这个地址 + 地址类型**。找不到就报找不到，绝不改连别人。
 *      地址类型必须一起存：随机地址设备下次换地址，但类型不匹配也会连不上。
 *   3. link / svc / ecu 三态分开记。连上 ≠ 认出服务 ≠ 收到有效帧。
 *
 * ★ r2（2026-11-14）本文件加的只有「观测」与「阶段派生」，通信行为未改：
 *   · 16 个阶段全部由现有状态变量派生（calcPhase()），不另存一份状态
 *   · 结果码把「已提交 / 已收到字节 / 已解析有效 / 超时 / NO DATA / SEARCHING /
 *     无法解析 / 发送失败」拆开，避免把不同故障合并成一个笼统结论
 *   · 探针结论只由握手探针那一次决定；之后的 0100 轮询只进 PID 统计，
 *     不会把「探针曾成功」翻成「超时」
 *   · 所有 logJson（写卡）都在主循环调用链里；BLE 任务与通知回调只写计数器/标志位
 *
 * ★ 关于"V1 没实现 010C/010D"的更正：
 *   V1 的 PID_LIST 里其实已经有 0100/0105/010D/010C/0104/0142。
 *   之所以日志里只有 0105/0142，是因为 SIM 模式下 obdTick() 第一行就 return，
 *   而 SIM 只合成那两个 PID；REAL 模式 BLE 又从未连上。
 *
 * ★ 只发 Mode 01（当前数据）只读查询，绝不发任何写入/控制/配置类服务。
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <freertos/semphr.h>

#include "v2.h"

/* ================= PID 表 ================= */
static PidRec s_pid[V2_PID_COUNT] = {
    /* cmd     pid   n  label     unit    scale  off    min    max   period */
    { "010C", 0x0C, 2, "RPM",    "rpm",  0.25f,  0.0f,   0.0f, 8000.f, 500 },
    { "010D", 0x0D, 1, "SPEED",  "km/h", 1.0f,   0.0f,   0.0f,  255.f, 500 },
    { "0105", 0x05, 1, "COOLANT","C",    1.0f, -40.0f, -40.0f, 150.f, 2000 },
    { "0142", 0x42, 2, "BATT",   "V",    0.001f, 0.0f,   0.0f,   20.f, 2000 },
    { "0104", 0x04, 1, "LOAD",   "%",    0.392f, 0.0f,   0.0f,  100.f, 2000 },
    { "0100", 0x00, 4, "SUPPORT","",     0.0f,   0.0f,   0.0f,    0.f, 30000 },
    { "0111", 0x11, 1, "THROTTLE","%",   100.f / 255.f, 0.f, 0.f, 100.f, 5000 },
    { "010F", 0x0F, 1, "IAT",    "C",    1.f,   -40.f, -40.f, 215.f, 5000 },
    { "0106", 0x06, 1, "STFT",   "%",    0.78125f, -100.f, -100.f, 99.21875f, 5000 },
    { "0107", 0x07, 1, "LTFT",   "%",    0.78125f, -100.f, -100.f, 99.21875f, 5000 },
    { "010E", 0x0E, 1, "TIMING", "deg",  0.5f,  -64.f, -64.f, 63.5f, 5000 },
    { "0110", 0x10, 2, "MAF",    "g/s",  0.01f,   0.f,   0.f, 655.35f, 5000 },
    { "011F", 0x1F, 2, "RUN TIME","s",   1.f,     0.f,   0.f, 65535.f, 5000 },
    { "0120", 0x20, 4, "SUPPORT","",     0.f,     0.f,   0.f, 0.f, 30000 },
    { "0140", 0x40, 4, "SUPPORT","",     0.f,     0.f,   0.f, 0.f, 30000 },
};
static constexpr int kCorePidCount = 6;
static constexpr uint32_t kExtraFreshMs = 45000;

int obdPidCount() { return V2_PID_COUNT; }
const PidRec *obdPid(int i) { return (i >= 0 && i < V2_PID_COUNT) ? &s_pid[i] : nullptr; }

/* SIM 的数值单独存放，绝不写进上面的 REAL 统计 */
static float    s_simVal[V2_PID_COUNT];
static bool     s_simHas[V2_PID_COUNT];

/* ================= 候选设备池 ================= */
static BleCand s_cand[V2_MAX_CAND];
static int     s_candN = 0;
static int     s_pick = 0;
static portMUX_TYPE s_candMux = portMUX_INITIALIZER_UNLOCKED;

/* ================= 跨任务请求标志 ================= */
static volatile bool s_reqScan = false;
static volatile int  s_reqConn = -1;
static volatile bool s_reqDisc = false;
static volatile bool s_reqRetry = false;
static volatile bool s_reqSaved = false;
static volatile bool s_autoPaused = false;    /* BLE OFF stays off until user requests a connection */
static volatile bool s_recoverLink = false;
static SemaphoreHandle_t s_clientUse = nullptr; /* BLE task retires the client, loop may be writing */
static BleCand s_retryCandidate{};
static char s_retryTarget[20] = {0};
/* ★ 主动断开（用户点了断开 / 切 SIM）不该被记成「连接失败」。
   bleTask 在断之前置位，ClientCb 据此把链路写成 IDLE 而不是 FAILED。 */
static volatile bool s_userDisc = false;

/* ================= 接收缓冲（不用 String，避免临界区里分配内存） ================= */
static char   s_rx[720];
static size_t s_rxLen = 0;
static bool s_rxOverflow = false;
static uint32_t s_rxAt = 0;
static portMUX_TYPE s_rxMux = portMUX_INITIALIZER_UNLOCKED;

/* ================= 连接态 ================= */
static BLEClient *s_client = nullptr;
static BLERemoteCharacteristic *s_chW = nullptr;
static BLERemoteCharacteristic *s_chN = nullptr;
static bool s_notifyOn = false;

/* ---- 诊断版观测（只读计数，不改变通信行为） ---- */
static volatile uint32_t s_notifyCount = 0;   /* notifyCb 触发次数 */
static volatile uint32_t s_firstByteAt  = 0;  /* 本轮首个通知字节到达的绝对时刻；0=尚无 */
static uint32_t s_firstByteMs = 0;            /* 首字节距 s_sentAt 的毫秒；0=未观测到 */
static uint32_t s_lastProbeMs = 0;            /* 最近一次 0100 探针判定耗时；0=未发生 */
static uint8_t  s_probeState  = PR_NONE;      /* ★ 探针结论（PhResult 值），只由握手探针写 */
static uint16_t s_timeoutSnaps = 0;           /* 已记录的超时片段条数（上限 TIMEOUT_SNAP_MAX） */
static char     s_wUuid[40] = {0};            /* 所选写特征 UUID */
static char     s_nUuid[40] = {0};            /* 所选通知特征 UUID */
static uint8_t  s_wProps = 0;                 /* bit0 可写 bit1 可写无响应 bit2 可读 */
static uint8_t  s_nProps = 0;                 /* bit0 可通知 bit1 可指示 bit2 可读 */
static int      s_svcCount = 0;               /* 服务数 */
/* ★ 特征清单有界副本：在 bleTask 里填，由主循环写卡（BLE 任务不写 SD） */
static char     s_charDesc[BLE_CHAR_MAX][24];
static int      s_charDescN = 0;

/* 最近一次应答的分类（0=空 1=有效 2=无数据 3=搜索中 4=无法解析） */
static uint8_t  s_lastRxCls = 0;

/* 最近一次 PID 轮询结果（屏幕「轮询」行用） */
static char     s_lastPidCmd[8] = {0};
static uint8_t  s_lastPidRes = PR_NONE;
static float    s_lastPidVal = 0.f;

static char   s_devName[33] = {0};
static char   s_devAddr[20] = {0};
static uint8_t s_devType = 0;
static int    s_devRssi = 0;
static int    s_svcChars = 0;

/* ================= 绑定（NVS） ================= */
static char    s_savedAddr[20] = {0};
static char    s_savedName[33] = {0};
static uint8_t s_savedType = 0;
static bool    s_savedBound = false;

/* ================= 请求/应答 ================= */
static bool     s_waiting = false;
static uint32_t s_sentAt = 0;
static uint32_t s_sentSub = 0;      /* 调用 writeValue 之前（提交时刻） */
static uint32_t s_lastTx = 0;
static uint32_t s_pidDue[V2_PID_COUNT] = {};
static uint32_t s_lastPromptAt = 0;
static uint32_t s_drainAt = 0;
static bool s_draining = false;
static bool s_drainSawPrompt = false;
static uint32_t s_lastOkMs = 0;
static char     s_sentCmd[8] = {0};
static bool     s_sentIsAt = false;
static int      s_pidIdx = -1;
static int      s_extraPidIdx = kCorePidCount - 1;
static uint8_t  s_coreRequestsSinceExtra = 0;

/* 握手步骤：0=ATZ 1=ATE0 2=ATH1 3=探针 0100 4=完成 */
static int  s_atStep = 0;
static bool s_ready = false;

/* ================= 阶段进度 =================
   阶段由现有状态变量派生，不另存一份，避免两处不一致。
   只在「阶段或结果发生变化」时写一条 SD 事件，不按屏幕刷新频率写。 */
static uint8_t  s_ph = PH_WAIT;
static uint32_t s_phSince = 0;
static uint8_t  s_phRes = PR_NONE;
static char     s_phNote[20] = {0};
static uint8_t  s_failPh = 0;              /* 最后失败阶段号（1..N） */
static char     s_failCn[40] = {0};        /* 最后失败（中文，屏幕用） */
static uint8_t  s_firstFailPh = 0;         /* 本次上电首个失败阶段号 */
static char     s_firstFailCn[40] = {0};
static uint32_t s_phEvents = 0;
static bool     s_ntfSeen = false;         /* 是否已写过「首次通知回调」事件 */
static bool     s_bleInfoSent = false;     /* 是否已写过本次连接的 ble-info */
static bool     s_missSent = false;        /* 是否已写过本次连接的 ble-miss */
static bool     s_regSent = false;         /* 是否已写过本次连接的 notify-reg */

/* 跨任务 → 主循环 的「待写卡」标志（BLE 任务只置位，不写文件） */
static volatile uint32_t s_regAt = 0;      /* registerForNotify 执行时刻；0=未执行 */
static volatile bool s_discPending = false;/* 发生了断开，等主循环写事件 */
static volatile uint32_t s_discStaleMs = 0;
static bool s_discLocal = false;
static uint32_t s_discAt = 0;
static int s_discReason = -1;
static portMUX_TYPE s_discMux = portMUX_INITIALIZER_UNLOCKED;

static bool     s_scanDone = false;        /* 扫描结束且候选>0 */
static int      s_scanN = 0;
static uint32_t s_linkAt = 0;              /* AT 稳定窗口终点 */
static bool     s_atBegun = false;         /* 是否已发出第一条 AT */
static uint8_t  s_atRes[3] = {0, 0, 0};    /* ATZ/ATE0/ATH1: 0 none 1 ok 2 timeout 3 sendfail */
static bool     s_bindWant = false;        /* 探针通过、等待写绑定 */
static bool     s_bindDoing = false;       /* 正在写绑定（阶段 PH_BIND） */
static uint32_t s_bindAt = 0;              /* 写绑定的最早时刻（显示停留用） */

/* ================= 连接步骤（bleTask 发布 → 主循环只读） =================
   ★ 为什么要有它：BLEClient::connect() 返回时只有「链路建立」，
     服务/特征的搜索发生在随后的 getServices() 里。上一版把两者都压在
     LINK_CONNECTING 一个变量上，结果「连接成功」「服务发现中」「服务缺失」
     三件事在计算阶段时被同一个分支吃掉，屏幕上永远看不到。
   ★ 每一步发布后停留 CONN_DWELL_MS：只为了让屏幕看得到、并保证主循环
     至少观测到一次；不参与任何逻辑判断。
   ★ 同时把每一步投递进一个小队列，由主循环写成 conn 事件 ——
     即使主循环当时在写 SD 而错过了这一帧，事件序列也不会缺。 */
enum ConnStep : uint8_t {
    CS_IDLE = 0,      /* 未连接 / 已断开 */
    CS_CONNECTING,    /* client->connect() 阻塞中 */
    CS_CONNECTED,     /* 链路已建立，尚未开始 GATT 发现 */
    CS_DISCOVER,      /* getServices() 阻塞中（真正的服务发现） */
    CS_CHARS_OK,      /* 可写 + 可通知特征都拿到 */
    CS_CHARS_MISS,    /* 服务或读写特征缺失 */
    CS_NOTIFY_DONE,   /* 通知注册调用已执行（不代表 CCCD 写成功） */
    CS_N
};
static const char *kCsStr[CS_N] = {
    "idle", "connecting", "connected", "discover", "chars_ok", "chars_miss", "notify_done"
};

/* ================= 连接步骤事件队列 =================
   ★ 队列条目在**入队时**就把「步骤 / 连接代号 / 地址 / 时刻」全部快照下来。
     早期版本让主循环写卡时再去读 s_devAddr，结果：
       · 首次连接：CONNECTING/CONNECTED/DISCOVER 被取出时 s_devAddr 还是空串；
       · 断开后重连另一台：这几条会沿用上一台设备的地址。
     两者都是「事件归属错乱」，比丢事件更难发现，所以必须在入队时锁定。
   ★ 容量 24（可用 23，环形丢一格判空）。单次连接尝试最多入队 6 条
     （connectAddr 内 5 条 + 主循环补 1 条 CS_IDLE），见报告第三节核算；
     23/6 ≈ 3.8 次尝试的余量。
   ★ 队列满时**不静默丢**：累计 s_connDrop、记被丢步骤位图 s_connLostMask，
     由主循环写成 conn-overflow 事件。 */
#define CONN_Q_N 24
static volatile uint8_t  s_connStep = CS_IDLE;
static volatile uint8_t  s_connGen  = 0;      /* 每次连接尝试/断开 +1，用于判断步骤是否过期 */
#if defined(CONFIG_BLUEDROID_ENABLED)
static esp_gatt_if_t s_gattIf = ESP_GATT_IF_NONE;
static volatile uint16_t s_gattConn = UINT16_MAX;
/* The Arduino callback omits the IDF reason. The custom hook sees the same event afterwards. */
static void captureDisconnectReason(esp_gattc_cb_event_t ev, esp_gatt_if_t iface,
                                    esp_ble_gattc_cb_param_t *param)
{
    if (ev != ESP_GATTC_DISCONNECT_EVT) return;
    portENTER_CRITICAL(&s_discMux);
    if (iface == s_gattIf && param->disconnect.conn_id == s_gattConn) {
        s_discReason = param->disconnect.reason;
        s_discPending = true;
        s_gattIf = ESP_GATT_IF_NONE;
        s_gattConn = UINT16_MAX;
    }
    portEXIT_CRITICAL(&s_discMux);
}
#endif

struct ConnEv {
    uint8_t  step;
    uint8_t  gen;
    uint32_t at;
    char     addr[20];                        /* 入队时快照，写卡时不再读 s_devAddr */
};
static ConnEv s_connQ[CONN_Q_N];
static volatile uint8_t  s_connQH = 0, s_connQT = 0;
static volatile uint16_t s_connDrop = 0;      /* 累计被丢弃的事件条数 */
static volatile uint8_t  s_connLostMask = 0;  /* 被丢步骤位图（bit = ConnStep） */
static uint16_t s_connDropSeen = 0;           /* 主循环已上报到的条数（仅主循环用） */
static portMUX_TYPE s_connMux = portMUX_INITIALIZER_UNLOCKED;

static bool s_charsReady = false;             /* 锁存：主循环据此写 ble-info，不怕错过窗口 */
static bool s_missPending = false;            /* 锁存：主循环据此写 ble-miss */
static volatile bool s_obdProfileFound = false;/* 本次连接是否发现 FFF0 服务 */

/* 发布一个连接步骤，并锁定它属于哪一次尝试、哪一台设备。
   可在 bleTask 或主循环调用；只写状态与队列，不写 SD。 */
static void pubStepEx(uint8_t st, const char *addr)
{
    char a[20];
    snprintf(a, sizeof(a), "%s", addr ? addr : "");   /* 字符串在临界区外拼好，缩短关中断时间 */

    taskENTER_CRITICAL(&s_connMux);
    const uint8_t h  = s_connQH;
    const uint8_t nh = (uint8_t)((h + 1) % CONN_Q_N);
    if (nh != s_connQT) {
        ConnEv &e = s_connQ[h];
        e.step = st;
        e.gen  = s_connGen;                 /* 记录本次尝试的代号，事后可把同一轮的事件分组 */
        e.at   = millis();
        memcpy(e.addr, a, sizeof(e.addr));
        s_connQH = nh;
    } else {
        /* 队列满：计数 + 记位图，绝不静默丢 */
        if (s_connDrop < 0xFFFF) s_connDrop = (uint16_t)(s_connDrop + 1);
        if (st < 8) s_connLostMask = (uint8_t)(s_connLostMask | (uint8_t)(1u << st));
    }
    s_connStep = st;                        /* 屏幕用的「当前步骤」无论如何都更新 */
    taskEXIT_CRITICAL(&s_connMux);
}

/* 没有更具体地址可用时（如断开、主动断开）退回「最后已知设备」 */
static void pubStep(uint8_t st) { pubStepEx(st, s_devAddr); }

/* 代号：读改写都要在临界区里做，否则 bleTask 与断开回调并发时可能丢一次自增 */
static uint8_t connGenNext()
{
    taskENTER_CRITICAL(&s_connMux);
    const uint8_t g = (uint8_t)(s_connGen + 1);
    s_connGen = g;
    taskEXIT_CRITICAL(&s_connMux);
    return g;
}
static void connGenBump()
{
    taskENTER_CRITICAL(&s_connMux);
    s_connGen = (uint8_t)(s_connGen + 1);
    taskEXIT_CRITICAL(&s_connMux);
}

static const char *kPhName[PH_N] = {
    "等待",   "扫描中", "已找到", "连接中", "连接失败", "已连接", "服务发现", "服务就绪",
    "服务缺失", "通知注册", "ATZ", "ATE0", "ATH1", "0100探针", "写绑定", "轮询", "就绪"
};
static const char *kPhCode[PH_N] = {
    "wait", "scan", "found", "conn", "conn_fail", "conn_ok", "svc", "svc_ok",
    "svc_miss", "notify", "atz", "ate0", "ath1", "probe", "bind", "poll", "ready"
};
/* 结果码：英文码进 SD，中文进屏幕。★「已发送」绝不写成「成功」 */
static const char *kPhResStr[PR_N] = {
    "none", "sent", "recv", "ok", "timeout", "nodata", "search", "unparse", "sendfail"
};
static const char *kPhResCn[PR_N]  = {
    "无结论", "已发送", "已收到", "成功", "超时", "无数据", "搜索中", "无法解析", "发送失败"
};

/* 最近 STALE_MS 内是否有过有效帧（「就绪」与「轮询」的分界） */
static bool dataFresh()
{
    return s_lastOkMs != 0 && (millis() - s_lastOkMs) < STALE_MS;
}

/* 由「连接步骤 + 握手变量」派生当前阶段
   ★ 判断顺序是按「真实控制流」排的，不是按编号排的：
     ① 链路已断（含服务缺失后的主动断开、被对端断开）先判掉，
        否则 s_connStep 还停在 discover 会让阶段永远卡在「服务发现」；
     ② 扫描中；
     ③ 连接步骤（connecting / connected / discover / chars_ok / chars_miss）；
     ④ 尚未连上 → 已找到 / 等待；
     ⑤ 握手与轮询。
   每一分支对应的状态都由某个源码事件写入，逐条可达性见交付报告。 */
static uint8_t calcPhase()
{
    if (g_mode != MODE_REAL) return PH_WAIT;

    const uint8_t cs = s_connStep;

    if (g_link == LINK_FAILED) return PH_CONN_FAIL;
    if (g_link == LINK_SCANNING) return PH_SCAN;

    switch (cs) {
    case CS_CONNECTING: return PH_CONN;      /* client->connect() 阻塞中 */
    case CS_CONNECTED:  return PH_CONN_OK;   /* 链路已建立、尚未发现服务 */
    case CS_DISCOVER:   return PH_SVC;       /* getServices() 阻塞中 */
    case CS_CHARS_OK:   return PH_SVC_OK;    /* 可写+可通知特征都拿到 */
    case CS_CHARS_MISS: return PH_SVC_MISS;  /* 特征缺失（停留后才断开） */
    default: break;                          /* CS_IDLE / CS_NOTIFY_DONE 走下面 */
    }

    if (g_link != LINK_CONNECTED) return s_scanDone ? PH_FOUND : PH_WAIT;
    if (g_svc != SVC_FOUND)       return PH_SVC;      /* 兜底，正常不会走到 */

    if (!s_atBegun)                return PH_NOTIFY;  /* 注册已执行、等 AT 稳定窗口 */
    if (s_atStep == 0)             return PH_ATZ;
    if (s_atStep == 1)             return PH_ATE0;
    if (s_atStep == 2)             return PH_ATH1;
    if (s_atStep == 3)             return PH_PROBE;
    if (s_bindWant || s_bindDoing) return PH_BIND;
    /* 握手完成后的两个稳态：
       就绪 = 探针通过 **且** 最近仍在收到有效帧（数据面确认）
       轮询 = 其余情况（探针未通过，或已 5s 没有有效帧）—— 仍在轮询，但数据面未确认 */
    return (s_ready && dataFresh()) ? PH_READY : PH_POLL;
}

/* 是否属于「失败」类结果 */
static bool resIsFail(uint8_t r)
{
    return r == PR_TIMEOUT || r == PR_NODATA || r == PR_SEARCH ||
           r == PR_BADFRM || r == PR_SENDFAIL;
}

/* 阶段事件：只在「阶段或结果变化」时调用 */
static void emitStage()
{
    char line[220];
    snprintf(line, sizeof(line),
             "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"stage\",\"sess\":%lu,"
             "\"ph\":%u,\"of\":%u,\"code\":\"%s\",\"st\":\"%s\",\"ms\":%lu,\"note\":\"%s\"}",
             (unsigned long)millis(), (unsigned long)g_sess,
             (unsigned)(s_ph + 1), (unsigned)PH_N, kPhCode[s_ph],
             kPhResStr[(s_phRes < PR_N) ? s_phRes : 0],
             (unsigned long)(millis() - s_phSince), s_phNote);
    logJson(line);
    s_phEvents++;

    /* ★ 只重画阶段区，不整屏重画。
       本函数只在主循环调用链（stageTick / stageResult / 写绑定）里执行，
       不会在 BLE 回调或 bleTask 中调用。 */
    uiPhasePaint();
}

/* 阶段结果变化（sent/recv/ok/timeout/nodata/search/unparse/sendfail）→ 写一条
   ★ PH_POLL 阶段不写 stage 事件（轮询每 0.2~2 秒一次，靠 rx/value/timeout 事件留证就够），
     但屏幕上的结果照常刷新。 */
static void stageResult(uint8_t res, const char *note)
{
    if (res >= PR_N) res = PR_NONE;
    const char *nt = note ? note : "";
    if (res == s_phRes && !strcmp(nt, s_phNote)) return;   /* 完全没变：不写不画 */

    s_phRes = (uint8_t)res;
    snprintf(s_phNote, sizeof(s_phNote), "%s", nt);

    /* 记「最后失败」与「首个失败」——失败后不会被笼统的"未连接"覆盖 */
    if (resIsFail(res)) {
        s_failPh = (uint8_t)(s_ph + 1);
        snprintf(s_failCn, sizeof(s_failCn), "%s%s%s%s",
                 kPhName[s_ph], kPhResCn[res], nt[0] ? " " : "", nt);
        if (!s_firstFailPh) {
            s_firstFailPh = s_failPh;
            snprintf(s_firstFailCn, sizeof(s_firstFailCn), "%s", s_failCn);
        }
    }

    if (s_ph == PH_POLL) { uiPhasePaint(); return; }   /* 轮询：只刷新屏幕，不写卡 */
    emitStage();
}

/* 取出 bleTask 投递的连接步骤，逐条写成 conn 事件。
   ★ 地址与代号取自**入队时的快照**，不是此刻的 s_devAddr ——
     这样首次连接不会写成空地址、重连另一台也不会沿用上一台的地址。
   ★ 溢出单独报：conn-overflow 带累计条数与被丢步骤位图，避免"事件序列看着完整"。
   写卡只在这里（主循环），BLE 任务与回调都不碰文件。 */
static void drainConnQ()
{
    for (;;) {
        uint8_t step = 0, gen = 0;
        uint32_t at = 0;
        char addr[20] = {0};

        taskENTER_CRITICAL(&s_connMux);
        if (s_connQT == s_connQH) { taskEXIT_CRITICAL(&s_connMux); break; }
        const ConnEv &e = s_connQ[s_connQT];
        step = e.step;
        gen  = e.gen;
        at   = e.at;
        memcpy(addr, e.addr, sizeof(addr));
        s_connQT = (uint8_t)((s_connQT + 1) % CONN_Q_N);
        taskEXIT_CRITICAL(&s_connMux);

        char line[240];
        snprintf(line, sizeof(line),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"conn\",\"sess\":%lu,"
                 "\"step\":\"%s\",\"gen\":%u,\"at\":%lu,\"lag\":%lu,\"addr\":\"%s\"}",
                 (unsigned long)millis(), (unsigned long)g_sess,
                 (step < CS_N) ? kCsStr[step] : "?", (unsigned)gen, (unsigned long)at,
                 (unsigned long)(millis() - at), addr);
        logJson(line);
    }

    /* 溢出上报：只在累计值变化时写一条 */
    uint16_t drop = 0;
    uint8_t mask = 0;
    taskENTER_CRITICAL(&s_connMux);
    drop = s_connDrop;
    mask = s_connLostMask;
    taskEXIT_CRITICAL(&s_connMux);

    if (drop != s_connDropSeen) {
        s_connDropSeen = drop;
        char lost[72];
        int o = 0;
        lost[0] = 0;
        for (int i = 0; i < CS_N && o < (int)sizeof(lost) - 12; i++) {
            if (mask & (uint8_t)(1u << i)) {
                o += snprintf(lost + o, sizeof(lost) - (size_t)o, "%s%s", o ? "," : "", kCsStr[i]);
            }
        }
        char line[220];
        snprintf(line, sizeof(line),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"conn-overflow\",\"sess\":%lu,"
                 "\"drop_total\":%u,\"lost_mask\":%u,\"lost\":\"%s\",\"q_n\":%d}",
                 (unsigned long)millis(), (unsigned long)g_sess,
                 (unsigned)drop, (unsigned)mask, lost, CONN_Q_N);
        logJson(line);
    }
}

/* 每轮主循环调用：阶段变化检测 + 跨任务标志落卡（回调内不写 SD） */
static void stageTick()
{
    drainConnQ();      /* ① 连接步骤：事件级留证（不依赖屏幕） */

    const uint8_t ph = calcPhase();
    if (ph != s_ph) {
        s_ph = ph;
        s_phSince = millis();
        s_phRes = PR_NONE;
        s_phNote[0] = 0;
        emitStage();
    }

    /* ---- 特征就绪 → 写一条 ble-info（含所选特征属性） ----
       ★ 用锁存的 s_charsReady，不用「当前阶段==SVC_OK」：主循环若错过那 300ms 窗口，
         阶段会跳过，但这条记录不能丢。 */
    if (s_charsReady && !s_bleInfoSent) {
        s_bleInfoSent = true;
        /* Full UUIDs are 36 characters each. The old 300-byte buffer truncated
           this JSON line, so jsonValidate() rejected it and SD lost w_uuid/n_uuid. */
        char bi[420];
        snprintf(bi, sizeof(bi),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"ble-info\",\"sess\":%lu,"
                 "\"addr\":\"%s\",\"name\":\"%s\",\"atype\":%u,\"svc_n\":%d,\"chars\":%d,"
                 "\"profile\":\"fff0\",\"w_uuid\":\"%s\",\"w_props\":\"%02X\",\"n_uuid\":\"%s\",\"n_props\":\"%02X\","
                 "\"notify_reg\":%d,\"reg_ok\":\"UNKNOWN\",\"cccd\":\"UNKNOWN\"}",
                 (unsigned long)millis(), (unsigned long)g_sess,
                 s_devAddr, s_devName, (unsigned)s_devType, s_svcCount, s_svcChars,
                 s_wUuid[0] ? s_wUuid : "-", (unsigned)s_wProps,
                 s_nUuid[0] ? s_nUuid : "-", (unsigned)s_nProps,
                 s_notifyOn ? 1 : 0);
        logJson(bi);

        /* ---- 特征清单（有界 BLE_CHAR_MAX 条） ---- */
        if (s_charDescN > 0) {
            char cl[330];
            int o = snprintf(cl, sizeof(cl),
                             "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"ble-chars\",\"sess\":%lu,"
                             "\"n\":%d,\"list\":[",
                             (unsigned long)millis(), (unsigned long)g_sess, s_charDescN);
            for (int i = 0; i < s_charDescN; i++) {
                /* ★ 逐条判余量，永不越界：写不下就提前收尾 */
                if (o < 0 || o > (int)sizeof(cl) - 30) break;
                o += snprintf(cl + o, sizeof(cl) - (size_t)o, "%s\"%s\"",
                              i ? "," : "", s_charDesc[i]);
            }
            if (o < 0) o = 0;
            if (o > (int)sizeof(cl) - 4) o = (int)sizeof(cl) - 4;
            snprintf(cl + o, sizeof(cl) - (size_t)o, "]}");
            logJson(cl);
        }
    }

    /* ---- 服务缺失 → 写一条 ble-miss（含已发现数量，便于判断是否 UUID 不符） ---- */
    if (s_missPending && !s_missSent) {
        s_missSent = true;
        char bm[220];
        snprintf(bm, sizeof(bm),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"ble-miss\",\"sess\":%lu,"
                 "\"addr\":\"%s\",\"svc_n\":%d,\"chars\":%d,\"expected\":\"fff0/fff2/fff1\",\"profile_found\":%s,\"why\":\"profile-or-char-missing\"}",
                 (unsigned long)millis(), (unsigned long)g_sess,
                 s_devAddr, s_svcCount, s_svcChars, s_obdProfileFound ? "true" : "false");
        logJson(bm);
    }

    /* ---- 通知注册调用已执行（由 bleTask 置时刻，这里落卡） ---- */
    if (!s_regSent && s_regAt != 0) {
        s_regSent = true;
        char rg[240];
        snprintf(rg, sizeof(rg),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"notify-reg\",\"sess\":%lu,"
                 "\"at\":%lu,\"call\":\"executed\",\"reg_ok\":\"UNKNOWN\",\"cccd\":\"UNKNOWN\","
                 "\"n_uuid\":\"%s\",\"n_props\":\"%02X\"}",
                 (unsigned long)millis(), (unsigned long)g_sess, (unsigned long)s_regAt,
                 s_nUuid[0] ? s_nUuid : "-", (unsigned)s_nProps);
        logJson(rg);
    }

    /* ---- 断开（由 BLE 回调置位）→ 主循环落卡，并保留失败原因 ---- */
    uint32_t discAt, discAge;
    bool discLocal, discPending;
    int discReason;
    portENTER_CRITICAL(&s_discMux);
    discPending = s_discPending;
    s_discPending = false;
    discAt = s_discAt;
    discAge = s_discStaleMs;
    discLocal = s_discLocal;
    discReason = s_discReason;
    portEXIT_CRITICAL(&s_discMux);
    if (discPending) {
        char reason[16] = "null";
        if (discReason >= 0) snprintf(reason, sizeof(reason), "%d", discReason);
        char dc[260];
        snprintf(dc, sizeof(dc),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"ble-disc\",\"sess\":%lu,"
                 "\"at\":%lu,\"saved\":%d,\"stale_ms\":%lu,\"local\":%s,\"reason_code\":%s,\"why\":\"%s\"}",
                 (unsigned long)millis(), (unsigned long)g_sess,
                 (unsigned long)discAt, s_savedBound ? 1 : 0, (unsigned long)discAge,
                 discLocal ? "true" : "false", reason, discReason >= 0 ? "stack" : "UNKNOWN");
        logJson(dc);
    }

    /* ---- 首次观测到通知回调 ---- */
    if (!s_ntfSeen && s_notifyCount > 0) {
        s_ntfSeen = true;
        char line[200];
        snprintf(line, sizeof(line),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"notify-first\",\"sess\":%lu,"
                 "\"n\":%lu,\"fb\":%lu,\"cccd\":\"UNKNOWN\"}",
                 (unsigned long)millis(), (unsigned long)g_sess,
                 (unsigned long)s_notifyCount, (unsigned long)s_firstByteMs);
        logJson(line);
    }
}

/* ================= 字符串工具 ================= */
const char *obdLinkStr()
{
    switch (g_link) {
    case LINK_IDLE:       return "idle";
    case LINK_SCANNING:   return "scanning";
    case LINK_CONNECTING: return "connecting";
    case LINK_CONNECTED:  return "connected";
    default:              return "failed";
    }
}
const char *obdSvcStr()
{
    return (g_svc == SVC_FOUND) ? "found" : (g_svc == SVC_MISSING ? "missing" : "unknown");
}
const char *obdEcuStr()
{
    return (g_ecu == ECU_VALID) ? "valid" : (g_ecu == ECU_NODATA ? "nodata" : "none");
}

bool obdSavedBound() { return s_savedBound; }
uint32_t obdStaleMs()
{
    if (!s_lastOkMs) return 999999UL;
    return millis() - s_lastOkMs;
}

int obdCandCount()
{
    taskENTER_CRITICAL(&s_candMux);
    const int n = s_candN;
    taskEXIT_CRITICAL(&s_candMux);
    return n;
}

const BleCand *obdCand(int i)
{
    if (i < 0 || i >= V2_MAX_CAND) return nullptr;
    return &s_cand[i];          /* 只读访问，写入在扫描回调里加锁 */
}

int obdPickIdx() { return s_pick; }

void obdPickToIndex(int i)
{
    const int n = obdCandCount();
    if (n <= 0) { s_pick = 0; return; }
    if (i < 0) i = 0;
    if (i >= n) i = n - 1;
    s_pick = i;
}

void obdPickNext()
{
    const int n = obdCandCount();
    if (n <= 0) { s_pick = 0; return; }
    s_pick = (s_pick + 1) % n;
}

/* ================= NVS 绑定 ================= */
static void loadBind()
{
    Preferences p;
    p.begin("sx4", true);
    String a = p.getString("addr", "");
    String nm = p.getString("name", "");
    const uint8_t t = (uint8_t)p.getUChar("atype", 0);
    p.end();
    if (a.length() >= 11) {
        snprintf(s_savedAddr, sizeof(s_savedAddr), "%s", a.c_str());
        snprintf(s_savedName, sizeof(s_savedName), "%s", nm.c_str());
        s_savedType = t;
        s_savedBound = true;
    } else {
        s_savedBound = false;
        s_savedAddr[0] = 0;
    }
}

static bool saveBind(const char *addr, uint8_t type, const char *name)
{
    Preferences p;
    if (!p.begin("sx4", false)) return false;
    const bool ok = p.putUChar("atype", type) == sizeof(type) &&
                    p.putString("addr", addr) == strlen(addr);
    if (ok) p.putString("name", name);         /* Optional label, not part of peer identity. */
    p.end();
    if (!ok) return false;
    taskENTER_CRITICAL(&s_candMux);
    snprintf(s_savedAddr, sizeof(s_savedAddr), "%s", addr);
    snprintf(s_savedName, sizeof(s_savedName), "%s", name);
    s_savedType = type;
    s_savedBound = true;
    taskEXIT_CRITICAL(&s_candMux);
    return true;
}

void obdForget()
{
    Preferences p;
    p.begin("sx4", false);
    p.remove("addr"); p.remove("atype"); p.remove("name");
    p.end();
    taskENTER_CRITICAL(&s_candMux);
    s_savedBound = false;
    s_savedAddr[0] = 0;
    s_savedName[0] = 0;
    s_retryTarget[0] = 0;
    s_reqRetry = false;
    s_autoPaused = true;
    taskEXIT_CRITICAL(&s_candMux);
    Serial.println(F("{\"v\":2,\"ev\":\"bind-cleared\"}"));
}

/* ================= 扫描回调 ================= */
class ScanCb : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override
    {
        if (g_mode == MODE_SIM) return;

        const String addr = String(dev.getAddress().toString().c_str());
        if (!addr.length()) return;

        bool hasFfe0 = false;
        const int nSvc = dev.getServiceUUIDCount();
        for (int i = 0; i < nSvc; i++) {
            const String u = String(dev.getServiceUUID(i).toString().c_str());
            if (u.indexOf("ffe0") >= 0 || u.indexOf("FFE0") >= 0) { hasFfe0 = true; break; }
        }
        const String nm = dev.haveName() ? String(dev.getName().c_str()) : String("");

        taskENTER_CRITICAL(&s_candMux);
        /* Capture a fresh match even if nearby devices already filled the display list. */
        if (s_retryTarget[0] && addr.equalsIgnoreCase(s_retryTarget)) {
            s_retryCandidate.used = true;
            snprintf(s_retryCandidate.addr, sizeof(s_retryCandidate.addr), "%s", addr.c_str());
            if (nm.length()) snprintf(s_retryCandidate.name, sizeof(s_retryCandidate.name), "%s", nm.c_str());
            s_retryCandidate.atype = dev.getAddressType();
        }
        /* 已有同地址 → 只更新 RSSI，不重复占位 */
        int slot = -1;
        for (int i = 0; i < s_candN; i++) {
            if (addr.equals(s_cand[i].addr)) { slot = i; break; }
        }
        if (slot < 0) {
            if (s_candN < V2_MAX_CAND) slot = s_candN++;
        }
        if (slot >= 0) {
            BleCand &c = s_cand[slot];
            c.used = true;
            snprintf(c.addr, sizeof(c.addr), "%s", addr.c_str());
            c.atype = dev.getAddressType();
            if (nm.length()) snprintf(c.name, sizeof(c.name), "%s", nm.c_str());
            c.rssi = (int8_t)dev.getRSSI();
            c.hasFfe0 = hasFfe0;
            c.lastMs = millis();
        }
        taskEXIT_CRITICAL(&s_candMux);
    }
};

/* ================= 通知回调 =================
   ★ 这里只做三件事：计数、记首字节时刻、往接收缓冲追加字节。
   绝不写 SD、绝不调用 logJson、绝不分配内存（本函数在 BLE 任务上下文里跑）。 */
static void notifyCb(BLERemoteCharacteristic *ch, uint8_t *data, size_t len, bool isNotify)
{
    (void)isNotify;
    if (!len || ch != s_chN) return;
    portENTER_CRITICAL(&s_rxMux);
    s_notifyCount = s_notifyCount + 1;
    s_rxAt = millis();
    if (s_firstByteAt == 0) s_firstByteAt = s_rxAt;
    const size_t avail = sizeof(s_rx) - 1 - s_rxLen;
    const size_t copy = len < avail ? len : avail;
    memcpy(s_rx + s_rxLen, data, copy);
    s_rxLen += copy;
    s_rxOverflow |= copy != len;
    s_rx[s_rxLen] = 0;
    portEXIT_CRITICAL(&s_rxMux);
}

static void rxReset()
{
    portENTER_CRITICAL(&s_rxMux);
    s_rxLen = 0;
    s_rx[0] = 0;
    s_rxOverflow = false;
    s_rxAt = 0;
    s_firstByteAt = 0;
    portEXIT_CRITICAL(&s_rxMux);
}

/* 把接收片段转成 JSON 安全字符串（\r\n 引号 反斜杠 转义，其余不可打印替换为 '.'） */
static void snapToJson(const char *in, size_t n, char *out, size_t cap)
{
    size_t o = 0;
    for (size_t i = 0; i < n && o + 3 < cap; i++) {
        const uint8_t c = (uint8_t)in[i];
        if (c == '\r')      { out[o++] = '\\'; out[o++] = 'r'; }
        else if (c == '\n') { out[o++] = '\\'; out[o++] = 'n'; }
        else if (c == '"')   { out[o++] = '\\'; out[o++] = '"'; }
        else if (c == '\\') { out[o++] = '\\'; out[o++] = '\\'; }
        else if (c >= 0x20 && c < 0x7f) out[o++] = (char)c;
        else out[o++] = '.';
    }
    out[o] = 0;
}

/* Consume one complete prompt-delimited reply; leave subsequent bytes for the drain. */
static size_t takeReply(char *out, size_t cap)
{
    portENTER_CRITICAL(&s_rxMux);
    const char *end = s_rxOverflow ? nullptr : (const char *)memchr(s_rx, '>', s_rxLen);
    const size_t n = end ? (size_t)(end - s_rx) + 1 : 0;
    if (n && n < cap) {
        memcpy(out, s_rx, n);
        out[n] = 0;
        s_rxLen -= n;
        memmove(s_rx, s_rx + n, s_rxLen + 1);
    }
    portEXIT_CRITICAL(&s_rxMux);
    return n < cap ? n : 0;
}

int obdRxLen()
{
    portENTER_CRITICAL(&s_rxMux);
    const int n = (int)s_rxLen;
    portEXIT_CRITICAL(&s_rxMux);
    return n;
}

/* ================= 客户端回调 =================
   ★ 这里也不写 SD：只置位 s_discPending，由主循环写 ble-disc 事件。 */
class ClientCb : public BLEClientCallbacks {
    void onConnect(BLEClient *c) override
    {
#if defined(CONFIG_BLUEDROID_ENABLED)
        portENTER_CRITICAL(&s_discMux);
        s_gattIf = c->getGattcIf();
        s_gattConn = c->getConnId();
        portEXIT_CRITICAL(&s_discMux);
#else
        (void)c;
#endif
    }
    void onDisconnect(BLEClient *c) override
    {
        if (c != s_client) return;
        connGenBump();          /* 让正在进行的 connectAddr 立刻作废，别把旧状态写回来 */
        s_chW = nullptr;
        s_chN = nullptr;
        s_notifyOn = false;
        s_ready = false;
        s_atStep = 0;
        s_atBegun = false;
        s_waiting = false;
        s_charsReady = false;
        g_link = s_userDisc ? LINK_IDLE : LINK_FAILED;   /* 主动断开 ≠ 连接失败 */
        g_ecu = ECU_NONE;
        portENTER_CRITICAL(&s_discMux);
        s_discStaleMs = obdStaleMs();
        s_discAt = millis();
        s_discLocal = s_userDisc;
        s_discReason = -1;
#if defined(CONFIG_BLUEDROID_ENABLED)
        if (s_gattConn == UINT16_MAX) s_discPending = true;
#else
        s_discPending = true;
#endif
        portEXIT_CRITICAL(&s_discMux);
        pubStep(CS_IDLE);
        if (!s_userDisc && !s_autoPaused) s_reqRetry = true;
    }
};
static ClientCb s_clientCallbacks;

static void releaseClient()
{
    xSemaphoreTake(s_clientUse, portMAX_DELAY);
    BLEClient *client = s_client;
    if (client) {
        if (client->isConnected()) {
            s_userDisc = true;
            client->disconnect();
            const uint32_t at = millis();
            while (client->isConnected() && millis() - at < 1500) vTaskDelay(pdMS_TO_TICKS(20));
        }
#if defined(CONFIG_BLUEDROID_ENABLED)
        const uint16_t conn = s_gattConn;
        const uint32_t at = millis();
        while (conn != UINT16_MAX && s_gattConn == conn && millis() - at < 200)
            vTaskDelay(pdMS_TO_TICKS(10));
#endif
        s_client = nullptr;
        s_chW = s_chN = nullptr;
        delete client;
        s_userDisc = false;
    }
    xSemaphoreGive(s_clientUse);
}

/* ================= 连接 ================= */
/* 128 位基础 UUID 前 8 位里取后 4 位（0000fff1-… → fff1）；已是短 UUID 则原样 */
static void uuidShort(const char *u, char *out, size_t n)
{
    const size_t len = strlen(u);
    const char *dash = strchr(u, '-');
    if (len == 4) { snprintf(out, n, "%s", u); return; }
    if (dash && (dash - u) == 8) {
        memcpy(out, dash - 4, 4);
        out[4] = 0;
        return;
    }
    snprintf(out, n, "%s", u);
}

static void charDescAdd(const char *uuid, bool w, bool wnr, bool nt, bool ind, bool rd)
{
    if (s_charDescN >= BLE_CHAR_MAX) return;
    char sh[8];
    uuidShort(uuid, sh, sizeof(sh));
    char pf[6];
    int k = 0;
    if (w)   pf[k++] = 'W';
    if (wnr) pf[k++] = 'w';
    if (nt)  pf[k++] = 'N';
    if (ind) pf[k++] = 'I';
    if (rd)  pf[k++] = 'R';
    pf[k] = 0;
    if (!k) snprintf(pf, sizeof(pf), "-");
    snprintf(s_charDesc[s_charDescN], sizeof(s_charDesc[0]), "%s:%s", sh, pf);
    s_charDescN++;
}

static bool connectAddr(const char *addr, uint8_t atype, const char *name)
{
    releaseClient();

    /* ② 代号：本次尝试的凭据。真断开会让它 +1，之后任何一步发现代号变了就立刻收手。 */
    const uint8_t gen = connGenNext();

    g_link = LINK_CONNECTING;
    g_svc = SVC_UNKNOWN;
    g_ecu = ECU_NONE;
    s_charsReady = false;
    s_missPending = false;
    pubStepEx(CS_CONNECTING, addr);
    Serial.printf("{\"v\":2,\"ev\":\"ble-connect\",\"addr\":\"%s\",\"atype\":%u,\"name\":\"%s\"}\n",
                  addr, (unsigned)atype, name ? name : "");

    s_client = BLEDevice::createClient();
    s_client->setClientCallbacks(&s_clientCallbacks);

    const BLEAddress a(String(addr), atype);
    if (!s_client->connect(a)) {
        Serial.printf("{\"v\":2,\"ev\":\"ble-connect-fail\",\"addr\":\"%s\"}\n", addr);
        releaseClient();
        g_link = LINK_FAILED;
        pubStepEx(CS_IDLE, addr);
        return false;
    }

    /* ---- ① 链路已建立（connect() 返回 true = OPEN 事件到手）----
       ★ 注意：服务/特征此时还没查，它们由下面的 getServices() 触发搜索。
       这两件事在这里被拆成两个可观测步骤，上一版把它们混成了一个。 */
    if (gen != s_connGen) { releaseClient(); return false; }
    g_link = LINK_CONNECTED;
    pubStepEx(CS_CONNECTED, addr);
    s_client->setMTU(517);
    vTaskDelay(pdMS_TO_TICKS(CONN_DWELL_MS));      /* 只为让「已连接」在屏上停留 */

    /* ---- ② GATT 服务/特征发现（getServices 内部发起搜索并等完成）---- */
    if (gen != s_connGen) { releaseClient(); return false; }
    g_svc = SVC_UNKNOWN;
    pubStepEx(CS_DISCOVER, addr);

    /* ---- 逐服务逐特征打印，日后适配器 UUID 不明时有据可查 ---- */
    const BLEUUID obdServiceUuid((uint16_t)0xFFF0);
    const BLEUUID obdWriteUuid((uint16_t)0xFFF2);
    const BLEUUID obdNotifyUuid((uint16_t)0xFFF1);
    bool obdServiceFound = false;
    s_obdProfileFound = false;
    s_svcChars = 0;
    s_svcCount = 0;
    s_charDescN = 0;
    s_wProps = 0;
    s_nProps = 0;
    s_chW = nullptr;
    s_chN = nullptr;
    s_wUuid[0] = 0;
    s_nUuid[0] = 0;
    auto services = s_client->getServices();

    /* 发现期间若链路已断（回调已把代号 +1），就别再往事件流里写「特征就绪」了 */
    if (gen != s_connGen) {
        releaseClient();
        return false;
    }
    for (auto &kv : *services) {
        BLERemoteService *svc = kv.second;
        if (!svc) continue;
        s_svcCount++;
        const String su = String(svc->getUUID().toString().c_str());
        const bool isObdService = svc->getUUID().equals(obdServiceUuid);
        if (isObdService) obdServiceFound = true;
        Serial.printf("{\"v\":2,\"ev\":\"svc\",\"uuid\":\"%s\"}\n", su.c_str());
        auto chars = svc->getCharacteristics();
        for (auto &kc : *chars) {
            BLERemoteCharacteristic *ch = kc.second;
            if (!ch) continue;
            s_svcChars++;
            const String cu = String(ch->getUUID().toString().c_str());
            const bool cw = ch->canWrite(), cwnr = ch->canWriteNoResponse();
            const bool cnt = ch->canNotify(), cind = ch->canIndicate();
            const bool crd = ch->canRead();
            Serial.printf("{\"v\":2,\"ev\":\"chr\",\"uuid\":\"%s\",\"write\":%d,\"wnr\":%d,"
                          "\"notify\":%d,\"ind\":%d,\"read\":%d}\n",
                          cu.c_str(), cw ? 1 : 0, cwnr ? 1 : 0,
                          cnt ? 1 : 0, cind ? 1 : 0, crd ? 1 : 0);
            charDescAdd(cu.c_str(), cw, cwnr, cnt, cind, crd);

            /* Target adapter profile confirmed by phone capture and board discovery:
               FFF0 service, FFF2 write, FFF1 notify. Keep both channels in that service. */
            if (isObdService && ch->getUUID().equals(obdWriteUuid) && (cw || cwnr)) {
                s_chW = ch;
                snprintf(s_wUuid, sizeof(s_wUuid), "%s", cu.c_str());
                s_wProps = (uint8_t)((cw ? 1 : 0) | (cwnr ? 2 : 0) | (crd ? 4 : 0));
            }
            if (isObdService && ch->getUUID().equals(obdNotifyUuid) && (cnt || cind)) {
                s_chN = ch;
                snprintf(s_nUuid, sizeof(s_nUuid), "%s", cu.c_str());
                s_nProps = (uint8_t)((cnt ? 1 : 0) | (cind ? 2 : 0) | (crd ? 4 : 0));
            }
        }
    }

    s_obdProfileFound = obdServiceFound;
    if (!s_chW || !s_chN) {
        /* ★ 先把「服务缺失」作为一个独立步骤停留一下，再断开。
           上一版这里直接 g_link=LINK_FAILED，于是屏幕上只看得到「连接失败」，
           「特征缺失」根本没有任何显示机会 —— 两者被合并了。 */
        g_svc = SVC_MISSING;
        s_missPending = true;
        pubStepEx(CS_CHARS_MISS, addr);
        Serial.printf("{\"v\":2,\"ev\":\"ble-no-chars\",\"svc\":\"missing\",\"svc_n\":%d,\"chars\":%d}\n",
                      s_svcCount, s_svcChars);
        vTaskDelay(pdMS_TO_TICKS(CONN_DWELL_MS * 2));
        releaseClient();
        g_link = LINK_FAILED;
        pubStepEx(CS_IDLE, addr);
        return false;
    }
    g_svc = SVC_FOUND;

    /* ★ 先把「身份 + 本轮观测」全部写好，再发布 CS_CHARS_OK。
       原因：主循环看到 CS_CHARS_OK 后会立刻写 ble-info，而 ble-info 要带 addr/name/UUID ——
       这些必须在发布之前就位，否则事件里会是空地址。 */
    snprintf(s_devAddr, sizeof(s_devAddr), "%s", addr);
    snprintf(s_devName, sizeof(s_devName), "%s", name ? name : "");
    s_devType = atype;
    s_devRssi = 0;

    s_bleInfoSent  = false;
    s_missSent     = false;
    s_regSent      = false;
    s_ntfSeen      = false;
    s_notifyCount  = 0;
    s_firstByteAt  = 0;
    s_firstByteMs  = 0;
    s_probeState   = PR_NONE;
    s_lastProbeMs  = 0;
    s_timeoutSnaps = 0;
    s_bindWant     = false;
    s_bindAt       = 0;
    s_atStep       = 0;
    s_ready        = false;
    s_waiting      = false;
    s_draining     = false;
    s_lastPromptAt = 0;
    s_regAt        = 0;
    const uint32_t clearedAt = millis();
    rxReset();
    s_extraPidIdx = kCorePidCount - 1;
    s_coreRequestsSinceExtra = 0;
    for (int i = 0; i < V2_PID_COUNT; i++) {
        s_pidDue[i] = clearedAt;
        s_pid[i].supported = 0;
        s_pid[i].hasVal = false;
        s_pid[i].lastOkMs = 0;
        s_pid[i].lastResult = PR_NONE;
        s_pid[i].lastEcu = -1;
    }
    s_lastOkMs     = 0;
    s_atBegun      = false;
    s_atRes[0] = s_atRes[1] = s_atRes[2] = 0;
    /* 锁存：主循环据此写 ble-info（即使错过了 SVC_OK 那 300ms 也不会丢） */
    s_charsReady = true;
    pubStepEx(CS_CHARS_OK, addr);
    vTaskDelay(pdMS_TO_TICKS(CONN_DWELL_MS));      /* 只为让「服务就绪」在屏上停留 */

    BLERemoteCharacteristic *notify = s_chN;
    if (gen != s_connGen || !notify) { releaseClient(); return false; }
    notify->registerForNotify(notifyCb, notify->canNotify());
    if (gen != s_connGen || !s_client->isConnected()) { releaseClient(); return false; }
    s_notifyOn = true;
    /* ★ registerForNotify() 返回 void，本库拿不到 CCCD 写入结果。
       s_regAt 只表示「调用已执行」，绝不写成「订阅成功」。 */
    s_regAt = millis();
    s_linkAt = s_regAt + AT_SETTLE_MS;
    pubStepEx(CS_NOTIFY_DONE, addr);      /* 最后一步：此后主循环才允许发 AT */

    Serial.printf("{\"v\":2,\"ev\":\"ble-ready\",\"addr\":\"%s\",\"svc\":\"found\",\"svc_n\":%d,"
                  "\"chars\":%d,\"notify\":%d,\"w_uuid\":\"%s\",\"n_uuid\":\"%s\","
                  "\"reg_ok\":\"UNKNOWN\",\"cccd\":\"UNKNOWN\"}\n",
                  addr, s_svcCount, s_svcChars, s_notifyOn ? 1 : 0,
                  s_wUuid[0] ? s_wUuid : "-", s_nUuid[0] ? s_nUuid : "-");
    return true;
}

/* ================= BLE 任务 ================= */
/* ★ 同步等待扫描窗口走完。
   BLEScan::start() 是异步的，紧接着调用 stop() 会让窗口实际为 0 毫秒，
   实测表现为 scan-begin 与 scan-end 同一时刻、n=0，永远扫不到设备。
   本函数运行在 bleTask 里，阻塞不会拖住主循环与 SD 写入。 */
static void scanWait(uint32_t ms)
{
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        delay(50);
        if (obdCandCount() >= V2_MAX_CAND) break;
    }
}

static bool scanTarget(BLEScan *sc, const BleCand &target, BleCand &found)
{
    taskENTER_CRITICAL(&s_candMux);
    s_retryCandidate = {};
    snprintf(s_retryTarget, sizeof(s_retryTarget), "%s", target.addr);
    snprintf(s_retryCandidate.name, sizeof(s_retryCandidate.name), "%s", target.name);
    taskEXIT_CRITICAL(&s_candMux);

    g_link = LINK_SCANNING;
    sc->clearResults();
    const uint32_t at = millis();
    if (sc->start(BLE_SCAN_MS / 1000, nullptr, false)) {
        while (millis() - at < BLE_SCAN_MS && !s_autoPaused && !s_reqDisc && !s_reqSaved &&
               s_reqConn < 0 && g_mode == MODE_REAL) {
            taskENTER_CRITICAL(&s_candMux);
            const bool seen = s_retryCandidate.used;
            taskEXIT_CRITICAL(&s_candMux);
            if (seen) break;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        sc->stop();
    }
    sc->clearResults();
    taskENTER_CRITICAL(&s_candMux);
    found = s_retryCandidate;
    s_retryTarget[0] = 0;
    taskEXIT_CRITICAL(&s_candMux);
    return found.used && !s_autoPaused && !s_reqDisc && !s_reqSaved &&
           s_reqConn < 0 && g_mode == MODE_REAL;
}


/* ★ 候选排序：带 FFE0 服务的优先，其次按 RSSI 由强到弱。
   理由：屏幕只显示 4 行且触摸无法滚动，若不排序，用户的适配器可能落在第 5 行之后，
   用触摸永远选不到。就近的适配器 RSSI 通常最强，会排到第一行。 */
static void sortCands()
{
    taskENTER_CRITICAL(&s_candMux);
    const int n = s_candN;
    for (int i = 0; i < n; i++) {
        int best = i;
        for (int j = i + 1; j < n; j++) {
            const BleCand &a = s_cand[j];
            const BleCand &b = s_cand[best];
            bool better = false;
            if (a.hasFfe0 != b.hasFfe0)      better = (a.hasFfe0 && !b.hasFfe0);
            else if (a.rssi != b.rssi)       better = (a.rssi > b.rssi);
            if (better) best = j;
        }
        if (best != i) {
            BleCand t = s_cand[i];
            s_cand[i] = s_cand[best];
            s_cand[best] = t;
        }
    }
    taskEXIT_CRITICAL(&s_candMux);
}

static void bleTask(void *arg)
{
    (void)arg;
    BLEScan *sc = BLEDevice::getScan();
    uint32_t retryAt = 0;
    BleCand peer{};

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));

        if (s_reqDisc) {
            s_reqDisc = false;
            s_reqRetry = false;
            s_reqSaved = false;
            s_reqConn = -1;
            s_recoverLink = false;
            releaseClient();
            peer = {};
            s_ready = false;
            s_atBegun = false;
            s_charsReady = false;
            g_link = LINK_IDLE;
            g_ecu = ECU_NONE;
            pubStep(CS_IDLE);
            continue;
        }

        if (g_mode == MODE_SIM) continue;

        if (s_reqSaved) {
            s_reqSaved = false;
            peer = {};
            retryAt = 0;
            s_reqRetry = s_savedBound;
        }

        if (s_recoverLink) {
            releaseClient();
            g_link = LINK_FAILED;
            pubStep(CS_IDLE);
            s_recoverLink = false;
            if (!s_autoPaused) s_reqRetry = true;
            continue;
        }

        if (s_reqConn >= 0) {
            const int idx = s_reqConn;
            s_reqConn = -1;
            /* 取副本，避免与扫描回调并发 */
            char addr[20] = {0}, nm[33] = {0};
            uint8_t tp = 0;
            bool ok = false;
            taskENTER_CRITICAL(&s_candMux);
            if (idx < s_candN) {
                snprintf(addr, sizeof(addr), "%s", s_cand[idx].addr);
                snprintf(nm, sizeof(nm), "%s", s_cand[idx].name);
                tp = s_cand[idx].atype;
                ok = true;
            }
            taskEXIT_CRITICAL(&s_candMux);
            if (ok) {
                s_reqRetry = false;
                snprintf(peer.addr, sizeof(peer.addr), "%s", addr);
                snprintf(peer.name, sizeof(peer.name), "%s", nm);
                peer.atype = tp;
                retryAt = millis();
                if (!connectAddr(addr, tp, nm)) s_reqRetry = true;
            }
            continue;
        }

        if (s_reqScan) {
            s_reqScan = false;
            taskENTER_CRITICAL(&s_candMux);
            s_candN = 0;
            for (int i = 0; i < V2_MAX_CAND; i++) {
                s_cand[i].used = false;
                s_cand[i].name[0] = 0;
                s_cand[i].addr[0] = 0;
                s_cand[i].hasFfe0 = false;
            }
            taskEXIT_CRITICAL(&s_candMux);
            s_pick = 0;
            s_scanDone = false;
            s_scanN = 0;
            /* Listing nearby devices must not replace the connection state of a live session. */
            if (g_link != LINK_CONNECTED) g_link = LINK_SCANNING;
            Serial.println(F("{\"v\":2,\"ev\":\"scan-begin\"}"));
            sc->start((uint32_t)(BLE_SCAN_MS / 1000), nullptr, false);
            scanWait(BLE_SCAN_MS);
            sc->stop();
            sc->clearResults();
            sortCands();          /* FFE0 优先 + RSSI 降序，保证第一行就是最可能是适配器的那个 */
            s_scanN = obdCandCount();
            s_scanDone = (s_scanN > 0);
            Serial.printf("{\"v\":2,\"ev\":\"scan-end\",\"n\":%d,\"ffs\":%d}\n",
                          s_scanN, (s_candN > 0 && s_cand[0].hasFfe0) ? 1 : 0);
            if (g_link == LINK_SCANNING) g_link = LINK_IDLE;
            continue;
        }

        /* Boot uses the persisted peer. During this boot, respect explicit manual selection. */
        if (s_reqRetry && !s_autoPaused &&
            (retryAt == 0 || millis() - retryAt >= BLE_RECONN_MS)) {
            s_reqRetry = false;
            if (g_link == LINK_CONNECTED) continue;
            if (!peer.addr[0]) {
                taskENTER_CRITICAL(&s_candMux);
                if (s_savedBound) {
                    snprintf(peer.addr, sizeof(peer.addr), "%s", s_savedAddr);
                    snprintf(peer.name, sizeof(peer.name), "%s", s_savedName);
                }
                taskEXIT_CRITICAL(&s_candMux);
            }
            if (!peer.addr[0]) continue;
            retryAt = millis();
            Serial.printf("{\"v\":2,\"ev\":\"retry\",\"addr\":\"%s\"}\n", peer.addr);
            BleCand found{};
            if (scanTarget(sc, peer, found)) {
                if (!connectAddr(found.addr, found.atype, found.name)) s_reqRetry = true;
            } else {
                g_link = LINK_FAILED;
                Serial.printf("{\"v\":2,\"ev\":\"saved-not-found\",\"addr\":\"%s\","
                              "\"hint\":\"will-retry\"}\n", peer.addr);
                if (!s_autoPaused) s_reqRetry = true;
            }
            continue;
        }
    }
}

/* ================= 初始化 ================= */
void obdBegin()
{
    loadBind();
    BLEDevice::init("LAOMO-GAUGE");
#if defined(CONFIG_BLUEDROID_ENABLED)
    BLEDevice::setCustomGattcHandler(captureDisconnectReason);
#endif
    s_clientUse = xSemaphoreCreateMutex();
    BLEScan *sc = BLEDevice::getScan();
    sc->setAdvertisedDeviceCallbacks(new ScanCb(), true);
    sc->setInterval(1349);
    sc->setWindow(449);
    sc->setActiveScan(true);
    if (s_clientUse) xTaskCreatePinnedToCore(bleTask, "ble", 8192, nullptr, 1, nullptr, 0);

    Serial.printf("{\"v\":2,\"ev\":\"ble-init\",\"saved\":%d,\"addr\":\"%s\",\"atype\":%u,"
                  "\"name\":\"%s\"}\n",
                  s_savedBound ? 1 : 0, s_savedAddr, (unsigned)s_savedType, s_savedName);
}

void obdRequestScan()      { s_reqScan = true; }
void obdSelectCurrent()    { s_autoPaused = false; s_reqSaved = false; s_reqConn = s_pick; }
void obdConnectSaved()     { s_autoPaused = false; if (s_savedBound) s_reqSaved = true; }
void obdDisconnect()       { s_autoPaused = true; s_reqDisc = true; }
void obdRetry()            { obdConnectSaved(); }

/* ================= 帧解析 ================= */
/* 返回解析出的完整帧数；坏帧计 bad */
static int parseFrames(const char *s, Frame *out, int max, int *bad)
{
    int nf = 0;
    *bad = 0;
    const char *p = s;
    while (*p && nf < max) {
        /* 取一行 */
        const char *e = p;
        while (*e && *e != '\r' && *e != '\n') e++;

        char line[96];
        size_t len = (size_t)(e - p);
        if (len >= sizeof(line)) len = sizeof(line) - 1;
        memcpy(line, p, len);
        line[len] = 0;

        p = (*e) ? e + 1 : e;

        /* 去掉空白与提示符 */
        char *q = line;
        while (*q == ' ') q++;
        size_t ql = strlen(q);
        while (ql && (q[ql - 1] == ' ' || q[ql - 1] == '>')) q[--ql] = 0;
        if (!ql) continue;
        if (!strncmp(q, "NO DATA", 7) || !strncmp(q, "SEARCHING", 9) || !strcmp(q, "?")) {
            (*bad)++;
            continue;
        }
        /* 注意：绝不能按首字符过滤（0xA0~0xAF 是合法数据字节）。
           AT 应答在 obdTick 里已单独走 at-rsp 分支，不会进到这里。 */
        if (!strncmp(q, "OK", 2) && ql <= 4) continue;

        int ecu = -1;
        char *hex = q;
        /* 形如 "7E8 04 41 0C 1A F8" */
        if (!strncmp(q, "7E8", 3) || !strncmp(q, "7e8", 3)) { ecu = 0x7E8; hex = q + 3; }
        else if (!strncmp(q, "7E9", 3) || !strncmp(q, "7e9", 3)) { ecu = 0x7E9; hex = q + 3; }

        /* 转成字节 */
        int bytes[12];
        int nb = 0;
        while (*hex && nb < 12) {
            while (*hex == ' ') hex++;
            if (!isxdigit((unsigned char)hex[0]) || !isxdigit((unsigned char)hex[1])) break;
            char bt[3] = { hex[0], hex[1], 0 };
            bytes[nb++] = (int)strtol(bt, nullptr, 16);
            hex += 2;
        }
        if (nb == 0) continue;

        int off = 0;
        /* 有 ISO-TP 长度字节时先剥掉 */
        if (nb >= 2 && bytes[0] >= 1 && bytes[0] <= 8 && bytes[0] + 1 <= nb) off = 1;
        const int avail = nb - off;

        if (ecu < 0 && avail >= 2 && bytes[off] == 0x41) {
            /* 无头模式下 ADC 报告本身就是 41 xx，直接当帧 */
            ecu = -1;
        }

        Frame &f = out[nf];
        f.ecu = ecu;
        f.n = avail > 8 ? 8 : avail;
        for (int i = 0; i < f.n; i++) f.d[i] = (uint8_t)bytes[off + i];
        nf++;
    }
    return nf;
}

/* 优先 7E8，缺失才回退 7E9 —— 与已验证的回放规则一致 */
static const Frame *pickFrame(const Frame *f, int n, uint8_t pid, int payloadBytes, bool *usedFallback)
{
    *usedFallback = false;
    const Frame *fallback = nullptr;
    const Frame *headerless = nullptr;
    for (int i = 0; i < n; i++) {
        if (f[i].n < 2 + payloadBytes || f[i].d[0] != 0x41 || f[i].d[1] != pid) continue;
        if (f[i].ecu == 0x7E8) return &f[i];
        if (f[i].ecu == 0x7E9 && !fallback) fallback = &f[i];
        if (f[i].ecu < 0 && !headerless) headerless = &f[i];
    }
    *usedFallback = fallback != nullptr;
    return fallback ? fallback : headerless;
}

/* 应答分类：把「没数据」「在搜索」「收到但解析不了」「有效」分开 */
static uint8_t classifyRx(const String &raw, bool valid, int bad, int nf)
{
    if (valid)                return 1;   /* valid */
    if (raw.length() == 0)    return 0;   /* empty */
    if (raw.indexOf("NO DATA") >= 0)   return 2;   /* nodata */
    if (raw.indexOf("SEARCHING") >= 0) return 3;   /* search */
    if (nf == 0 && bad > 0)   return 3;   /* 只有 '?' 之类 */
    return 4;                             /* 有原文但解析不出有效帧 */
}
static const char *kRxClsStr[5] = { "empty", "valid", "nodata", "search", "unparse" };
const char *obdLastRxCls() { return kRxClsStr[(s_lastRxCls < 5) ? s_lastRxCls : 0]; }

/* 分类 → 阶段结果码（供 0100 探针与轮询显示用） */
static uint8_t clsToResult(uint8_t cls)
{
    switch (cls) {
    case 1:  return PR_OK;
    case 2:  return PR_NODATA;
    case 3:  return PR_SEARCH;
    case 4:  return PR_BADFRM;
    default: return PR_BADFRM;      /* 空应答也算「没解析出有效帧」 */
    }
}

const char *obdPidResultStr(const PidRec &pid)
{
    return kPhResStr[pid.lastResult < PR_N ? pid.lastResult : PR_NONE];
}

static bool isSupportPage(uint8_t pid)
{
    return pid == 0x00 || pid == 0x20 || pid == 0x40;
}

static uint32_t supportMask(const Frame &frame)
{
    return ((uint32_t)frame.d[2] << 24) | ((uint32_t)frame.d[3] << 16) |
           ((uint32_t)frame.d[4] << 8) | frame.d[5];
}

static void applySupportPage(int page, uint32_t mask)
{
    for (PidRec &entry : s_pid) {
        const int bit = entry.pid - page;
        if (bit < 1 || bit > 32) continue;
        if ((mask >> (32 - bit)) & 1u) entry.supported = 1;
        else if (entry.pid != 0x42 && entry.supported != 1) entry.supported = 2;
    }
}

static uint32_t combinedSupportMask(const Frame *frames, int count, uint8_t page,
                                    const Frame &selected)
{
    uint32_t mask = supportMask(selected);
    for (int i = 0; i < count; i++) {
        const Frame &frame = frames[i];
        if (frame.n >= 6 && frame.d[0] == 0x41 && frame.d[1] == page &&
            (frame.ecu == 0x7E8 || frame.ecu == 0x7E9)) mask |= supportMask(frame);
    }
    return mask;
}

static bool decodePidValue(const PidRec &pid, const Frame &frame, float &value)
{
    if (isSupportPage(pid.pid)) return true;
    uint32_t bytes = 0;
    for (int i = 0; i < pid.nbytes; i++) bytes = (bytes << 8) | frame.d[2 + i];
    value = (float)bytes * pid.scale + pid.offset;
    return value >= pid.minV && value <= pid.maxV;
}

/* ================= 一条应答的完整处理（REAL 与 SIM 共用） =================
   ★ r2 改动：先解析再写一条事件，事件里同时带「原文 + 分类 + 是否有效」，
   这样 SD 里一条就足以区分「没收到」「收到但解析不了」「收到有效帧」。 */
void obdFeedRaw(const char *cmd, const String &raw, uint32_t ms, bool isSim)
{
    const char *org = isSim ? "sim" : "real";

    /* ---- 解析 ---- */
    Frame fr[8];
    int bad = 0;
    const int nf = parseFrames(raw.c_str(), fr, 8, &bad);

    /* 找对应的 PID 记录 */
    int pi = -1;
    for (int i = 0; i < V2_PID_COUNT; i++) if (!strcmp(s_pid[i].cmd, cmd)) { pi = i; break; }

    bool fallback = false;
    const Frame *f = pi >= 0 ? pickFrame(fr, nf, s_pid[pi].pid, s_pid[pi].nbytes, &fallback) : nullptr;

    const bool isNoData = (raw.indexOf("NO DATA") >= 0);
    float val = 0.f;
    const bool valid = f && decodePidValue(s_pid[pi], *f, val);
    const int ecuUsed = f ? f->ecu : -1;

    const uint8_t cls = classifyRx(raw, valid, bad, nf);
    if (!isSim) s_lastRxCls = cls;

    /* ---- 原始应答逐字保留（转义 \r\n），带长度上限与截断标记 ---- */
    {
        static char esc[RX_ESC_MAX + 8];
        size_t o = 0;
        bool trunc = false;
        for (size_t i = 0; i < raw.length(); i++) {
            if (o + 7 >= sizeof(esc)) { trunc = true; break; }
            const uint8_t c = (uint8_t)raw[i];
            if (c == '\r')      o += (size_t)snprintf(esc + o, sizeof(esc) - o, "\\r");
            else if (c == '\n') o += (size_t)snprintf(esc + o, sizeof(esc) - o, "\\n");
            else if (c == '\\') o += (size_t)snprintf(esc + o, sizeof(esc) - o, "\\\\");
            else if (c == '"')  o += (size_t)snprintf(esc + o, sizeof(esc) - o, "\\\"");
            else if (c >= 0x20 && c < 0x7f) esc[o++] = (char)c;
            else                o += (size_t)snprintf(esc + o, sizeof(esc) - o, "\\u%04x", c);
        }
        esc[o] = 0;

        static char line[900];
        snprintf(line, sizeof(line),
                 "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"rx\",\"org\":\"%s\",\"sess\":%lu,"
                 "\"cmd\":\"%s\",\"ms\":%lu,\"cls\":\"%s\",\"valid\":%s,\"nf\":%d,\"bad\":%d,"
                 "\"ecu\":\"%s\",\"rawlen\":%u,\"raw_trunc\":%s,\"raw\":\"%s\"}",
                 (unsigned long)millis(), org, (unsigned long)g_sess, cmd, (unsigned long)ms,
                 kRxClsStr[cls], valid ? "true" : "false", nf, bad,
                 ecuUsed < 0 ? "none" : (ecuUsed == 0x7E8 ? "7E8" : "7E9"),
                 (unsigned)raw.length(), trunc ? "true" : "false", esc);
        logJson(line);
    }

    if (pi < 0) return;

    if (isSim) {
        if (valid) { g_simOk++; s_simHas[pi] = true; s_simVal[pi] = val; }
        return;          /* ★ SIM 数据到此为止，绝不写进 REAL 统计 */
    }

    /* ---- REAL ---- */
    s_pid[pi].req++;
    s_pid[pi].lastResult = clsToResult(cls);
    if (valid) {
        s_pid[pi].ok++;
        s_pid[pi].lastOkMs = millis();
        s_pid[pi].lastVal = val;
        s_pid[pi].hasVal = true;
        s_pid[pi].lastEcu = ecuUsed;
        if (s_pid[pi].pid != 0x00) s_pid[pi].supported = 1;
        s_lastOkMs = millis();
        g_realOk++;
        g_ecu = ECU_VALID;

        if (isSupportPage(s_pid[pi].pid)) {
            const uint8_t page = s_pid[pi].pid;
            const uint32_t unionMask = combinedSupportMask(fr, nf, page, *f);
            applySupportPage(page, unionMask);
            logJsonf(
                     "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"support\",\"org\":\"real\","
                     "\"sess\":%lu,\"cmd\":\"%s\",\"mask\":\"%08lX\",\"union_mask\":\"%08lX\",\"ecu\":%s}",
                     (unsigned long)millis(), (unsigned long)g_sess, cmd,
                     (unsigned long)supportMask(*f), (unsigned long)unionMask,
                     ecuUsed < 0 ? "null" : (ecuUsed == 0x7E8 ? "\"7E8\"" : "\"7E9\""));
        } else {
            char v[260];
            snprintf(v, sizeof(v),
                     "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"value\",\"org\":\"real\","
                     "\"sess\":%lu,\"pid\":\"%s\",\"name\":\"%s\",\"val\":%.2f,\"unit\":\"%s\","
                     "\"ecu\":\"%s\",\"fb\":%s,\"ok\":true}",
                     (unsigned long)millis(), (unsigned long)g_sess, s_pid[pi].cmd,
                     s_pid[pi].label, val, s_pid[pi].unit,
                     ecuUsed == 0x7E8 ? "7E8" : (ecuUsed == 0x7E9 ? "7E9" : "none"),
                     fallback ? "true" : "false");
            logJson(v);
        }

        /* Preserve the initial probe result, even if a later 0100 succeeds. */
        if (s_atStep == 3) {
            s_probeState = PR_OK;            /* ★ 探针结论只在这里写 */
            s_atStep = 4;                    /* ★ 无论是否首次绑定都必须前进，否则死循环发 0100 */
            stageResult(PR_OK, "0100");
        }
        if (s_pid[pi].pid == 0x00 && !s_ready) {
            s_ready = true;
            if (!s_savedBound || strcasecmp(s_devAddr, s_savedAddr) != 0 || s_devType != s_savedType)
                s_bindWant = true;
        }
        /* 轮询阶段的最近一次 PID 结果 */
        if (s_atStep >= 4 && !isSupportPage(s_pid[pi].pid)) {
            snprintf(s_lastPidCmd, sizeof(s_lastPidCmd), "%s", s_pid[pi].cmd);
            s_lastPidRes = PR_OK;
            s_lastPidVal = val;
        }
    } else {
        if (bad > 0 && !isNoData) {
            s_pid[pi].badframe++;
        } else {
            s_pid[pi].nodata++;
            g_realNoData++;
            /* NO DATA, empty replies and parse errors are temporary outcomes. */
            if (g_ecu == ECU_NONE) g_ecu = ECU_NODATA;
        }
        const uint8_t r = clsToResult(cls);
        if (s_atStep == 3) {
            s_probeState = r;                /* 0100 探针的失败分类 */
            s_atStep = 4;                    /* 不让探针无限重发 */
            stageResult(r, "0100");
        } else if (s_atStep >= 4 && s_pid[pi].pid != 0x00) {
            snprintf(s_lastPidCmd, sizeof(s_lastPidCmd), "%s", s_pid[pi].cmd);
            s_lastPidRes = r;
            s_lastPidVal = 0.f;
            stageResult(r, s_pid[pi].cmd);   /* PH_POLL：只刷新屏幕，不写卡 */
        }
    }
}

/* ================= 请求调度 ================= */
static void logSync(const char *event, const char *cause, const char *raw)
{
    char escaped[TIMEOUT_SNAP_LEN * 2 + 8];
    const size_t n = strlen(raw);
    snapToJson(raw, n < TIMEOUT_SNAP_LEN ? n : TIMEOUT_SNAP_LEN, escaped, sizeof(escaped));
    logJsonf("{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"%s\",\"sess\":%lu,"
             "\"cmd\":\"%s\",\"cause\":\"%s\",\"rawlen\":%u,\"partial\":\"%s\"}",
             (unsigned long)millis(), event, (unsigned long)g_sess, s_sentCmd, cause,
             (unsigned)n, escaped);
}

static void startDrain()
{
    /* Keep the old RX buffer intact. The next prompt still belongs to the old exchange. */
    s_draining = true;
    s_drainAt = millis();
    s_drainSawPrompt = false;
}

static void requestRecovery(const char *cause)
{
    logSync("obd-recover", cause, "");
    s_recoverLink = true;
}

static void pollDrain()
{
    static char old[720];
    if (takeReply(old, sizeof(old))) {
        s_drainSawPrompt = true;
        logSync("obd-late", "drain", old);
        return;
    }

    portENTER_CRITICAL(&s_rxMux);
    const bool empty = s_rxLen == 0 && !s_rxOverflow;
    const uint32_t lastRx = s_rxAt;
    portEXIT_CRITICAL(&s_rxMux);
    const uint32_t now = millis();
    if (s_drainSawPrompt && empty && now - lastRx >= OBD_MIN_GAP_MS) {
        s_draining = false;
        s_lastPromptAt = now;
    } else if (now - s_drainAt >= OBD_DRAIN_MS) {
        requestRecovery("prompt-missing-or-incomplete");
    }
}

static bool wrongResponse(const char *cmd, const char *raw)
{
    Frame frames[8];
    int bad = 0;
    const int n = parseFrames(raw, frames, 8, &bad);
    if (s_sentIsAt) {
        for (int i = 0; i < n; i++) if (frames[i].n >= 2 && frames[i].d[0] == 0x41) return true;
        return false;
    }
    const uint8_t wanted = (uint8_t)strtoul(cmd + 2, nullptr, 16);
    const bool adapterReply = strstr(raw, "OK") != nullptr || strstr(raw, "ELM327") != nullptr;
    bool otherPid = false;
    for (int i = 0; i < n; i++) {
        if (frames[i].n < 2 || frames[i].d[0] != 0x41) continue;
        if (frames[i].d[1] == wanted) return false;
        otherPid = true;
    }
    return otherPid || adapterReply;
}

static bool sendCmd(const char *cmd, bool isAt)
{
    if (!s_clientUse || xSemaphoreTake(s_clientUse, 0) != pdTRUE) return false;
    BLERemoteCharacteristic *write = s_chW;
    const uint8_t gen = s_connGen;
    if (!write || g_link != LINK_CONNECTED || s_connStep != CS_NOTIFY_DONE) {
        xSemaphoreGive(s_clientUse);
        return false;
    }

    portENTER_CRITICAL(&s_rxMux);
    if (s_rxLen || s_rxOverflow) {
        portEXIT_CRITICAL(&s_rxMux);
        xSemaphoreGive(s_clientUse);
        startDrain();
        return false;
    }
    s_firstByteAt = 0;
    portEXIT_CRITICAL(&s_rxMux);
    s_firstByteMs = 0;

    snprintf(s_sentCmd, sizeof(s_sentCmd), "%s", cmd);
    s_sentIsAt = isAt;
    s_waiting = true;

    char buf[12];
    snprintf(buf, sizeof(buf), "%s\r", cmd);

    /* ★ 提交时刻与返回时刻分开记录：
       t_sub = 调用 writeValue 之前；t_wr = 返回之后。
       返回值只表示本地 BLE 库调用返回，不代表适配器收到，更不代表 ECU 应答。 */
    const uint32_t tSub = millis();
    const bool wok = write->writeValue((uint8_t *)buf, strlen(buf), false);
    const uint32_t tWr = millis();

    s_sentSub = tSub;
    s_sentAt  = tWr;
    s_lastTx  = tWr;
    if (!wok || gen != s_connGen) s_waiting = false;
    xSemaphoreGive(s_clientUse);

    /* 先显示「已发送待响应」，再进入等待 —— 屏幕上永远是「正在执行」，不会把已发当已回 */
    if (!wok) stageResult(PR_SENDFAIL, cmd);
    else      stageResult(PR_SENT, cmd);

    char line[300];
    snprintf(line, sizeof(line),
             "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"tx\",\"org\":\"real\",\"sess\":%lu,"
             "\"cmd\":\"%s\",\"at\":%s,\"wrsp\":false,\"t_sub\":%lu,\"t_wr\":%lu,\"wms\":%lu,\"wok\":%d}",
             (unsigned long)tWr, (unsigned long)g_sess, cmd, isAt ? "true" : "false",
             (unsigned long)tSub, (unsigned long)tWr, (unsigned long)(tWr - tSub),
             wok ? 1 : 0);
    logJson(line);
    if (!wok && !isAt) {
        for (PidRec &pid : s_pid) {
            if (!strcmp(pid.cmd, cmd)) { pid.lastResult = PR_SENDFAIL; break; }
        }
    }
    if (!wok && gen == s_connGen) requestRecovery("write-failed");
    return true;
}

static int nextExtraPid(uint32_t now)
{
    if (!s_ready || s_coreRequestsSinceExtra < kCorePidCount - 1) return -1;
    const uint32_t budget = OBD_PID_TIMEOUT_MS + 2 * OBD_MIN_GAP_MS;
    for (int i = 0; i < kCorePidCount; i++) {
        const PidRec &pid = s_pid[i];
        if (!isSupportPage(pid.pid) && pid.hasVal && now - pid.lastOkMs >= STALE_MS - budget)
            return -1;
    }
    const int count = V2_PID_COUNT - kCorePidCount;
    for (int k = 1; k <= count; k++) {
        const int i = kCorePidCount + (s_extraPidIdx - kCorePidCount + k) % count;
        const PidRec &pid = s_pid[i];
        if (pid.supported != 1 || (isSupportPage(pid.pid) && pid.hasVal)) continue;
        if ((int32_t)(now - s_pidDue[i]) >= 0) return i;
    }
    return -1;
}

void obdTick()
{
    stageTick();                               /* ★ 阶段变化检测（含未连接时的扫描/连接阶段） */
    if (g_mode == MODE_SIM) return;            /* SIM 绝不发真请求 */

    /* ---- 写绑定：先停在一个可见的「写绑定」阶段，再写 NVS（都在主循环） ---- */
    if (s_bindWant && g_link == LINK_CONNECTED) {
        if (s_bindAt == 0) {
            s_bindAt = millis() + BIND_DWELL_MS;   /* 先让屏幕显示得到 */
            stageTick();
            return;
        }
        if ((int32_t)(millis() - s_bindAt) < 0) return;
        s_bindWant = false;
        s_bindDoing = true;
        stageTick();                            /* 进入 PH_BIND 并刷新屏幕/写事件 */
        const bool saved = saveBind(s_devAddr, s_devType, s_devName);
        logJsonf("{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"bind-result\","
                 "\"sess\":%lu,\"addr\":\"%s\",\"ok\":%s}",
                 (unsigned long)millis(), (unsigned long)g_sess, s_devAddr, saved ? "true" : "false");
        s_bindDoing = false;
        s_bindAt = 0;
        stageTick();                            /* 转到 PH_READY / PH_POLL */
        return;                                 /* 下一轮再开始轮询 */
    }

    /* ★ 只有连接步骤走完（通知已注册）才允许发 AT。
       否则在「特征缺失」停留期间，若只缺了通知特征、s_chW 仍是有效的，
       握手会提前开始，白等一次 ATZ 超时。 */
    if (g_link != LINK_CONNECTED || !s_chW || s_connStep != CS_NOTIFY_DONE ||
        s_reqDisc || s_recoverLink) return;

    if (s_draining) { pollDrain(); return; }

    const uint32_t now = millis();

    /* ---- 等待应答 ---- */
    if (s_waiting) {
        static char buf[720];
        const int buffered = obdRxLen();
        const size_t n = takeReply(buf, sizeof(buf));
        const bool done = n != 0;
        const uint32_t limit = s_sentIsAt ? (s_atStep == 0 ? ATZ_TIMEOUT_MS : OBD_AT_TIMEOUT_MS)
                                          : (s_atStep == 3 ? OBD_PROBE_TIMEOUT_MS : OBD_PID_TIMEOUT_MS);

        /* ★ 首字节：一旦观测到就锁定（1 表示写入返回前/同时到达） */
        if (s_firstByteAt != 0 && s_firstByteMs == 0) {
            const int32_t d = (int32_t)(s_firstByteAt - s_sentAt);
            s_firstByteMs = (d <= 0) ? 1u : (uint32_t)d;
        }

        /* ★ 收到字节但还没收完：显示「已收到 N 字节」，但绝不显示「成功」 */
        if (!done && buffered > 0 && s_phRes != PR_RECV) stageResult(PR_RECV, s_sentCmd);

        if (done) {
            if (wrongResponse(s_sentCmd, buf)) {
                logSync("obd-late", "pid-mismatch", buf);
                if (now - s_sentAt <= limit) return;
                /* Leave this request pending. The timeout below will enter drain. */
            } else {
                const uint32_t rtt = (uint32_t)(millis() - s_sentAt);
                String rsp(buf);
                s_waiting = false;
                s_lastPromptAt = millis();

                if (s_sentIsAt) {
                    const bool ok = s_atStep == 0 ? rsp.indexOf("ELM") >= 0 : rsp.indexOf("OK") >= 0;
                    char line[450], escaped[200];
                    snapToJson(buf, n < TIMEOUT_SNAP_LEN ? n : TIMEOUT_SNAP_LEN, escaped, sizeof(escaped));
                    snprintf(line, sizeof(line),
                             "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"at-rsp\",\"org\":\"real\","
                             "\"sess\":%lu,\"cmd\":\"%s\",\"ms\":%lu,\"fb\":%lu,\"rxlen\":%u,"
                             "\"ok\":%s,\"raw\":\"%s\"}",
                             (unsigned long)millis(), (unsigned long)g_sess, s_sentCmd,
                             (unsigned long)rtt, (unsigned long)s_firstByteMs, (unsigned)n,
                             ok ? "true" : "false", escaped);
                    logJson(line);
                    if (ok && s_atStep < 3) s_atRes[s_atStep] = 1;
                    stageResult(ok ? PR_OK : PR_BADFRM, s_sentCmd);
                    if (s_atStep < 3) s_atStep++;
                } else {
                    if (s_atStep == 3) s_lastProbeMs = rtt;
                    obdFeedRaw(s_sentCmd, rsp, rtt, false);
                }
                return;
            }
        }

        if (now - s_sentAt > limit) {
            /* ★ 先取片段快照，再清缓冲（超时场景最需要的证据就是「到底收到了什么」） */
            static char snapRaw[TIMEOUT_SNAP_LEN + 1];
            static char snapEsc[TIMEOUT_SNAP_LEN * 2 + 8];
            size_t sn = 0, total = 0;
            portENTER_CRITICAL(&s_rxMux);
            total = s_rxLen;
            sn = (total > (size_t)TIMEOUT_SNAP_LEN) ? (size_t)TIMEOUT_SNAP_LEN : total;
            if (sn) memcpy(snapRaw, s_rx, sn);
            portEXIT_CRITICAL(&s_rxMux);
            snapRaw[sn] = 0;
            snapEsc[0] = 0;
            if (sn) snapToJson(snapRaw, sn, snapEsc, sizeof(snapEsc));

            const bool keepSnap = (s_timeoutSnaps < TIMEOUT_SNAP_MAX);
            if (keepSnap && sn) s_timeoutSnaps++;

            s_waiting = false;
            startDrain();
            if (s_sentIsAt) {
                char line[420];
                snprintf(line, sizeof(line),
                         "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"at-timeout\",\"org\":\"real\","
                         "\"sess\":%lu,\"cmd\":\"%s\",\"ms\":%lu,\"limit_ms\":%lu,\"rxlen\":%u,"
                         "\"fb\":%lu,\"snap\":%d,\"partial\":\"%s\"}",
                         (unsigned long)millis(), (unsigned long)g_sess, s_sentCmd,
                         (unsigned long)(now - s_sentAt), (unsigned long)limit,
                         (unsigned)total, (unsigned long)s_firstByteMs,
                         keepSnap ? 1 : 0, keepSnap ? snapEsc : "");
                logJson(line);
                if (s_atStep < 3) s_atRes[s_atStep] = 2;      /* timeout */
                stageResult(PR_TIMEOUT, s_sentCmd);
                if (s_atStep < 3) s_atStep++;      /* AT 超时不致命，继续下一步 */
            } else {
                g_realTo++;                       /* 发送时已记过 req，这里不重复累加 */
                if (s_atStep == 3) s_probeState = PR_TIMEOUT;   /* 只记这一次探针结论 */
                for (int i = 0; i < V2_PID_COUNT; i++) {
                    if (strcmp(s_pid[i].cmd, s_sentCmd)) continue;
                    s_pid[i].to++;
                    s_pid[i].req++;
                    s_pid[i].lastResult = PR_TIMEOUT;
                    break;
                }
                char line[460];
                snprintf(line, sizeof(line),
                         "{\"v\":2,\"t\":%lu,\"src\":\"obd\",\"ev\":\"timeout\",\"org\":\"real\","
                         "\"sess\":%lu,\"cmd\":\"%s\",\"ms\":%lu,\"limit_ms\":%lu,\"rxlen\":%u,"
                         "\"fb\":%lu,\"snap\":%d,\"partial\":\"%s\"}",
                         (unsigned long)millis(), (unsigned long)g_sess, s_sentCmd,
                         (unsigned long)(now - s_sentAt), (unsigned long)limit,
                         (unsigned)total, (unsigned long)s_firstByteMs, keepSnap ? 1 : 0,
                         keepSnap ? snapEsc : "");
                logJson(line);
                if (s_atStep == 3) {
                    s_atStep = 4;
                    stageResult(PR_TIMEOUT, "0100");
                } else {
                    snprintf(s_lastPidCmd, sizeof(s_lastPidCmd), "%s", s_sentCmd);
                    s_lastPidRes = PR_TIMEOUT;
                    s_lastPidVal = 0.f;
                    if (s_atStep >= 4) stageResult(PR_TIMEOUT, s_sentCmd);
                }
            }
            return;
        }
        return;
    }

    /* ---- 握手：稳定窗口 → ATZ → ATE0 → ATH1 → 0100 探针 ---- */
    if (s_lastPromptAt && now - s_lastPromptAt < OBD_MIN_GAP_MS) return;
    if (s_atStep < 4) {
        if (!s_atBegun) {
            /* ★ 稳定窗口：注册 CCCD 之后缓 AT_SETTLE_MS 再发 ATZ。
               这段时间屏幕显示「通知注册」，用户看得到、事件里也对得上。 */
            if ((int32_t)(now - s_linkAt) < 0) return;
            s_atBegun = true;
            stageTick();               /* 先显示 ATZ 阶段，再发命令 */
            return;
        }
        static uint32_t atNext = 0;
        if ((int32_t)(now - atNext) < 0) return;
        if (s_atStep == 0      && sendCmd("ATZ",  true)) atNext = now + 900;
        else if (s_atStep == 1 && sendCmd("ATE0", true)) atNext = now + 400;
        else if (s_atStep == 2 && sendCmd("ATH1", true)) atNext = now + 400;
        else if (s_atStep == 3 && sendCmd("0100", false)) atNext = now + 600;
        return;
    }

    /* ---- 正常轮询 ---- */
    if (now - s_lastTx < OBD_MIN_GAP_MS) return;

    const int extra = nextExtraPid(now);
    if (extra >= 0) {
        if (sendCmd(s_pid[extra].cmd, false)) {
            s_pidDue[extra] = now + s_pid[extra].periodMs;
            s_extraPidIdx = extra;
            s_coreRequestsSinceExtra = 0;
            g_realReq++;
        }
        return;
    }

    for (int k = 0; k < kCorePidCount; k++) {
        const int i = (s_pidIdx + 1 + k) % kCorePidCount;
        if (s_pid[i].supported == 2) continue;          /* 已知不支持，跳过 */
        if ((int32_t)(now - s_pidDue[i]) < 0) continue;
        if (sendCmd(s_pid[i].cmd, false)) {
            s_pidDue[i] = now + s_pid[i].periodMs;
            s_pidIdx = i;
            if (s_coreRequestsSinceExtra < kCorePidCount - 1) s_coreRequestsSinceExtra++;
            g_realReq++;
        }
        return;
    }
}

float obdVal(int i, bool *has, bool *isSim)
{
    if (i < 0 || i >= V2_PID_COUNT) { if (has) *has = false; return 0.f; }
    if (g_mode == MODE_SIM) {
        if (isSim) *isSim = true;
        if (has) *has = s_simHas[i];
        return s_simVal[i];
    }
    if (isSim) *isSim = false;
    const uint32_t freshMs = i < kCorePidCount ? STALE_MS : kExtraFreshMs;
    if (has) *has = g_link == LINK_CONNECTED && s_pid[i].hasVal &&
                    (isSupportPage(s_pid[i].pid) || millis() - s_pid[i].lastOkMs < freshMs);
    return s_pid[i].lastVal;
}

/* ================= 诊断输出 ================= */
void obdString(char *out, size_t n)
{
    if (!out) return;
    snprintf(out, n, "%s %s", s_devName[0] ? s_devName : "-", s_devAddr[0] ? s_devAddr : "-");
}

/* ================= 诊断版 getter（只读观测） ================= */
uint32_t obdNotifyCount() { return s_notifyCount; }
uint32_t obdFirstByteMs() { return s_firstByteMs; }
uint32_t obdLastProbeMs() { return s_lastProbeMs; }
uint8_t  obdNotifyFlag()  { return s_notifyOn ? 1 : 0; }

const char *obdProbeStr()
{
    switch (s_probeState) {
    case PR_OK:       return "valid";     /* 得到有效支持位图 */
    case PR_SENT:     return "sent";
    case PR_RECV:     return "recv";
    case PR_NODATA:   return "nodata";    /* 明确 NO DATA */
    case PR_SEARCH:   return "search";    /* SEARCHING / ? */
    case PR_BADFRM:   return "unparse";   /* 有原文但解析不出 */
    case PR_TIMEOUT:  return "timeout";   /* 本期限内无有效响应 */
    case PR_SENDFAIL: return "sendfail";
    default:          return "none";      /* 尚未发生 */
    }
}

/* ---- 阶段进度 getter ---- */
uint8_t obdPhaseCount() { return (uint8_t)PH_N; }
uint8_t obdPhaseNo()
{
    const uint8_t ph = calcPhase();
    return (ph < PH_N) ? (uint8_t)(ph + 1) : 0;
}
const char *obdPhaseName() { const uint8_t ph = calcPhase(); return ph < PH_N ? kPhName[ph] : "?"; }
const char *obdPhaseCode() { const uint8_t ph = calcPhase(); return ph < PH_N ? kPhCode[ph] : "?"; }
uint32_t obdPhaseMs()      { const uint8_t ph = calcPhase();
                             return (ph == s_ph) ? (uint32_t)(millis() - s_phSince) : 0; }
const char *obdPhaseResultStr()
{
    /* 阶段已变但结果还是上一阶段的 → 视为「无结论」，避免张冠李戴 */
    const uint8_t ph = calcPhase();
    if (ph != s_ph) return kPhResCn[PR_NONE];
    return kPhResCn[(s_phRes < PR_N) ? s_phRes : 0];
}
const char *obdPhaseResultCode()
{
    const uint8_t ph = calcPhase();
    if (ph != s_ph) return kPhResStr[PR_NONE];
    return kPhResStr[(s_phRes < PR_N) ? s_phRes : 0];
}
const char *obdLastNote()    { return s_phNote; }
const char *obdConnStepStr()
{
    const uint8_t cs = s_connStep;
    return (cs < CS_N) ? kCsStr[cs] : "?";
}
bool obdPidBusy()            { return s_waiting && !s_sentIsAt; }
const char *obdSentCmd()     { return s_sentCmd; }
uint8_t     obdLastFailPhase() { return s_failPh; }
const char *obdLastFailNote()  { return s_failCn; }
uint8_t     obdFirstFailPhase(){ return s_firstFailPh; }
const char *obdFirstFailNote() { return s_firstFailCn; }
uint32_t    obdStageEvents()   { return s_phEvents; }

const char *obdAtCmdName(int i)
{
    static const char *k[3] = { "atz", "ate0", "ath1" };
    return (i >= 0 && i < 3) ? k[i] : "?";
}
const char *obdAtResultStr(int i)
{
    static const char *k[4] = { "none", "ok", "to", "wfail" };
    if (i < 0 || i > 2) return "?";
    return k[(s_atRes[i] < 4) ? s_atRes[i] : 0];
}

const char *obdLastPidStr(char *out, size_t n)
{
    if (!out || n == 0) return "";
    if (!s_lastPidCmd[0]) { snprintf(out, n, "无"); return out; }
    const char *rs = kPhResCn[(s_lastPidRes < PR_N) ? s_lastPidRes : 0];
    if (s_lastPidRes == PR_OK) {
        const char *unit = "";
        for (int i = 0; i < V2_PID_COUNT; i++) {
            if (!strcmp(s_pid[i].cmd, s_lastPidCmd)) { unit = s_pid[i].unit; break; }
        }
        snprintf(out, n, "%s %s %.0f%s", s_lastPidCmd, rs, s_lastPidVal, unit);
    } else {
        snprintf(out, n, "%s %s", s_lastPidCmd, rs);
    }
    return out;
}

void obdUuidStr(char *out, size_t n)
{
    if (!out || n == 0) return;
    snprintf(out, n, "w=%s n=%s", s_wUuid[0] ? s_wUuid : "-", s_nUuid[0] ? s_nUuid : "-");
}

void obdDiagJson()
{
    Serial.printf("\n{\"ev\":\"obd-diag\",\"mode\":\"%s\",\"link\":\"%s\",\"svc\":\"%s\","
                  "\"ecu\":\"%s\",\"addr\":\"%s\",\"name\":\"%s\",\"atype\":%u,"
                  "\"svc_n\":%d,\"chars\":%d,\"notify\":%d,\"ready\":%d,\"at_step\":%d,"
                  "\"real_req\":%lu,\"real_ok\":%lu,\"real_nodata\":%lu,\"real_to\":%lu,"
                  "\"stale_ms\":%lu,\"cand\":%d,\"scan_n\":%d,\"saved\":%d,"
                  "\"nfy\":%lu,\"fb_ms\":%lu,\"p0100\":\"%s\",\"p0100_ms\":%lu,"
                  "\"w_uuid\":\"%s\",\"n_uuid\":\"%s\",\"reg_ok\":\"UNKNOWN\",\"cccd\":\"UNKNOWN\","
                  "\"ph\":%u,\"of\":%u,\"ph_code\":\"%s\",\"ph_st\":\"%s\",\"ph_ms\":%lu,"
                  "\"conn\":\"%s\",\"pid_busy\":%d,\"pid_cmd\":\"%s\","
                  "\"fail\":%u,\"fail_note\":\"%s\",\"fail1\":%u,\"fail1_note\":\"%s\","
                  "\"atz\":\"%s\",\"ate0\":\"%s\",\"ath1\":\"%s\",\"ph_ev\":%lu}\n",
                  g_mode == MODE_SIM ? "SIM" : "REAL",
                  obdLinkStr(), obdSvcStr(), obdEcuStr(),
                  s_devAddr, s_devName, (unsigned)s_savedType,
                  s_svcCount, s_svcChars, s_notifyOn ? 1 : 0, s_ready ? 1 : 0, s_atStep,
                  (unsigned long)g_realReq, (unsigned long)g_realOk,
                  (unsigned long)g_realNoData, (unsigned long)g_realTo,
                  (unsigned long)obdStaleMs(), obdCandCount(), s_scanN, s_savedBound ? 1 : 0,
                  (unsigned long)s_notifyCount, (unsigned long)s_firstByteMs,
                  obdProbeStr(), (unsigned long)s_lastProbeMs,
                  s_wUuid[0] ? s_wUuid : "-", s_nUuid[0] ? s_nUuid : "-",
                  (unsigned)obdPhaseNo(), (unsigned)PH_N,
                  obdPhaseCode(), obdPhaseResultCode(), (unsigned long)obdPhaseMs(),
                  obdConnStepStr(), obdPidBusy() ? 1 : 0, s_sentCmd,
                  (unsigned)s_failPh, s_failCn, (unsigned)s_firstFailPh, s_firstFailCn,
                  obdAtResultStr(0), obdAtResultStr(1), obdAtResultStr(2),
                  (unsigned long)s_phEvents);
}
