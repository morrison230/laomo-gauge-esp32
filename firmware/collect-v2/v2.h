/*
 * v2.h —— collect v2 的共享类型 / 常量 / 跨文件函数声明
 *
 * 为什么拆文件：V1 是单文件 1261 行，V2 要加 BLE 设备绑定、Wi-Fi、诊断页，
 * 单文件会到 2500 行以上难以维护。Arduino 会把同目录的 .ino 按
 * 「主文件优先、其余按字母序」拼成一个编译单元，所以这里集中放声明，
 * 各实现文件只写函数体，不重复定义全局变量。
 *
 * ★ 三态分离（V1 把它们混成一个 g_bleState，导致"看起来连上了"）
 *   mode    REAL / SIM          数据来源
 *   link    BLE 链路状态
 *   svc     OBD 服务（FFE0/FFE1）是否识别
 *   ecu     是否真的收到 41 xx 有效帧
 *   四者独立记录、独立显示。
 */
#pragma once

#include <Arduino.h>

#define V2_APP_VERSION "collect v2 r6-sensors"

/* ================= 引脚 / 频率（与已验证板子一致，不得随意改） ================= */
#define PIN_SD_CS     15
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23
#define PIN_LCD_CS    5
#define PIN_LCD_DC    27
#define PIN_LCD_BL    25
#define PIN_BOOT      0
#define PIN_GPS_RX    32

#define GPS_BAUD      9600
#define SERIAL_BAUD   921600
#define SD_FREQ_HZ    20000000UL
#define SD_MOUNT      "/sd"
#define TRIPS_VFS     "/trips"

/* ================= 时序 ================= */
#define FLUSH_MS        500      /* 周期落盘：V1 是 1000，V2 收紧到 500（缩小断电丢失窗口） */
#define FLUSH_BYTES     3072
#define UI_PERIOD_MS    500
#define OBD_AT_TIMEOUT_MS 1200    /* ATE0 and ATH1 use the same window as a normal read */
#define OBD_PID_TIMEOUT_MS 1200   /* 轮询的候选值，待实车验证 */
#define OBD_DRAIN_MS 2000         /* 超时后给旧请求的提示符留出独立窗口 */
#define OBD_PROBE_TIMEOUT_MS 3000
#define ATZ_TIMEOUT_MS  2000     /* 山寨 ELM327 复位可能要 1~2 秒 */
/* ★ 连上之后到发第一条 AT 之间的等待。
   作用只是「让『通知注册』这一阶段在屏幕上有停留时间、便于观察」；
   它**不保证** CCCD 写入成功，也不代表链路已经稳定。 */
#define AT_SETTLE_MS    600
/* ★ 连接步骤的显示停留（CS_CONNECTED / CS_CHARS_OK / CS_CHARS_MISS）。
   只为可观察，不参与逻辑判断；同时也保证主循环至少观测到一次该阶段。 */
#define CONN_DWELL_MS   300
#define BIND_DWELL_MS   300      /* 「写绑定」阶段的显示停留 */
/* 超时前接收片段留证上限：只记前 N 次、每次最多 M 字节，避免 trip 文件膨胀 */
#define TIMEOUT_SNAP_MAX   20
#define TIMEOUT_SNAP_LEN   96
/* 单条 rx 事件里原文的转义后长度上限（超出截断并标 raw_trunc=1） */
#define RX_ESC_MAX      620
/* 每次连接最多记录多少条「特征清单」（有界，避免 16 个特征×随机 UUID 撑爆卡） */
#define BLE_CHAR_MAX    10
#define OBD_MIN_GAP_MS  120      /* 两次请求之间的最小间隔，别把适配器问死 */
#define STALE_MS        5000     /* 超过这个时间没有有效响应 → 数值标 STALE */
#define BLE_SCAN_MS     6000
#define BLE_RECONN_MS   8000

/* ================= 状态机 ================= */
enum V2Mode : uint8_t { MODE_REAL = 0, MODE_SIM = 1 };

enum V2Link : uint8_t {
    LINK_IDLE = 0, LINK_SCANNING, LINK_CONNECTING, LINK_CONNECTED, LINK_FAILED
};

enum V2Svc : uint8_t { SVC_UNKNOWN = 0, SVC_FOUND, SVC_MISSING };

enum V2Ecu : uint8_t { ECU_NONE = 0, ECU_NODATA, ECU_VALID };

/* 五个页面（触摸校准不再是独立页面，收进「高级诊断」作为二级功能） */
enum V2Page : uint8_t {
    PAGE_MAIN = 0,   /* 主界面：四项数值 + 车辆/定位/存储/网络 */
    PAGE_DEV,        /* 蓝牙设备：分页选设备并连接 */
    PAGE_WIFI,       /* 无线网络：分页选网络 */
    PAGE_DIAG,       /* 系统诊断：中文状态清单 */
    PAGE_ADV,        /* 高级诊断：PID 表 + 技术字段 + 触摸校准入口 */
    PAGE_N
};

/* 按钮/命中区定义。绘制与命中判定共用同一张表，杜绝"画在这、点在那"。
   放在头文件里是因为 Arduino 会在 .ino 顶部插入函数原型，结构体必须先可见。 */
struct BtnDef {
    int16_t     x, y, w, h;
    uint8_t     id;      /* HitId */
    uint8_t     style;   /* 0 普通 1 主操作 2 危险 */
    const char *label;
};

/* 触摸命中 ID。0 = 未命中；行选中用 HIT_ROW_BASE + 行号 */
/* ================= OBD 连接阶段（2026-11-14 r2 修正版） =================
   阶段号用于屏幕与 SD 事件，序号从 1 开始对外显示。
   ★ 阶段由「连接步骤 s_connStep（bleTask 发布）+ 握手变量」派生，不另存一份状态。
   ★ 每一个阶段都必须在真实控制流里能被观测到（≥ DWELL 窗口），
     否则就不该写进这张表 —— 上一版就有 3 个永远显示不出来的阶段，已修。 */
enum ObdPhase : uint8_t {
    PH_WAIT = 0,      /*  1 等待开始 / 等待扫描 */
    PH_SCAN,          /*  2 正在扫描 */
    PH_FOUND,         /*  3 已找到并选中适配器（等待连接） */
    PH_CONN,          /*  4 BLE 连接中（client->connect 阻塞中） */
    PH_CONN_FAIL,     /*  5 BLE 连接失败（含服务缺失后的断开、被对端断开） */
    PH_CONN_OK,       /*  6 链路已建立，尚未开始 GATT 发现 */
    PH_SVC,           /*  7 GATT 服务/特征发现中（getServices 阻塞中） */
    PH_SVC_OK,        /*  8 服务与读写特征已发现 */
    PH_SVC_MISS,      /*  9 服务或读写特征缺失（停留后才断开） */
    PH_NOTIFY,        /* 10 通知注册调用已执行（等 AT 稳定窗口） */
    PH_ATZ,           /* 11 ELM 初始化：ATZ */
    PH_ATE0,          /* 12 ELM 初始化：ATE0 */
    PH_ATH1,          /* 13 ELM 初始化：ATH1 */
    PH_PROBE,         /* 14 0100 探针 */
    PH_BIND,          /* 15 写入设备绑定（NVS） */
    PH_POLL,          /* 16 轮询：握手完成但数据面未确认/已陈旧 */
    PH_READY,         /* 17 就绪：握手完成且最近仍有有效帧 */
    PH_N
};

/* 阶段结果码。★ 这里刻意把「请求已提交」「收到部分字节」「收到有效响应」拆成三档：
   请求已发 ≠ 已收到字节 ≠ 已解析出有效帧 ≠ ECU 应答正确。 */
enum PhResult : uint8_t {
    PR_NONE = 0,      /* 无结论（阶段刚开始，尚未发命令） */
    PR_SENT,          /* 请求已提交给本地 BLE 库，等待响应 */
    PR_RECV,          /* 已收到部分字节，尚无完整帧（'>' 未到） */
    PR_OK,            /* 收到有效响应（AT 有应答 / 0100 位图有效 / PID 解码通过） */
    PR_TIMEOUT,       /* 本期限内未得到有效响应 */
    PR_NODATA,        /* 解析器读到 NO DATA */
    PR_SEARCH,        /* 解析器读到 SEARCHING / ? */
    PR_BADFRM,        /* 收到了原文，但无法解析成有效帧（格式不符） */
    PR_SENDFAIL,      /* 本地 writeValue 返回失败：请求可能没提交 */
    PR_N
};

enum HitId : uint8_t {
    HIT_NONE = 0,
    HIT_MAIN_DEV = 1, HIT_MAIN_WIFI, HIT_MAIN_DIAG,
    HIT_DEV_PREV, HIT_DEV_NEXT, HIT_DEV_RESCAN, HIT_DEV_CONNECT, HIT_DEV_BACK,
    HIT_WIFI_PREV, HIT_WIFI_NEXT, HIT_WIFI_SCAN, HIT_WIFI_CONNECT, HIT_WIFI_BACK,
    HIT_DIAG_ADV, HIT_DIAG_BACK,
    HIT_ADV_CAL, HIT_ADV_BACK,
    HIT_CAL_START, HIT_CAL_CANCEL,
    /* ★ 诊断版新增（2026-11-14）：诊断页底部中键 = 保存并封存行程。
       追加在末尾，所有既有 hit id 数值保持不变。 */
    HIT_DIAG_SAVE, HIT_ADV_PIDS
};
#define HIT_ROW_BASE 0x40

enum V2Sd : uint8_t {
    SD_NONE = 0, SD_MOUNTED, SD_REC, SD_SAVED, SD_ERROR, SD_VERIFY, SD_RECOVER
};

/* ================= PID 表 ================= */
struct PidRec {
    const char *cmd;      /* "010C" */
    uint8_t     pid;      /* 0x0C */
    uint8_t     nbytes;   /* 数据字节数（不含 0x41/pid） */
    const char *label;    /* "RPM" */
    const char *unit;     /* "rpm" */
    float       scale;
    float       offset;
    float       minV, maxV;
    uint16_t    periodMs;
    /* 运行时统计（REAL 与 SIM 共用同一张表，但 org 不同时不会互相累加） */
    uint32_t    req, ok, nodata, badframe, to;
    uint32_t    lastOkMs;
    float       lastVal;
    bool        hasVal;
    uint8_t     supported;   /* 0 未知 / 1 支持 / 2 不支持 */
    uint8_t     lastResult;
    int16_t     lastEcu;     /* last valid response, -1 for a headerless frame */
};

#define V2_PID_COUNT 15

/* ================= BLE 候选设备 ================= */
struct BleCand {
    bool     used;
    char     addr[20];    /* "aa:bb:cc:dd:ee:ff" */
    uint8_t  atype;       /* BLE_ADDR_PUBLIC / RANDOM —— 必须一起存，否则重连会失败 */
    char     name[33];
    int8_t   rssi;
    bool     hasFfe0;
    uint32_t lastMs;
};

#define V2_MAX_CAND 16

/* OBD 应答里的一帧。放在头文件里是因为 Arduino 会在 .ino 顶部自动插入
   函数原型，若结构体定义在实现文件中间，原型就会引用到未声明的类型。 */
struct Frame {
    int     ecu;      /* 0x7E8 / 0x7E9 / -1 表示无头 */
    uint8_t d[8];
    int     n;
};

/* ================= 全局（全部定义在 collect-v2.ino） ================= */
extern V2Mode  g_mode;
extern V2Link  g_link;
extern V2Svc   g_svc;
extern V2Ecu   g_ecu;
extern V2Page  g_page;
extern V2Sd    g_uisd;

extern uint32_t g_sess;        /* 本次上电的会话号 */
extern uint32_t g_bootMs;
extern char     g_resetWhy[20];

/* REAL / SIM 计数器物理分开，绝不互相累加 */
extern uint32_t g_realReq, g_realOk, g_realNoData, g_realTo;
extern uint32_t g_simReq,  g_simOk;

/* ================= a_store.ino ================= */
bool  storeMount();
bool  storeOpenTrip();
void  storeTick();
void  storeFinishAndSave();
void  storeRecoverCheck();
bool  storeFlush();
bool  jsonValidate(const char *s);
void  logJson(const char *line);
void  logJsonf(const char *fmt, ...);
uint32_t storeLines();
uint32_t storeBytes();
uint32_t storeErrWrites();
const char *storePathVfs();
bool  storeIsRecording();
bool  storeOk();
void tripExportCommand(const char *args);
void tripExportTick();

/* ================= b_gps.ino ================= */
void gpsBegin();
void gpsTick();
int  gpsFixQ();
int  gpsSats();
void gpsStr(char *out, size_t n);    /* out=nullptr 时只往串口打一行状态 */
void gpsSimSet(int q, int sat);      /* SIM 模式下伪造定位状态（只改 UI 显示值） */

/* ================= c_obd.ino ================= */
void obdFeedRaw(const char *cmd, const String &raw, uint32_t ms, bool isSim);
float obdVal(int i, bool *has, bool *isSim);   /* 按当前 mode 返回 real 或 sim 的数值 */

/* ================= c_obd.ino ================= */
void  obdBegin();                    /* 仅 REAL 模式调用；会起 BLE 任务 */
void  obdTick();                     /* 主循环：请求/应答/解码 */
void  obdRequestScan();
int   obdCandCount();
const BleCand *obdCand(int i);
int   obdPickIdx();
void  obdPickNext();                 /* 光标下移（UI 用） */
void  obdPickToIndex(int i);
void  obdSelectCurrent();            /* 连接光标所在设备 */
void  obdConnectSaved();
void  obdForget();
void  obdDisconnect();
void  obdRetry();
void  obdServiceHandshakeTick();     /* 供主循环驱动 AT 握手 */
const char *obdLinkStr();
const char *obdSvcStr();
const char *obdEcuStr();
const PidRec *obdPid(int i);
int   obdPidCount();
const char *obdPidResultStr(const PidRec &pid);
uint32_t obdStaleMs();               /* 距最后一次有效 ECU 帧的毫秒数 */
bool  obdSavedBound();
void  obdString(char *out, size_t n);/* 当前设备 "name addr rssi" */
void  obdDiagJson();
/* ---- 诊断版新增（只读观测，不改变通信行为） ---- */
uint32_t obdNotifyCount();        /* 通知回调累计触发次数（订阅是否真的生效的实证） */
uint32_t obdFirstByteMs();        /* 最近一次请求「首字节到达」距发送的毫秒数；0=未观测到 */
uint32_t obdLastProbeMs();        /* 最近一次 0100 探针的判定耗时；0=未发生 */
const char *obdProbeStr();        /* 0100 探针结果：none/sent/recv/valid/nodata/search/unparse/timeout/sendfail */
uint8_t  obdNotifyFlag();         /* 1=已调用 registerForNotify（不代表对端已启用、不代表 CCCD 写成功） */
void     obdUuidStr(char *out, size_t n);  /* "w=<写UUID> n=<通知UUID>" */
int      obdRxLen();              /* 当前接收缓冲字节数（只用于屏幕显示） */

/* ---- 阶段进度（2026-11-14 r2 修正版；只在变化时写 SD 事件） ---- */
uint8_t     obdPhaseNo();       /* 1..obdPhaseCount()，0=未知 */
uint8_t     obdPhaseCount();    /* 阶段总数 */
const char *obdPhaseName();     /* 当前阶段中文名 */
const char *obdPhaseCode();     /* 当前阶段英文短码（写事件/SD 用） */
uint32_t    obdPhaseMs();       /* 当前阶段已持续毫秒 */
const char *obdPhaseResultStr();/* 当前阶段结果中文：无结论/已发送/已收到/成功/超时/无数据/搜索中/无法解析/发送失败 */
const char *obdPhaseResultCode();/* 当前阶段结果英文码（SD 用） */
const char *obdLastNote();      /* 最近一次命令/原因（可能为空串） */
uint8_t     obdLastFailPhase(); /* 最后失败阶段号（1..N，0=无） */
const char *obdLastFailNote();  /* 最后失败原因 */
uint8_t     obdFirstFailPhase();/* 本次上电首个失败阶段号（0=无） */
const char *obdFirstFailNote(); /* 本次上电首个失败原因 */
uint32_t    obdStageEvents();   /* 已写出的阶段事件条数 */
const char *obdConnStepStr();   /* 连接步骤：idle/connecting/connected/discover/chars_ok/chars_miss/notify_done */
/* 三条 AT 命令各自的结果：0=none 1=ok 2=timeout 3=sendfail */
const char *obdAtResultStr(int i);
const char *obdAtCmdName(int i);
/* 最近一次 PID 轮询：写入 out，返回 out */
const char *obdLastPidStr(char *out, size_t n);
/* 当前是否有 PID 请求在飞（只读；用于屏幕「等待中」） */
bool        obdPidBusy();
const char *obdSentCmd();       /* 最近一次发出的命令 */
/* 最近一次应答的分类：none/valid/nodata/search/unparse/empty */
const char *obdLastRxCls();
void  obdSetSavedAddrForTest();      /* 占位，不使用 */

/* ================= d_net.ino ================= */
void netBegin();
void netScan();
int  netScanCount();
void netScanList(char *out, size_t n);
/* 扫描列表取值：任一 out 可为 nullptr 表示不需要 */
void netApGet(int i, char *ssid, size_t n, int *rssi, unsigned *ch);
const char *netSsid();
void netSetCred(const char *ssid, const char *pass);
void netConnect();
void netDisconnect();
void netStatusJson();
void netTest();
bool netHasCred();
const char *netStateStr();
const char *netIpStr();
int  netRssi();
void netTaskStart();

/* ================= e_ui.ino ================= */
bool uiBegin();
void uiDraw(bool force);
void uiPhasePaint();                 /* 只重画「阶段区」（诊断页/高级页顶部与底部几行），不整屏重画 */
void uiNextPage();
void uiAction();                     /* 长按：执行当前页动作 */
void uiToast(const char *msg, uint16_t ms);
void uiTick();
void uiMarkMove(int16_t x, int16_t y);   /* 记录最近一次点击位置（TOUCH MARK 十字用） */
void uiDrawStats();       /* 绘制计数：整屏重画 / 局部重画 / 页面切换 */
void uiDrawStatsReset();
uint8_t uiTouch(int16_t x, int16_t y);   /* 触摸点击：返回命中的 HitId，0 = 未命中 */
const char *uiHitName(uint8_t id);       /* 命中名（串口日志用） */
void uiMarkSet(bool on);                 /* 触摸十字标记（现场调试用） */

/* ================= f_touch.ino（FT6336） ================= */
bool touchBegin();
bool touchReady();
bool touchTick(int16_t *sx, int16_t *sy);   /* 抬起瞬间返回 true + 屏幕坐标 */
void touchLoadCal();
void touchSaveCalRaw(int16_t rxTL, int16_t ryTL, int16_t rxBR, int16_t ryBR);
void touchForgetCal();
void touchLastRaw(int16_t *rx, int16_t *ry);
void touchDiagJson();
int  touchScanBus(char *out, size_t n);
bool touchCalValid();
uint32_t touchTaps();                    /* 累计点击次数（诊断页显示） */
void touchLogAdd(int16_t rx, int16_t ry, int16_t sx, int16_t sy, uint8_t page, uint8_t hit);
void touchLogDump();                     /* 串口打印最近 24 次点击 */
