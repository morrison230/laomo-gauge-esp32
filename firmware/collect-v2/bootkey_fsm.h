/* bootkey_fsm.h —— BOOT 键（GPIO0）轻触/长按状态机
 *
 * 为什么单独抽出来：
 *   GPIO0 在这块板子上接的是 CH343 自动下载电路（DTR 经三极管拉低），
 *   上电、复位、串口开关端口都会产生毫秒级 LOW 毛刺。原来的 bootKeyTick()
 *   没有任何消抖——这一次读到 LOW、下一次读到 HIGH 就算一次短按，于是
 *   "没人碰也会自己翻页"。把判定逻辑抽成不依赖 Arduino API 的纯状态机，
 *   一方面固件里只有一处实现，另一方面可以在 PC 上做模型测试。
 *
 * 规则（与用户确认的口径一致）：
 *   1. 开机 maskMs 内完全屏蔽，并持续把内部状态复位成"未按下"
 *   2. 屏蔽结束后，必须先连续 stableMs 读到 HIGH，才建立"空闲基线"(armed)
 *   3. LOW 连续 stableMs 才认定按下；HIGH 连续 stableMs 才认定释放
 *   4. 释放时按住时长 < shortMaxMs 且长按未触发 → 返回 1（短按）
 *   5. 按住 >= longMs → 返回 2（长按），一次按压只返回一次
 *   6. 长按途中的瞬时 HIGH 毛刺不取消操作，必须走"稳定释放"判定
 *
 * 返回：0 = 无事件，1 = 短按，2 = 长按
 */
#pragma once
#include <stdint.h>

struct BootKeyFsm {
    /* ---- 阈值：与旧实现的 400ms / 800ms 操作习惯保持一致 ---- */
    uint32_t maskMs     = 3000;   /* 开机屏蔽 */
    uint32_t stableMs   = 40;     /* 稳定电平判定窗口 */
    uint32_t shortMaxMs = 400;    /* 短按上限（旧值） */
    uint32_t longMs     = 800;    /* 长按阈值（旧值） */

    /* ---- 状态 ---- */
    bool     sampleInit  = false;  /* 是否已拿到第一个采样 */
    bool     rawPrev     = false;  /* 上一次原始电平（true = 低 = 按下） */
    uint32_t rawChangeMs = 0;      /* 原始电平最后一次变化的时刻 */
    bool     armed       = false;  /* 已建立空闲基线（见过稳定 HIGH） */
    bool     down        = false;  /* 已确认按下 */
    uint32_t downAtMs    = 0;      /* 本次按下开始时刻（原始 LOW 起点） */
    bool     longFired   = false;  /* 本次按压的长按事件是否已发出 */

    void begin(uint32_t nowMs)
    {
        sampleInit  = false;
        rawPrev     = false;
        rawChangeMs = nowMs;
        armed       = false;
        down        = false;
        downAtMs    = 0;
        longFired   = false;
    }

    /* rawLow：本次采样 GPIO0 是否为低（低有效）
       nowMs ：调用方的 millis()                                          */
    int tick(bool rawLow, uint32_t nowMs)
    {
        if (!sampleInit) { sampleInit = true; rawPrev = rawLow; rawChangeMs = nowMs; }

        /* ① 开机屏蔽期：只跟踪电平，绝不产生事件，也不允许建立基线 */
        if (nowMs < maskMs) {
            if (rawLow != rawPrev) { rawPrev = rawLow; rawChangeMs = nowMs; }
            armed     = false;
            down      = false;
            longFired = false;
            return 0;
        }

        /* ② 电平刚变化：计时重来，本次不判定（抖动全在这里被吃掉） */
        if (rawLow != rawPrev) { rawPrev = rawLow; rawChangeMs = nowMs; return 0; }

        /* ③ 还没稳定够 stableMs：继续等 */
        if ((uint32_t)(nowMs - rawChangeMs) < stableMs) return 0;

        /* ④ 屏蔽后的第一个稳定 HIGH 才建立空闲基线；
              开机就是 LOW（被按着 / 被 DTR 拉住）时永远不 arm，不会误判 */
        if (!armed) {
            if (!rawLow) armed = true;
            return 0;
        }

        /* ⑤ 稳定 LOW：按下 / 长按计时 */
        if (rawLow) {
            if (!down) { down = true; downAtMs = rawChangeMs; longFired = false; }
            if (!longFired && (uint32_t)(nowMs - downAtMs) >= longMs) {
                longFired = true;
                return 2;
            }
            return 0;
        }

        /* ⑥ 稳定 HIGH：释放
           ★ 按压时长必须量到"释放那一刻"，而不是"释放被确认的那一刻"。
             走到这里说明这个 HIGH 已经连续稳定 stableMs，而 rawChangeMs 在每次
             电平变化时都会被重写 —— 所以它必然是**这次**释放的起始时间。
             如果中途出现过短暂 HIGH 毛刺又回到 LOW，那次 rawChangeMs 已经被
             LOW 的那次变化覆盖掉了，不会拿旧时间去算时长。                  */
        if (down) {
            const uint32_t dur = (uint32_t)(rawChangeMs - downAtMs);
            const bool wasLong = longFired;
            down      = false;
            longFired = false;
            if (!wasLong && dur < shortMaxMs) return 1;   /* 短按只在释放时发一次 */
        }
        return 0;
    }
};
