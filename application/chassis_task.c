#include "chassis_task.h"

#include <stddef.h>
#include <stdio.h>

#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "blackbox.h"
#include "chassis_config.h"
#include "Delay.h"
#include "distance_sensor.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "odometry.h"
#include "oled.h"
#include "params.h"
#include "vision_link.h"

/* STOP 的确认时间必须短于链路超时：否则「连续看到人」还没攒满，
 * 链路就先判超时（fresh=0），永远退不出搜索。 */
#if (CHASSIS_SEARCH_HOLD_MS >= CHASSIS_LINK_TIMEOUT_MS)
#error "CHASSIS_SEARCH_HOLD_MS must be < CHASSIS_LINK_TIMEOUT_MS"
#endif

/* 盲转差速不能超过总差速上限，否则等于没限。 */
#if (CHASSIS_FOLLOW_BLIND_STEER > CHASSIS_FOLLOW_STEER_MAX)
#error "CHASSIS_FOLLOW_BLIND_STEER must be <= CHASSIS_FOLLOW_STEER_MAX"
#endif

/* ============================================================================
 * 在线调参：把宏名重定向到运行期参数表
 *
 * 为什么这么做：控制律里有十几处直接引用 CHASSIS_FOLLOW_* 宏。
 * 如果逐处改成 Params_Get()，改动面大、容易漏、以后也不好对照文档。
 * 这里在「编译期检查全部做完之后」把宏名重新定义为函数调用，
 * 于是下面控制律代码一个字不改，读到的却是运行期值。
 *
 * 注意：
 *   1) 必须放在上面两个 #if 之后 —— #if 里不能用函数调用；
 *   2) 只重定向"现场要整定"的参数，安全相关的不在其中；
 *   3) PARAM_LIVE = 0 时整块失效，等于回到纯宏模式（一键回滚）。
 * ========================================================================== */
#if (PARAM_LIVE != 0)
#undef  CHASSIS_FOLLOW_STEER_GAIN_Q
#define CHASSIS_FOLLOW_STEER_GAIN_Q           (Params_Get(PARAM_ID_SG))
#undef  CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG
#define CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG (Params_Get(PARAM_ID_SDB))
#undef  CHASSIS_FOLLOW_STEER_MAX
#define CHASSIS_FOLLOW_STEER_MAX              (Params_Get(PARAM_ID_SMAX))
#undef  CHASSIS_FOLLOW_DISTANCE_GAIN_Q
#define CHASSIS_FOLLOW_DISTANCE_GAIN_Q        (Params_Get(PARAM_ID_DG))
#undef  CHASSIS_FOLLOW_SPEED_MAX
#define CHASSIS_FOLLOW_SPEED_MAX              (Params_Get(PARAM_ID_VMAX))
#undef  CHASSIS_FOLLOW_BASE_SPEED
#define CHASSIS_FOLLOW_BASE_SPEED             (Params_Get(PARAM_ID_BASE))
#undef  CHASSIS_FOLLOW_DISTANCE_DEADBAND_CM
#define CHASSIS_FOLLOW_DISTANCE_DEADBAND_CM   (Params_Get(PARAM_ID_DDB))
#undef  CHASSIS_FOLLOW_STEER_KD
#define CHASSIS_FOLLOW_STEER_KD               (Params_Get(PARAM_ID_KD))
#undef  CHASSIS_FOLLOW_TURN_ONLY_TENTH_DEG
#define CHASSIS_FOLLOW_TURN_ONLY_TENTH_DEG    (Params_Get(PARAM_ID_TO))
#undef  CHASSIS_LINE_STEER_GAIN
#define CHASSIS_LINE_STEER_GAIN               (Params_Get(PARAM_ID_LG))
#undef  CHASSIS_LINE_BASE_SPEED
#define CHASSIS_LINE_BASE_SPEED               (Params_Get(PARAM_ID_LV))
#undef  CHASSIS_FOLLOW_BLIND_DEV_TENTH_DEG
#define CHASSIS_FOLLOW_BLIND_DEV_TENTH_DEG    (Params_Get(PARAM_ID_BD))
#undef  CHASSIS_FOLLOW_BLIND_STEER
#define CHASSIS_FOLLOW_BLIND_STEER            (Params_Get(PARAM_ID_BS))
#undef  CHASSIS_LINE_SLOW_GAIN
#define CHASSIS_LINE_SLOW_GAIN                (Params_Get(PARAM_ID_LS))
#undef  CHASSIS_LINE_LOST_HOLD_MS
#define CHASSIS_LINE_LOST_HOLD_MS             ((uint32_t)Params_Get(PARAM_ID_LH))
#undef  CHASSIS_LINE_STEER_KD
#define CHASSIS_LINE_STEER_KD                 (Params_Get(PARAM_ID_LKD))
#undef  CHASSIS_LINE_CROSS_MS
#define CHASSIS_LINE_CROSS_MS                 ((uint16_t)Params_Get(PARAM_ID_CRS))
#endif

/* 当前模式 */
static ChassisMode_t s_mode = CHASSIS_MODE_LINE_TRACK;

/* 蓝牙命令：s_link_cmd 记住上一帧的命令码（做边沿检测），
 * s_remote_stop = 1 表示收到过 stop，整车停住直到再来 mode1/mode2 或按键。 */
static uint8_t s_link_cmd = 0U;
static uint8_t s_remote_stop = 0U;

/* ---------- 提示音（0x05 帧）的边沿检测状态 ----------
 * 蜂鸣器在云台板，这里只负责"什么时候该响"。
 * 全部按上沿触发：状态没变就不发帧，否则一直丢线会响成一片。 */
static uint8_t s_beep_remote_stop_prev = 1U;
/* 模式上报（0x06）用：0xFF 表示"还没发过"，配合 1 s 心跳 */
static uint8_t s_mode_reported = 0xFFU;
static uint32_t s_mode_report_ms = 0U;
static uint32_t s_wave_ms = 0U;          /* 上次发波形帧的时刻 */  /* 上电即 STOP，首次解锁才算"起跑" */
static uint8_t s_beep_link_prev = 0U;         /* 上一拍链路是否在线 */
static uint8_t s_beep_line_lost = 0U;         /* 本次脱线是否已响过 */
static uint16_t s_line_black_ms = 0U;         /* 7 路全黑已持续多久 */
static uint8_t s_line_black_beeped = 0U;      /* 本次全黑是否已响过（防重复长鸣） */
/* 黑匣子（方案 B）：100 ms 采一条进 RAM，停车时整块落盘 */
static uint32_t s_bb_ms = 0U;
static uint8_t s_bb_flushed = 0U;
static uint8_t s_line_arrived = 0U;           /* 十字路口已停车锁存（按 PB13/蓝牙解锁） */
static uint8_t s_line_armed = 1U;             /* 0 = 需先见到白色才允许再次判定十字路口 */

/* 节流基准 */
static uint32_t s_last_display_ms = 0U;
static uint32_t s_last_debug_ms = 0U;

/* 按键消抖状态 */
static uint8_t s_key_last_raw = 0U;
static uint8_t s_key_stable_down = 0U;
static uint32_t s_key_last_change_ms = 0U;

/* ------------------------------------------------- 模式 2：丢人搜索状态
 *
 * 只有「链路在线（VisionLink_IsReceiving）= 1」且「没人（IsFresh）= 0」
 * 才进入搜索；断链直接停车、状态清零，因为线松了转圈没有意义。
 *
 * 枚举值刻意与串口调试字段 srch=%d 的约定一一对应：
 *   0 = 不在搜索（正常跟随 / 断链 / 确认期）
 *   1 = 正在转圈（SEARCHING）
 *   2 = 转圈之间暂停（PAUSING）
 *   3 = 缓慢搜索（SLOW，快搜轮数用尽后持续原地慢转）
 *   4 = 发现目标、停车确认中（STOP：人已出现在画面里，先整车静止 HOLD_MS）
 *
 * STOP 的意义：转圈时 K230 画面会抖，检测时有时无。若看到一帧就立刻按
 * 距离/偏角输出速度，观感就是「人就在旁边，车还在动」。所以先停住确认，
 * 连续 HOLD_MS 都看到人才真正退出搜索；期间又丢了就回原搜索子状态。
 */
typedef enum {
    CHASSIS_SEARCH_IDLE = 0,
    CHASSIS_SEARCH_SPIN = 1,
    CHASSIS_SEARCH_PAUSE = 2,
    CHASSIS_SEARCH_SLOW = 3,
    CHASSIS_SEARCH_STOP = 4
} ChassisSearchState_t;

static ChassisSearchState_t s_search_state = CHASSIS_SEARCH_IDLE;
static ChassisSearchState_t s_search_prev_state = CHASSIS_SEARCH_IDLE; /* 进 STOP 前的搜索子状态 */
static int8_t s_search_dir = 1;              /* 搜索转向方向：+1 / -1，初值 +1 */
static uint16_t s_search_rounds = 0U;        /* 已完成的转圈段数 */
static uint32_t s_search_phase_ms = 0U;      /* 当前子状态已持续的时间 */
static uint32_t s_hold_ms = 0U;              /* STOP 下连续看到人的时间 */
static uint32_t s_lost_ms = 0U;              /* 在线但连续看不到人的时间 */
static int16_t s_last_dev_tenth = 0;         /* 最近一次「有人」时的 yaw 偏角 */
static uint8_t s_safety_stopped = 0U;        /* 近距急停迟滞状态 */

/* ------------------------------------------------- P0 观测：当前执行分支
 *
 * 只记录、不参与任何判断（不改变任何分支行为），供串口 st= 字段使用：
 *   0 = 未进跟随（模式 1 / 默认分支）
 *   1 = 近距急停（距离 < 25 cm）
 *   2 = 断链停车（IsReceiving()==0）
 *   3 = 在线丢人后的确认期（还没开始转圈）
 *   4 = 搜索·转圈（SPIN）
 *   5 = 搜索·暂停（PAUSE）
 *   6 = 搜索·缓慢（SLOW）
 *   7 = 搜索·看到人后的停车确认（STOP）
 *   8 = 对准（有人，但距离无效）
 *   9 = 正常跟随（有人且有有效距离）
 *
 * sw = 最近一个完整 1 s 窗口内的分支切换次数（判断状态抖动有多剧烈） */
static uint8_t s_branch_code = 0U;            /* 本拍执行的分支码 */
static uint8_t s_branch_prev = 0U;            /* 上一拍的分支码，用于统计切换 */
static uint16_t s_branch_switch_count = 0U;   /* 当前窗口内累计切换次数 */
static uint16_t s_branch_sw_per_sec = 0U;     /* 上一个完整窗口的切换次数 */
static uint32_t s_branch_win_ms = 0U;         /* 当前统计窗口起点 */
/* dage 直接取 DistanceSensor_GetAgeMs()（距离模块内部记录的「距上次被接受的采样」）。
 * 2026-11-14 起不要在这里按 valid 记时刻：加了「保持」之后，丢帧期间 valid 仍为 1，
 * 按 valid 记时间会让 dage 永远是 0，正好把要观测的丢帧藏起来。 */

/* 记录本拍走的分支；分支变化时累加切换计数（纯统计，无副作用）。 */
static void ChassisTask_SetBranch(uint8_t code)
{
    s_branch_code = code;
    if (code != s_branch_prev) {
        s_branch_prev = code;
        if (s_branch_switch_count < 0xFFFFU) {
            s_branch_switch_count++;
        }
    }
}

/* --------------------------------------- 偏角野值门（跳变抑制，2026-11-16）
 *
 * dev 来自云台 yaw 角，每拍的变化不该超过「车能转多快」：
 * 满舵差速 120‰ 约 35~40 度/秒，30 ms 一帧最多 1~2 度。
 * 实测日志出现过单帧跳 70~105 度（K230 目标框跳到别人/别的东西上），
 * 底盘照这个值打方向，就是「走着走着突然偏一下」。
 *
 * 做法：单帧变化超过门限 -> 弃用，返回上一次接受的值；
 *       连续被弃用 REJECT 次 -> 认定目标真的换了，接受新值。
 * 正常跟踪时原样直通，零延迟。 */
static int32_t s_dev_used = 0;               /* 上一次被接受的偏角（0.1 度） */
static uint8_t s_dev_init = 0U;              /* 0 = 还没有参考值 */
static uint8_t s_dev_reject = 0U;            /* 连续被弃用的帧数 */

/* 转向量一阶低通（2026-11-17 加，治「一冲一冲」）
 * y[n] = y[n-1] + (x[n] - y[n-1]) × LPF_Q / 100
 * 阶跃变斜坡：α=0.35、5 ms 一拍 → τ≈14 ms、fc≈11 Hz。
 * 只压快变成分，稳态值不变；比 250~300 ms 的链路死时间小一个数量级。 */
static int32_t s_steer_filt = 0;
static int32_t s_steer_damp_dev = 0;     /* 上一拍的偏角，用来算变化率（阻尼项） */
static uint32_t s_steer_damp_ms = 0U;    /* 上一拍的时刻，0 = 还没有历史 */

/* 「距离无效但看到人」时沿用上一次有效的前进量（2026-11-17 加）
 * 原来固定 BASE_SPEED，从 350‰ 直接掉到 180‰，用户反馈「明显迟钝」。
 * 现在从上次的速度平滑接续；连续无效超过 BLIND_FORWARD_MS 才降回基准速度。 */
static int32_t s_forward_last = 0;           /* 上一次非零的前进量（‰，负 = 前进） */
static uint32_t s_dist_invalid_ms = 0U;      /* 距离连续无效的时长（ms） */

static int32_t ChassisTask_FilterDev(int32_t raw)
{
    int32_t delta;

    if (s_dev_init == 0U) {
        s_dev_used = raw;
        s_dev_init = 1U;
        s_dev_reject = 0U;
        return raw;
    }

    delta = raw - s_dev_used;
    if (delta < 0) {
        delta = -delta;
    }

    if (delta <= (int32_t)CHASSIS_FOLLOW_DEV_JUMP_TENTH_DEG) {
        s_dev_used = raw;
        s_dev_reject = 0U;
        return raw;
    }

    s_dev_reject++;
    if (s_dev_reject >= (uint8_t)CHASSIS_FOLLOW_DEV_JUMP_REJECT) {
        /* 连续多帧都指向同一个新位置：认定真的换了目标 */
        s_dev_used = raw;
        s_dev_reject = 0U;
        return raw;
    }

    return s_dev_used;
}

/* 清搜索状态。保留 s_search_dir：它记录「上次搜索的方向」，
 * 下次丢人且没有有效偏角时优先沿用，避免每次都从同一侧开始。 */
static void ChassisTask_ResetSearch(void)
{
    s_search_state = CHASSIS_SEARCH_IDLE;
    s_search_prev_state = CHASSIS_SEARCH_IDLE;
    s_search_rounds = 0U;
    s_search_phase_ms = 0U;
    s_hold_ms = 0U;
    s_lost_ms = 0U;
}

/* 用户切模式：搜索相关状态全部回到初值。 */
static void ChassisTask_ResetFollowState(void)
{
    ChassisTask_ResetSearch();
    s_search_dir = 1;
    s_search_phase_ms = 0U;
    s_last_dev_tenth = 0;
    s_safety_stopped = 0U;
    s_dev_init = 0U;
    s_dev_reject = 0U;
    s_steer_filt = 0;
    s_steer_damp_dev = 0;
    s_steer_damp_ms = 0U;
    s_forward_last = 0;
    s_dist_invalid_ms = 0U;
}

/* 进入搜索：优先朝人最后出现的方向转；偏角在死区内则沿用上次的方向。 */
static void ChassisTask_EnterSearch(void)
{
    /* 提示音：丢人进搜索，三声急促（与模式1 的丢线同一个音型） */
    VisionLink_SendBeep(LINK_BEEP_LOST);

    if (s_last_dev_tenth > (int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG) {
        s_search_dir = 1;
    } else if (s_last_dev_tenth < -(int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG) {
        s_search_dir = -1;
    } else {
        /* 偏角太小或没有历史：保留上一次的 s_search_dir（初值 +1） */
    }

    /* 转向低通状态清零（审计 ①）：搜索/暂停期间 steer 由状态机直接给出、
     * 不更新 s_steer_filt，所以它会一直停在「进搜索前」的旧值上。
     * 不清零的话，重新看到人后的第一拍低通会从那个旧值开始衰减，
     * 表现为车头不自然地摆一下。 */
    s_steer_filt = 0;
    s_steer_damp_dev = 0;
    s_steer_damp_ms = 0U;

    s_search_rounds = 0U;
    s_search_phase_ms = 0U;
    s_search_prev_state = CHASSIS_SEARCH_IDLE;
    s_hold_ms = 0U;
    s_search_state = CHASSIS_SEARCH_SPIN;
}

/* 推进搜索状态机，返回本周期的转向量（forward 恒为 0）。
 *
 * SEARCHING --MAX_MS--> PAUSING --PAUSE_MS--> rounds++；未满 ROUNDS 则反向回 SEARCHING
 * rounds >= ROUNDS -> SLOW：保持最后方向、以 SLOW_DUTY 持续慢转，不再计时/换向
 *
 * STOP 不在这里推进：它由 ChassisTask_UpdateFollow() 在「看到人」时切进来，
 * 本函数只负责 SPIN / PAUSE / SLOW。
 *
 * 注意：近距急停时调用方会提前 return，本函数不被调用，
 *       所以计时自然冻结，障碍解除后不会「瞬间跳状态」。 */
static int32_t ChassisTask_UpdateSearchState(void)
{
    /* 缓慢搜索是终态：方向锁存、匀速慢转，不再累计计时/轮数，也不换向。
     * 只有 detected=1（调用方先切 STOP）、用户切模式或断链能退出（均由调用方复位）。 */
    if (s_search_state == CHASSIS_SEARCH_SLOW) {
        return (int32_t)s_search_dir * (int32_t)CHASSIS_SEARCH_SLOW_DUTY;
    }

    s_search_phase_ms += (uint32_t)CHASSIS_CONTROL_PERIOD_MS;

    if (s_search_state == CHASSIS_SEARCH_SPIN) {
        if (s_search_phase_ms >= (uint32_t)CHASSIS_SEARCH_MAX_MS) {
            s_search_state = CHASSIS_SEARCH_PAUSE;
            s_search_phase_ms = 0U;
        }
    } else if (s_search_state == CHASSIS_SEARCH_PAUSE) {
        if (s_search_phase_ms >= (uint32_t)CHASSIS_SEARCH_PAUSE_MS) {
            s_search_phase_ms = 0U;
            s_search_rounds++;

            if (s_search_rounds >= (uint16_t)CHASSIS_SEARCH_ROUNDS) {
                /* 轮数用尽：不换向，保持最后一段转圈的方向转入缓慢搜索。 */
                s_search_state = CHASSIS_SEARCH_SLOW;
            } else {
                s_search_dir = (int8_t)((s_search_dir > 0) ? -1 : 1);
                s_search_state = CHASSIS_SEARCH_SPIN;
            }
        }
    } else {
        /* IDLE：不动 */
    }

    if (s_search_state == CHASSIS_SEARCH_SPIN) {
        /* 转圈找人时按「有没有距离」分两档速度：
         *   距离有效 -> CHASSIS_SEARCH_SPIN_DUTY（160‰，正常速度扫）
         *   距离无效 -> CHASSIS_SEARCH_SLOW_DUTY（80‰，慢速找）
         * 距离无效说明超声波也没回波，这时看不清环境，转快了容易撞、
         * 也容易把人甩出画面，所以先降速（2026-11-16 用户要求）。 */
        int32_t spin_duty = (DistanceSensor_IsValid() != 0U)
                                ? (int32_t)CHASSIS_SEARCH_SPIN_DUTY
                                : (int32_t)CHASSIS_SEARCH_SLOW_DUTY;
        return (int32_t)s_search_dir * spin_duty;
    }
    return 0;
}

/* --------------------------------------------------------- 内部：模式逻辑 -- */

/* 模式 1：循迹 + 云台跟踪（本板只做循迹）
 *
 * ⚠️ 按任务要求「循迹代码先空着」：判线算法与转向律都还没实现，
 *    这里保持停车，不做任何驱动。判线实现在 modules/line_sensor.c。 */
static void ChassisTask_UpdateLineTrack(void)
{
    /* 脱线保持用的两个静态量：上一拍的差速量、已经保持了多久 */
    static int32_t s_line_last_steer = 0;
    static uint32_t s_line_lost_ms = 0U;
    static int32_t s_line_last_pos = 0;

    LineSensor_t line = LineSensor_Get();
    int32_t steer;
    int32_t forward;
    int32_t left;
    int32_t right;
    int32_t mag;
    int32_t factor;

    /* ---------- 提示音 1：脱线（三声急促） ----------
     * 只在"刚脱线"那一下响，压回线再脱线才会再响。 */
    if (line.lost != 0U) {
        if (s_beep_line_lost == 0U) {
            s_beep_line_lost = 1U;
            VisionLink_SendBeep(LINK_BEEP_LOST);
        }
    } else {
        s_beep_line_lost = 0U;
    }

    /* ---------- 提示音 2 + 十字路口停车 ----------
     * 7 路全黑（active_mask 位=1 表示压黑线，故 0x7F）= 十字路口/停车线。
     * 2026-11-21 需求：遇到十字路口不再直行通过 —— 全黑持续
     * CHASSIS_LINE_CROSS_MS 就立刻停车 + 长鸣 1 s，然后锁存不动，
     * 直到按 PB13 或蓝牙发 mode1/mode2 重新起跑。
     * CHASSIS_LINE_CROSS_MS = 0 时只长鸣不停车（可在线用 crs 调）。 */
    if ((line.active_mask & 0x7FU) == 0x7FU) {
        if (s_line_black_ms < 1000U) {
            s_line_black_ms += CHASSIS_CONTROL_PERIOD_MS;
        }
        if ((s_line_armed != 0U) && (s_line_black_ms >= CHASSIS_LINE_CROSS_MS) &&
            (s_line_black_beeped == 0U)) {
            s_line_black_beeped = 1U;
            VisionLink_SendBeep(LINK_BEEP_STOP_LINE);
            if (CHASSIS_LINE_CROSS_MS != 0U) {
                s_line_arrived = 1U;
                s_line_armed = 0U;      /* 解锁后必须先离开黑区，避免原地反复触发 */
            }
        }
    } else {
        s_line_black_ms = 0U;
        s_line_black_beeped = 0U;
        s_line_armed = 1U;              /* 见到白色 = 已离开路口，重新允许判定 */
    }

    /* 已判到十字路口：停车并锁存，后面的循迹控制全部跳过 */
    if (s_line_arrived != 0U) {
        ChassisTask_SetBranch(13U);
        Motor_SetSpeed(0, 0);
        s_line_last_steer = 0;
        s_line_last_pos = 0;
        s_line_lost_ms = 0U;
        return;
    }

    /* 1) 脱线：先按上一拍差速再走 CHASSIS_LINE_LOST_HOLD_MS，超时才停。
     *    这样偏出去的车有机会自己压回线，而不是一走神就趴窝。
     *    （line.lost 本身已带 20 拍 = 100 ms 的确认，偶发一两拍不会触发。） */
    if (line.lost != 0U) {
        if (s_line_lost_ms < CHASSIS_LINE_LOST_HOLD_MS) {
            s_line_lost_ms += CHASSIS_CONTROL_PERIOD_MS;

            left = -(int32_t)CHASSIS_LINE_BASE_SPEED - s_line_last_steer;
            right = -(int32_t)CHASSIS_LINE_BASE_SPEED + s_line_last_steer;

            if (left > (int32_t)CHASSIS_MOTOR_DUTY_MAX) {
                left = (int32_t)CHASSIS_MOTOR_DUTY_MAX;
            } else if (left < -(int32_t)CHASSIS_MOTOR_DUTY_MAX) {
                left = -(int32_t)CHASSIS_MOTOR_DUTY_MAX;
            }
            if (right > (int32_t)CHASSIS_MOTOR_DUTY_MAX) {
                right = (int32_t)CHASSIS_MOTOR_DUTY_MAX;
            } else if (right < -(int32_t)CHASSIS_MOTOR_DUTY_MAX) {
                right = -(int32_t)CHASSIS_MOTOR_DUTY_MAX;
            }

            ChassisTask_SetBranch(11U);
            Motor_SetSpeed((int16_t)left, (int16_t)right);
            return;
        }

        ChassisTask_SetBranch(11U);
        Motor_SetSpeed(0, 0);

        s_line_last_steer = 0;
        s_line_last_pos = 0;
        s_line_lost_ms = 0U;
        return;
    }

    s_line_lost_ms = 0U;

    /* 2) 线位置 -> 差速。position：-100（最左）~ +100（最右），0 = 线在正中。
     *    「线在右边」= 车偏左了 = 车得往右拐。 */
    steer = ((int32_t)line.position * CHASSIS_LINE_STEER_GAIN) / 100;

    /* 2.1) D 项：position 的变化量。默认 KD=0，等于没有这一行；
     *      实车看 S 弯摆得厉害再给数（20~80 起步）。 */
    if (CHASSIS_LINE_STEER_KD != 0) {
        steer += (((int32_t)line.position - s_line_last_pos) * CHASSIS_LINE_STEER_KD) / 10;
    }
    s_line_last_pos = (int32_t)line.position;

    steer *= CHASSIS_LINE_STEER_SIGN;

    if (steer > CHASSIS_LINE_STEER_MAX) {
        steer = CHASSIS_LINE_STEER_MAX;
    } else if (steer < -CHASSIS_LINE_STEER_MAX) {
        steer = -CHASSIS_LINE_STEER_MAX;
    }

    /* 3) 弯道降速：偏得越多速度越低，避免急弯冲出去。
     *    factor 最低留 20%，防止整数算成 0 直接停车。 */
    mag = (int32_t)line.position;
    if (mag < 0) {
        mag = -mag;
    }

    factor = 100 - ((int32_t)CHASSIS_LINE_SLOW_GAIN * mag) / 100;
    if (factor < 20) {
        factor = 20;
    }

    /* 前进量取负：本车 Motor_SetSpeed() 负值 = 前进（与模式 2 同一套极性） */
    forward = -(((int32_t)CHASSIS_LINE_BASE_SPEED * factor) / 100);

    /* 4) 差速合成：与模式 2 同一套约定 —— 左减右加，
     *    steer 为正时左轮更偏前进，车头向右转。 */
    left = forward - steer;
    right = forward + steer;

    if (left > (int32_t)CHASSIS_MOTOR_DUTY_MAX) {
        left = (int32_t)CHASSIS_MOTOR_DUTY_MAX;
    } else if (left < -(int32_t)CHASSIS_MOTOR_DUTY_MAX) {
        left = -(int32_t)CHASSIS_MOTOR_DUTY_MAX;
    }
    if (right > (int32_t)CHASSIS_MOTOR_DUTY_MAX) {
        right = (int32_t)CHASSIS_MOTOR_DUTY_MAX;
    } else if (right < -(int32_t)CHASSIS_MOTOR_DUTY_MAX) {
        right = -(int32_t)CHASSIS_MOTOR_DUTY_MAX;
    }

    s_line_last_steer = steer;

    ChassisTask_SetBranch(12U);
    Motor_SetSpeed((int16_t)left, (int16_t)right);
}

/* 模式 2：纯人体跟随
 *
 * 两路互相独立的输入合成差速：
 *
 *   转向  <- 云台 yaw 偏角      「云台往哪边转，车就往哪边转」
 *   前后  <- 超声波距离闭环      「保持设定车距：太远前进，太近后退」
 *
 *   left  = 前后 - 转向
 *   right = 前后 + 转向
 *
 * 为什么转向是「减」：本车 Motor_SetSpeed() 正值 = 后退、负值 = 前进
 * （实机验证过的极性，距离环里「目标 - 实测」的符号修正就是为配合它）。
 * 「车头右转」要求左轮更偏前进（更负）、右轮更偏后退（更正），
 * 所以 steer 为正（云台右偏）时必须是 left = 前后 - steer、right = 前后 + steer。
 * 早期写成「左加右减」，结果 Dev 为正时车往左转，实车转向反向，故反转。
 *
 * 转向用偏角而不是画面偏差，是因为云台一直把人保持在画面中心，
 * 画面偏差恒接近 0（详见 ../docs/双板分工与协议.md）。
 */
static void ChassisTask_UpdateFollow(void)
{
    VisionLink_t link;
    int16_t dev_tenth;
    int32_t dev_f;                           /* 过野值门后的偏角（0.1 度） */
    int32_t steer = 0;
    int32_t forward_speed = 0;
    int32_t left;
    int32_t right;
    const int32_t speed_max = (int32_t)CHASSIS_FOLLOW_SPEED_MAX;
    const int32_t duty_max = (int32_t)CHASSIS_MOTOR_DUTY_MAX;

    /* ---------- 0. 近距急停（最高优先级，迟滞） ----------
     * 有效距离 < STOP 时整车停（前进/转向全清零）；回到 >= RESUME 才解除。
     * 搜索态同样适用：离得太近就停车不转。
     * 本保护独立于距离是否有效：只要读到过小的有效值就停车，
     * 对「距离无效、慢速直行」那个分支同样生效。 */
    if (DistanceSensor_IsValid() != 0U) {
        uint16_t distance_cm = DistanceSensor_GetCm();

        if (distance_cm < (uint16_t)CHASSIS_SAFETY_STOP_CM) {
            s_safety_stopped = 1U;
        } else if (distance_cm >= (uint16_t)CHASSIS_SAFETY_RESUME_CM) {
            s_safety_stopped = 0U;
        } else {
            /* 迟滞区 [STOP, RESUME)：保持原判，防止在阈值附近反复停/走 */
        }
    }
    if (s_safety_stopped != 0U) {
        ChassisTask_SetBranch(1U);
        Motor_SetSpeed(0, 0);
        return;
    }

    /* ---------- 1. 断链：停车，搜索状态清零 ---------- */
    /* IsReceiving() 只看帧是否超时；IsFresh() 额外要求 detected=1。
     * 两者必须分开，否则「线松了」会被误当成「人丢了」而原地转圈。 */
    if (VisionLink_IsReceiving() == 0U) {
        ChassisTask_ResetSearch();
        ChassisTask_SetBranch(2U);
        Motor_SetSpeed(0, 0);
        return;
    }

    link = VisionLink_Get();

    /* ---------- 2. 搜索中重新看到人：先整车停车确认 ----------
     * 「看到一帧」不等于「可以跟」：转圈时画面在抖，检测时有时无。
     * 只要 fresh=1 就立刻切 STOP，forward/steer 都置 0（整车静止）；
     * 连续 HOLD_MS 都看到人才退出搜索。中途又丢人 -> 回原搜索子状态，
     * s_search_phase_ms / s_search_rounds 原样保留，不重置，避免来回抖。 */
    if (s_search_state != CHASSIS_SEARCH_IDLE) {
        if (link.fresh != 0U) {
            if (s_search_state != CHASSIS_SEARCH_STOP) {
                s_search_prev_state = s_search_state;
                s_search_state = CHASSIS_SEARCH_STOP;
                s_hold_ms = 0U;
            }

            s_last_dev_tenth = link.yaw_dev_tenth_deg;
            s_hold_ms += (uint32_t)CHASSIS_CONTROL_PERIOD_MS;

            if (s_hold_ms >= (uint32_t)CHASSIS_SEARCH_HOLD_MS) {
                /* 确认完成：退出搜索；本拍仍静止，下一拍起走正常跟随 */
                ChassisTask_ResetSearch();
            }

            ChassisTask_SetBranch(7U);
            Motor_SetSpeed(0, 0);
            return;
        }

        if (s_search_state == CHASSIS_SEARCH_STOP) {
            /* 确认期又丢人：回原搜索子状态，继续用原来的计时/轮数 */
            s_search_state = s_search_prev_state;
        }
        s_hold_ms = 0U;
    }

    /* ---------- 3. 有人：正常跟随 ---------- */
    if (link.fresh != 0U) {
        /* 搜索态已在上面的 STOP 分支处理；这里通常本来就是非搜索态。
         * 只在真的还挂着搜索状态时才清，避免跟随中每拍白跑一次（审计 B2）。 */
        if (s_search_state != CHASSIS_SEARCH_IDLE) {
            ChassisTask_ResetSearch();
        }
        dev_f = ChassisTask_FilterDev((int32_t)link.yaw_dev_tenth_deg);
        s_last_dev_tenth = (int16_t)dev_f;

        /* ---------- 3.1 转向：云台偏角 ---------- */
        dev_tenth = (int16_t)dev_f;

        /* 死区：云台微抖不该让车摆头 */
        if (dev_tenth > (int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG) {
            dev_tenth = (int16_t)(dev_tenth - (int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG);
        } else if (dev_tenth < -(int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG) {
            dev_tenth = (int16_t)(dev_tenth + (int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG);
        } else {
            dev_tenth = 0;
        }

        /* 0.1 度 × (千分比/度) / 10 = 千分比 */
        steer = ((int32_t)dev_tenth * (int32_t)CHASSIS_FOLLOW_STEER_GAIN_Q) / 10;
        steer = steer * (int32_t)CHASSIS_FOLLOW_STEER_SIGN;

        /* 阻尼项（D）：按「偏角变化率」反向施加一个量。
         * 纯 P 时，车转过去的过程中 dev 一直在变小，但 P 只看当前 dev，
         * 于是减速太晚 —— 实测 dev 从 +29.0° 翻到 -41.7° 还在冲。
         * 这里用 (dev - 上一拍dev)/dt 得到底盘相对目标的角速度，
         * 乘 KD 后叠加：正在快速靠近目标时它把转向量往回拉，等于提前刹车。
         * 单位：KD 是 ‰/(度/秒)，rate 是 度/秒。 */
        {
            const uint32_t now_damp = Delay_GetMs();
            uint32_t dt_ms = now_damp - s_steer_damp_ms;

            if (dt_ms == 0U) {
                dt_ms = 1U;
            }
            if (dt_ms > 100U) {
                dt_ms = 100U;
            }

            if (s_steer_damp_ms != 0U) {
                int32_t rate_dps = ((dev_f - s_steer_damp_dev) * 1000)
                                 / ((int32_t)dt_ms * 10);
                steer += (int32_t)CHASSIS_FOLLOW_STEER_KD
                       * rate_dps
                       * (int32_t)CHASSIS_FOLLOW_STEER_SIGN;
            }

            s_steer_damp_dev = dev_f;
            s_steer_damp_ms = (now_damp == 0U) ? 1U : now_damp;
        }

        /* 差速单独封顶：限制转向的快慢（不同于 duty_max 的总量限幅）。
         * 为什么必须单独限：转向靠左右轮反向形成，差速越大转越快；
         * 而云台帧有 250~300 ms 死时间，转太快会在「发现已经对准」之前
         * 就把人甩出摄像头视场（实测现象：直接丢人、进搜索）。
         * 45 度偏角 -> 120‰，转速约 35~40 度/秒，死时间内只转 10~12 度。 */
        if (steer > (int32_t)CHASSIS_FOLLOW_STEER_MAX) {
            steer = (int32_t)CHASSIS_FOLLOW_STEER_MAX;
        }
        if (steer < -(int32_t)CHASSIS_FOLLOW_STEER_MAX) {
            steer = -(int32_t)CHASSIS_FOLLOW_STEER_MAX;
        }

        /* 盲转限速：|偏角| >= 90 度说明云台已经顶到行程限位（yaw 3 或 267 度），
         * 摄像头看不到更偏的方向，转向环等于开环 —— 转多快都是瞎猜。
         * 实测：150‰ 盲转会在「人重新进画面」之前冲过头 153 度，然后反方向再冲，
         * 表现出来就是左右画龙。所以盲转区改用固定小差速：慢转 + 频繁回看。 */
        if (dev_f >= (int32_t)CHASSIS_FOLLOW_BLIND_DEV_TENTH_DEG
            || dev_f <= -(int32_t)CHASSIS_FOLLOW_BLIND_DEV_TENTH_DEG) {
            steer = (steer < 0) ? -(int32_t)CHASSIS_FOLLOW_BLIND_STEER
                                : (int32_t)CHASSIS_FOLLOW_BLIND_STEER;
        }

        /* 转向量一阶低通（见文件头 s_steer_filt 说明）。
         * 死区边界上转向量是阶跃的（0 直接跳到 32‰），直接进差速会造成
         * 「一冲一冲」：力矩突变、车身侧倾、云台视角跟着晃、下一拍偏角又变。
         * 低通把它变成 ~15 ms 的斜坡，代价只是 14 ms 滞后。
         * 整数运算要为小差值保底一步，否则 (diff×35)/100 会被截断成 0，
         * 转向量永远爬不上去。 */
        {
            int32_t diff = steer - s_steer_filt;
            int32_t step = (diff * (int32_t)CHASSIS_FOLLOW_STEER_LPF_Q) / 100;

            if ((step == 0) && (diff != 0)) {
                step = (diff > 0) ? 1 : -1;
            }
            s_steer_filt += step;
            steer = s_steer_filt;
        }

        if (steer > duty_max) {
            steer = duty_max;
        }
        if (steer < -duty_max) {
            steer = -duty_max;
        }

        /* ---------- 3.2 前后：超声波距离闭环 ---------- */
        if (DistanceSensor_IsValid() != 0U) {
            /* 实机验证：原符号导致近了前进、远了后退，故改为「目标 - 实测」 */
            int32_t error_cm = (int32_t)CHASSIS_FOLLOW_TARGET_DISTANCE_CM
                             - (int32_t)DistanceSensor_GetCm();
            const int32_t deadband_cm = (int32_t)CHASSIS_FOLLOW_DISTANCE_DEADBAND_CM;

            /* 距离死区：差在范围内就不前后动，避免原地来回蹭 */
            if (error_cm > deadband_cm) {
                error_cm -= deadband_cm;
            } else if (error_cm < -deadband_cm) {
                error_cm += deadband_cm;
            } else {
                error_cm = 0;
            }

            /* 千分比/cm × cm = 千分比；误差为正 = 太近 = 后退，为负 = 太远 = 前进 */
            forward_speed = error_cm * (int32_t)CHASSIS_FOLLOW_DISTANCE_GAIN_Q;

            if (forward_speed > speed_max) {
                forward_speed = speed_max;
            }
            if (forward_speed < -speed_max) {
                forward_speed = -speed_max;
            }

            /* 后退保护 1：后退单独限速（前进不受影响，仍是 SPEED_MAX）
             * 实测：人站 2 m 外时超声打不准、常被误判成「很近」，
             * 距离环就会给满速后退 —— 车头背离人，最容易撞墙。 */
            if (forward_speed > (int32_t)CHASSIS_FOLLOW_REVERSE_MAX) {
                forward_speed = (int32_t)CHASSIS_FOLLOW_REVERSE_MAX;
            }

            /* 后退保护 2：距离已经旧了就禁止后退
             * 只在「距上次被接受的采样超过 STALE_MS」时禁退，
             * 正常偶发丢一两帧不影响，也不会拿 1.5 秒前的旧近距离值往后倒。 */
            if (forward_speed > 0
                && DistanceSensor_GetAgeMs() > (uint16_t)CHASSIS_FOLLOW_REVERSE_STALE_MS) {
                forward_speed = 0;
            }

            /* 记下「正在跑的非零前进量」，供距离失效时平滑接续（见文件头说明）。
             * 只记非零值：停在死区内（误差 < 10 cm）时不该把记忆刷成 0，
             * 否则一丢距离就变成停车，又回到「走两步停一下」。 */
            if (forward_speed != 0) {
                s_forward_last = forward_speed;
            }
            s_dist_invalid_ms = 0U;

            ChassisTask_SetBranch(9U);
        } else {
            /* 测不到有效距离，但视觉已确认前方有人：继续往前走，
             * 转向仍用上面 3.1 段算好的偏角律。
             *
             * 为什么不再「原地对准、一步不走」：
             *   实测超声经常连续 4~6 秒一个有效回波都没有（人站 1.5 m 也收不到，
             *   us 常是 47499 us = 819 cm 这种超量程值），
             *   于是车永远停在「只转不前进」状态，看起来就是「连直线都走不了」。
             *
             * 速度取哪个（2026-11-17 改）：
             *   原来是写死的 BASE_SPEED，而距离有效时最大能到 SPEED_MAX(350‰)，
             *   一丢距离就从 350 掉到 180，用户反馈「明显迟钝」。
             *   现在改成「沿用上一次正在跑的非零前进量」—— 从 350 平滑接续；
             *   连续无效超过 BLIND_FORWARD_MS 再降回 BASE_SPEED，避免满速盲跑。
             *
             * 只允许前进（负值）：记忆里若是「后退」（正值），盲跑后退最危险，
             * 一律当 0 处理。
             *
             * 符号：Motor_SetSpeed() 负值 = 前进。 */
            s_dist_invalid_ms += (uint32_t)CHASSIS_CONTROL_PERIOD_MS;
            if ((s_forward_last != 0)
                && (s_dist_invalid_ms <= (uint32_t)CHASSIS_FOLLOW_BLIND_FORWARD_MS)) {
                forward_speed = s_forward_last;
            } else {
                forward_speed = -(int32_t)CHASSIS_FOLLOW_BASE_SPEED;
            }
            if (forward_speed > 0) {
                forward_speed = 0;
            }
            ChassisTask_SetBranch(10U);
        }
    } else {
        /* ---------- 4. 在线但丢人：确认后原地转圈搜索 ---------- */
        s_lost_ms += (uint32_t)CHASSIS_CONTROL_PERIOD_MS;

        /* 确认期内先停车：单帧/几帧漏检不该让车突然转圈 */
        if (s_lost_ms < (uint32_t)CHASSIS_SEARCH_CONFIRM_MS) {
            ChassisTask_SetBranch(3U);
            Motor_SetSpeed(0, 0);
            return;
        }

        if (s_search_state == CHASSIS_SEARCH_IDLE) {
            ChassisTask_EnterSearch();
        }

        /* 搜索时前后量恒为 0，转向量由状态机给出；
         * 差速合成仍走下面同一段，forward=0 时就是原地转。
         * 若刚从 STOP 退回，状态机沿用原来的阶段计时/轮数继续转。 */
        steer = ChassisTask_UpdateSearchState();
        forward_speed = 0;

        /* P0 观测：搜索子状态映射成分支码（4=转圈 5=暂停 6=缓慢） */
        if (s_search_state == CHASSIS_SEARCH_SPIN) {
            ChassisTask_SetBranch(4U);
        } else if (s_search_state == CHASSIS_SEARCH_PAUSE) {
            ChassisTask_SetBranch(5U);
        } else if (s_search_state == CHASSIS_SEARCH_SLOW) {
            ChassisTask_SetBranch(6U);
        } else {
            ChassisTask_SetBranch(3U);
        }
    }

    /* ---------- 4.5 大偏角：只转不走 ---------- */
    /* 人跑到侧后方（偏角 >= 45 度）时，「前进 + 转向」会走成大弧线：
     * 前进满速 350‰ 而差速最多 120‰，转 180 度要走 2~3 米，
     * 人始终待在侧后方，车越开越远，最后丢人
     * （实测 dev=157.9 度时 out=-516/-213：两轮都在前进，只在慢慢拐）。
     * 所以这里把前进量清 0，原地转过来再说。 */
    if (dev_f >= (int32_t)CHASSIS_FOLLOW_TURN_ONLY_TENTH_DEG
        || dev_f <= -(int32_t)CHASSIS_FOLLOW_TURN_ONLY_TENTH_DEG) {
        forward_speed = 0;
    }

    /* ---------- 5. 合成差速 ---------- */
    /* 符号说明（实机验证过的极性）：Motor_SetSpeed() 正值 = 后退、负值 = 前进。
     * 「车头右转」= 左轮更偏前进（更负）+ 右轮更偏后退（更正），
     * 所以 steer 为正（云台右偏）时必须「左减右加」。
     * 注意：这里不是笔误，别改回「左加右减」——那会让车朝反方向转。 */
    left = forward_speed - steer;
    right = forward_speed + steer;

    /* 允许为负：太近时要能倒车。但仍要夹在硬件限幅内。 */
    if (left > duty_max) {
        left = duty_max;
    }
    if (left < -duty_max) {
        left = -duty_max;
    }
    if (right > duty_max) {
        right = duty_max;
    }
    if (right < -duty_max) {
        right = -duty_max;
    }

    Motor_SetSpeed((int16_t)left, (int16_t)right);
}

/* ------------------------------------------------------------ 内部：按键 -- */

static void ChassisTask_PollKey(uint32_t now_ms)
{
    uint8_t down = BSP_Key_IsDown(BSP_KEY_MODE);

    if (down != s_key_last_raw) {
        s_key_last_raw = down;
        s_key_last_change_ms = now_ms;
        return;
    }

    if ((uint32_t)(now_ms - s_key_last_change_ms) < CHASSIS_KEY_DEBOUNCE_MS) {
        return;
    }

    if (down != 0U) {
        if (s_key_stable_down == 0U) {
            s_key_stable_down = 1U;

            /* 按键优先：按下就把蓝牙 stop 的锁存解掉，人手上说了算 */
            s_remote_stop = 0U;

            /* 同时解掉"十字路口已停车"的锁存，按一下就能重新起跑 */
            s_line_arrived = 0U;
            s_line_armed = 0U;
            s_line_black_ms = 0U;
            s_line_black_beeped = 0U;

            if (s_mode == CHASSIS_MODE_LINE_TRACK) {
                s_mode = CHASSIS_MODE_FOLLOW;
            } else {
                s_mode = CHASSIS_MODE_LINE_TRACK;
            }

            /* 用户切模式 = 搜索状态复位（含 SLOW），回到初始方向 */
            ChassisTask_ResetFollowState();

            /* 切模式瞬间先停车，避免带着上一个模式的输出冲出去 */
            Motor_Stop();
        }
    } else {
        s_key_stable_down = 0U;
    }
}

/* -------------------------------------------------- 内部：蓝牙命令处理 ----
 * 手机发 mode1/mode2/stop -> 蓝牙 -> 云台板解析 -> 链路帧第 11 字节 -> 这里。
 *
 * 为什么要边沿检测：云台板为了让命令更抗干扰，会把同一个命令码连发 5 帧
 * （150 ms）。这里只在「命令码发生变化」时动作一次，之后一直是同一个值
 * 就不再重复执行；命令回到 0 后，下一次再发同样的命令又能触发。
 *
 * 动作前一律 Motor_Stop()：避免带着上一个模式的输出冲出去。
 * ------------------------------------------------------------------------- */
static void ChassisTask_PollRemoteCommand(void)
{
    uint8_t command = VisionLink_GetCommand();

    if (command == s_link_cmd) {
        return;                     /* 没变化：不重复执行 */
    }
    s_link_cmd = command;

    if (command == 0U) {
        return;                     /* 命令撤掉了，只更新记录 */
    }

    Motor_Stop();
    ChassisTask_ResetFollowState();

    /* 任何蓝牙命令都解掉"十字路口已停车"的锁存，重新起跑 */
    s_line_arrived = 0U;
    s_line_armed = 0U;
    s_line_black_ms = 0U;
    s_line_black_beeped = 0U;

    if (command == 1U) {
        s_mode = CHASSIS_MODE_LINE_TRACK;
        s_remote_stop = 0U;
    } else if (command == 2U) {
        s_mode = CHASSIS_MODE_FOLLOW;
        s_remote_stop = 0U;
    } else if (command == 3U) {
        s_remote_stop = 1U;
    } else {
        /* 未知命令码：忽略（未来扩展时不会误动作） */
    }
}

/* ------------------------------------------------------------ 内部：显示 -- */

/* 把 text 补空格到 16 列后整行写出，避免切换模式时残留旧字符 */
/* ---------------------------------------------- OLED 差异刷新（分片写入） ----
 * 这块屏没有显存回读，原来每次刷新都是「整行 16 字符 × 2 页 = 256 个数据字节」
 * 直接写 I2C；4 行合计约 1024 字节、每个字节 3 个 I2C 字节约 20 us，
 * 一次整屏刷新 ≈ 21 ms —— 这 21 ms 里控制环完全停住，而且每 100 ms 发生一次。
 *
 * 现在改成两级削峰：
 *   1. 字符级差异：组好文本后与缓存逐字符比较，只把变化的字符压进队列
 *      （屏幕上 "Mode:"、"Dist:" 这些前缀永远不变，一屏通常只变 2~6 个数字）；
 *   2. 每拍限量：主循环每一拍最多吐出 OLED_DIFF_PER_TICK 个字符，
 *      单拍最坏阻塞从 21 ms 降到约 2 ms，其余拍完全不受影响。
 * ------------------------------------------------------------------------- */
#define OLED_DIFF_PER_TICK    3U
#define OLED_DIFF_QUEUE_MAX   (4U * 16U)

typedef struct {
    uint8_t line;               /* 1 ~ 4 */
    uint8_t column;             /* 1 ~ 16 */
    char    ch;
} ChassisOledCell_t;

static char s_oled_cache[4][16];                   /* 屏上当前显示的内容 */
static ChassisOledCell_t s_oled_queue[OLED_DIFF_QUEUE_MAX];
static uint8_t s_oled_queue_count = 0U;
static uint8_t s_oled_queue_head = 0U;

/* 主循环单拍最长耗时的观测（过去 1 s 内的最大值，单位 ms）。
 * 它的用途是验证「削峰到底有没有生效」：改之前撞上刷屏那一拍会是 20 ms 以上，
 * 改之后应该稳定在个位数。 */
static uint32_t s_loop_max_ms = 0U;

/* 上面那个最大值出现在第几段（定位卡顿用）。
 *   1=链路解析 2=超声波 3=循迹采样 4=按键 5=蓝牙命令 6=显示 7=控制律 8=串口回传 */
static uint8_t s_loop_max_stage = 0U;
static uint8_t s_oled_chars_max = 0U;      /* 过去 1 s 内单拍最多写了几个 OLED 字符 */

/* 记录「从本拍开始到现在」的累计耗时，并把最慢的那一段记下来。
 * 只是读一个变量，单次成本 1 us 量级，可以每段都调用。 */
static void ChassisTask_MarkStage(uint8_t stage, uint32_t now_ms)
{
    uint32_t spent_ms = Delay_GetMs() - now_ms;

    if (spent_ms > s_loop_max_ms) {
        s_loop_max_ms = spent_ms;
        s_loop_max_stage = stage;
    }
}

static void ChassisTask_ShowPaddedLine(uint8_t line, const char *text)
{
    char buffer[17];
    uint8_t i = 0U;
    uint8_t row = (uint8_t)(line - 1U);

    while (text[i] != '\0' && i < 16U) {
        buffer[i] = text[i];
        i++;
    }
    while (i < 16U) {
        buffer[i] = ' ';
        i++;
    }
    buffer[16] = '\0';

    /* 只把变化的字符排进队列，I2C 留到 FlushDisplay 里慢慢写 */
    for (i = 0U; i < 16U; i++) {
        if (s_oled_cache[row][i] == buffer[i]) {
            continue;
        }
        if (s_oled_queue_count >= OLED_DIFF_QUEUE_MAX) {
            break;              /* 队列满：本轮丢掉，下一次差异比较还会补上 */
        }

        s_oled_queue[(uint8_t)((s_oled_queue_head + s_oled_queue_count) % OLED_DIFF_QUEUE_MAX)]
            .line = line;
        s_oled_queue[(uint8_t)((s_oled_queue_head + s_oled_queue_count) % OLED_DIFF_QUEUE_MAX)]
            .column = (uint8_t)(i + 1U);
        s_oled_queue[(uint8_t)((s_oled_queue_head + s_oled_queue_count) % OLED_DIFF_QUEUE_MAX)]
            .ch = buffer[i];
        s_oled_queue_count++;
        s_oled_cache[row][i] = buffer[i];
    }
}

/* 每拍调用：最多写 OLED_DIFF_PER_TICK 个字符，写完立刻返回 */
static void ChassisTask_FlushDisplay(void)
{
    uint8_t n;
    uint8_t written = 0U;

    for (n = 0U; n < OLED_DIFF_PER_TICK; n++) {
        ChassisOledCell_t cell;

        if (s_oled_queue_count == 0U) {
            break;
        }

        cell = s_oled_queue[s_oled_queue_head];
        s_oled_queue_head = (uint8_t)((s_oled_queue_head + 1U) % OLED_DIFF_QUEUE_MAX);
        s_oled_queue_count--;

        OLED_ShowChar(cell.line, cell.column, cell.ch);
        written++;
    }
    if (written > s_oled_chars_max) {
        s_oled_chars_max = written;
    }
}

/* 第 1 行链路状态：OK = 有人；SRCH = 在线但没人（搜索/暂停/缓慢搜索/确认期）；
 * -- = 断链（超时收不到帧）。
 * STOP（srch=4）期间 fresh=1，所以仍显示 OK —— 按要求 OLED 只有这三种字串。 */
static const char *ChassisTask_LinkStatusText(void)
{
    if (VisionLink_IsFresh() != 0U) {
        return "OK";
    }
    if (VisionLink_IsReceiving() != 0U) {
        return "SRCH";
    }
    return "--";
}

static void ChassisTask_RefreshDisplay(void)
{
    char text[24];                  /* 第 4 行最长 L:-700  R:+700，17 不够 */

    /* ---------- 第 1 行：模式 + 链路状态 ---------- */
    if (s_remote_stop != 0U) {
        (void)snprintf(text, sizeof(text), "Mode:STOP %s", ChassisTask_LinkStatusText());
    } else if ((s_mode == CHASSIS_MODE_LINE_TRACK) && (s_line_arrived != 0U)) {
        (void)snprintf(text, sizeof(text), "Mode:LINE DONE");
    } else if (s_mode == CHASSIS_MODE_LINE_TRACK) {
        (void)snprintf(text, sizeof(text), "Mode:LINE %s", ChassisTask_LinkStatusText());
    } else {
        (void)snprintf(text, sizeof(text), "Mode:FOLL %s", ChassisTask_LinkStatusText());
    }
    ChassisTask_ShowPaddedLine(1U, text);

    /* ---------- 第 2 行：模式相关 ---------- */
    if (s_mode == CHASSIS_MODE_LINE_TRACK) {
        LineSensor_t line = LineSensor_Get();
        uint8_t bit;

        text[0] = 'R';
        text[1] = 'a';
        text[2] = 'w';
        text[3] = ':';
        for (bit = 0U; bit < LINE_SENSOR_CHANNEL_COUNT; bit++) {
            text[4U + bit] = (((line.raw_levels >> bit) & 0x01U) != 0U) ? '1' : '0';
        }
        text[4U + LINE_SENSOR_CHANNEL_COUNT] = '\0';
        ChassisTask_ShowPaddedLine(2U, text);
    } else {
        /* 云台 yaw 偏角，按整度显示（内部是 0.1 度） */
        int16_t dev_deg = (int16_t)(VisionLink_GetYawDevTenthDeg() / 10);

        (void)snprintf(text, sizeof(text), "Dev:%+04d", (int)dev_deg);
        ChassisTask_ShowPaddedLine(2U, text);
    }

    /* ---------- 第 3 行：超声波距离 ---------- */
    if (DistanceSensor_IsValid() != 0U) {
        (void)snprintf(text, sizeof(text), "Dist:%03ucm", (unsigned)DistanceSensor_GetCm());
    } else {
        (void)snprintf(text, sizeof(text), "Dist:---cm");
    }
    ChassisTask_ShowPaddedLine(3U, text);

    /* ---------- 第 4 行：左右轮实际输出 ----------
     * 原来这里是 OLED_ShowString + ShowSignedNum 混着直接写屏，
     * 现在统一拼成一行字符串，走上面的差异刷新。 */
    (void)snprintf(text, sizeof(text), "L:%+04d  R:%+04d",
                   (int)Motor_GetLeftOutput(),
                   (int)Motor_GetRightOutput());
    ChassisTask_ShowPaddedLine(4U, text);
}

/* ------------------------------------------------------------ 对外接口 -- */

void ChassisTask_Init(void)
{
    uint32_t now_ms;

    /* 0) 1 ms 时基：所有超时、dage、速度环 dt 都依赖它，必须最先起来 */
    Delay_Init();

    /* 0.1) 在线调参参数表：必须早于任何控制律调用，
     *      否则第一拍读到的是全 0（车会按 0 增益跑一拍） */
    Params_Init();
    Blackbox_Init();

    /* 1) 电机先初始化：BSP 末尾会让电机滑行，确保上电不动 */
    Motor_Init();

    /* 2) 编码器（只做诊断，不参与控制） */
    BSP_Encoder_Init();

    /* 2.1) 里程计：以当前编码器计数为零点。
     *      必须在 BSP_Encoder_Init() 之后，否则读到的是未初始化的 0；
     *      也必须在第一次 Odometry_Update() 之前，否则第一拍会算进一个巨大增量。 */
    Odometry_Init();

    /* 3) 输入 */
    LineSensor_Init();
    BSP_Key_Init();

    /* 4) 双板链路 */
    VisionLink_Init();

    /* 5) 超声波测距 */
    DistanceSensor_Init();

    /* 6) 显示 */
    OLED_Init();
    OLED_Clear();

    /* 7) 时间基准与上电默认状态 */
    now_ms = Delay_GetMs();
    s_last_display_ms = now_ms;
    s_last_debug_ms = now_ms;
    s_key_last_change_ms = now_ms;
    s_key_last_raw = BSP_Key_IsDown(BSP_KEY_MODE);
    s_key_stable_down = 0U;
    s_mode = CHASSIS_MODE_LINE_TRACK;
    /* 上电默认进入 STOP：整车不上电就走。必须用手机发 mode1 / mode2 解锁，
     * 或按一下模式按键（按键会清掉 stop 并切换模式）。
     * 理由：上电瞬间云台还没对准、链路还没建立，先停住最安全。 */
    s_remote_stop = 1U;
    ChassisTask_ResetFollowState();

    /* P0 观测量的初值：统计窗口从「现在」开始，
     * 否则上电第一秒会打印出一个巨大的 sw。 */
    s_branch_code = 0U;
    s_branch_prev = 0U;
    s_branch_switch_count = 0U;
    s_branch_sw_per_sec = 0U;
    s_branch_win_ms = now_ms;

    /* 上电默认：电机滑行（安全状态） */
    Motor_Stop();

    ChassisTask_RefreshDisplay();
}

void ChassisTask_Loop(void)
{
    uint32_t now_ms;

    now_ms = Delay_GetMs();

    /* ---------- 1. 双板链路解析 ---------- */
    VisionLink_Poll();
    ChassisTask_MarkStage(1U, now_ms);

    /* 0) 里程计：每个控制拍读一次编码器计数（自持快照，不消费速度环的增量）。
     *    放在所有分支之前，保证停车/搜索/循迹/跟随都在累计；
     *    放在分支之后的话，一进搜索航向就会“丢一段”。 */
    Odometry_Update();

    /* ---------- 2. 超声波（内部按 60 ms 节流触发） ---------- */
    DistanceSensor_Poll();
    ChassisTask_MarkStage(2U, now_ms);

    /* ---------- 3. 循迹采样 ---------- */
    LineSensor_Update();
    ChassisTask_MarkStage(3U, now_ms);

    /* ---------- 4. 按键消抖与模式切换 ---------- */
    ChassisTask_PollKey(now_ms);
    ChassisTask_MarkStage(4U, now_ms);

    /* ---------- 4.5 蓝牙命令（mode1 / mode2 / stop） ---------- */
    ChassisTask_PollRemoteCommand();
    ChassisTask_MarkStage(5U, now_ms);

    /* ---------- 4.6 提示音（起跑 / 断链） ----------
     * 起跑 = STOP 锁存被解开（收到 mode1/mode2 或按了模式键），两声上行；
     * 断链 = 链路从"在线"变"离线"，长鸣 0.5 s。
     * 都是上沿触发，所以不会每拍都响。 */
    {
        uint8_t rx_now = VisionLink_IsReceiving();

        if ((s_beep_remote_stop_prev != 0U) && (s_remote_stop == 0U)) {
            VisionLink_SendBeep(LINK_BEEP_START);
        }
        s_beep_remote_stop_prev = s_remote_stop;

        if ((s_beep_link_prev != 0U) && (rx_now == 0U)) {
            VisionLink_SendBeep(LINK_BEEP_LINK_LOST);
        }
        s_beep_link_prev = rx_now;
    }

    /* ---------- 4.7 模式上报（给云台板用，2026-11-23） ----------
     * 云台靠它决定"丢人后要不要扫视"：只在模式 1（循迹）才扫。
     * 模式变化立刻发一次；之后每 1 s 补一次，丢一帧也能在 1 s 内恢复。 */
    {
        uint8_t mode_code;
        uint32_t now_mode_ms = Delay_GetMs();

        if (s_remote_stop != 0U) {
            mode_code = LINK_MODE_STOP;
        } else if (s_mode == CHASSIS_MODE_LINE_TRACK) {
            mode_code = LINK_MODE_LINE;
        } else {
            mode_code = LINK_MODE_FOLLOW;
        }

        if ((mode_code != s_mode_reported) ||
            ((uint32_t)(now_mode_ms - s_mode_report_ms) >= 1000U)) {
            VisionLink_SendMode(mode_code);
            s_mode_reported = mode_code;
            s_mode_report_ms = now_mode_ms;
        }
    }

    /* ---------- 5. 按模式分派 ---------- */
    if (s_remote_stop != 0U) {
        /* 蓝牙 stop 生效期间：整车停住（不前进也不转）。
         * 退出方式：手机再发 mode1/mode2，或者按一下模式按键。 */
        ChassisTask_SetBranch(0U);
        Motor_Stop();
    } else {
        switch (s_mode) {
        case CHASSIS_MODE_LINE_TRACK:
            ChassisTask_UpdateLineTrack();
            break;
        case CHASSIS_MODE_FOLLOW:
            ChassisTask_UpdateFollow();
            break;
        default:
            s_mode = CHASSIS_MODE_LINE_TRACK;
            ChassisTask_SetBranch(0U);
            Motor_Stop();
            break;
        }
    }
    ChassisTask_MarkStage(7U, now_ms);

    /* ---------- 6. 显示（分片：组文本 100 ms 一次，写屏每拍一点点） ---------- */
    if ((uint32_t)(now_ms - s_last_display_ms) >= CHASSIS_DISPLAY_PERIOD_MS) {
        s_last_display_ms = now_ms;
        ChassisTask_RefreshDisplay();       /* 只组文本 + 压差异队列，不碰 I2C */
    }
    ChassisTask_FlushDisplay();             /* 每拍最多 6 个字符 */
    ChassisTask_MarkStage(6U, now_ms);

    /* ---------- 7. 节流串口回传 ---------- */
    if ((uint32_t)(now_ms - s_last_debug_ms) >= CHASSIS_DEBUG_PERIOD_MS) {
        char text[256];   /* 加上 th/od/eL/eR 后最长约 244 字节 */
        LineSensor_t line = LineSensor_Get();
        DistanceSensor_t dist = DistanceSensor_Get();
        int16_t sp_left = Motor_GetLeftSpeed();
        int16_t sp_right = Motor_GetRightSpeed();
        /* 原始计数（不是增量、不是换算后的速度）。
         * 用途只有一个：判断编码器到底有没有在计数。
         *   手转轮子时 eL/eR 应平滑地递增或递减；
         *   若某个数乱跳、偶尔不动、忽增忽减 → 该路 A/B 有一相没通。 */
        int16_t ec_left = BSP_Encoder_ReadLeftCount();
        int16_t ec_right = BSP_Encoder_ReadRightCount();
        unsigned dage_ms;

        s_last_debug_ms = now_ms;

        /* P0 观测：每满 1 s 结算一次「分支切换次数」，作为 sw= 输出。
         * 用固定的 1 s 窗口而不是「两次打印之间」，是因为打印周期是 500 ms，
         * 不按窗口结算的话同一个数字会被打印两次，看着像抖动减半。 */
        if ((uint32_t)(now_ms - s_branch_win_ms) >= 1000U) {
            s_branch_sw_per_sec = s_branch_switch_count;
            s_branch_switch_count = 0U;
            s_branch_win_ms = now_ms;
        }

        /* 距离「多久没有新采样」：直接取距离模块的 age（距上次被接受的采样）。
         * 注意 dist.valid 在保持期间仍为 1，不能用它算 dage，否则永远读到 0。
         * 这个数大，说明超声在跟随过程中成片丢，而不是「偶尔一格」。 */
        dage_ms = (unsigned)DistanceSensor_GetAgeMs();

        /* 用 snprintf 而不是 sprintf：这一行字段多，极端情况下（全是最长数值）
         * 可能超出缓冲，sprintf 会直接越界踩栈。 */
        (void)snprintf(text, sizeof(text),
                      "C: mode=%d raw=%02X pos=%+04d lost=%d out=%d/%d dev=%d link=%d "
                      "dist=%u/%u us=%u sp=%d/%d err=%lu srch=%d "
                      "st=%u sw=%u dage=%u oled=%u lp=%u/%u oc=%u th=%+04d od=%u eL=%d eR=%d\r\n",
                      (int)s_mode,
                      (unsigned)line.raw_levels,
                      (int)line.position,
                      (int)line.lost,
                      (int)Motor_GetLeftOutput(),
                      (int)Motor_GetRightOutput(),
                      (int)VisionLink_GetYawDevTenthDeg(),
                      (int)VisionLink_IsFresh(),
                      (unsigned)dist.distance_cm,
                      (unsigned)dist.valid,
                      (unsigned)dist.echo_us,
                      (int)sp_left,
                      (int)sp_right,
                      (unsigned long)VisionLink_Get().checksum_error_count,
                      (int)s_search_state,
                      (unsigned)s_branch_code,
                      (unsigned)s_branch_sw_per_sec,
                      dage_ms,
                      (unsigned)OLED_IsPresent(),
                      (unsigned)s_loop_max_ms,
                      (unsigned)s_loop_max_stage,
                      (unsigned)s_oled_chars_max,
                      (int)Odometry_GetHeadingTenthDeg(),
                      (unsigned)Odometry_GetDistanceCm(),
                      (int)ec_left,
                      (int)ec_right);
        s_loop_max_ms = 0U;             /* 每个窗口重新取最大值 */
        s_loop_max_stage = 0U;          /* 段号一起清零，否则一直停在上一条 */
        s_oled_chars_max = 0U;          /* 上一秒的 OLED 字符峰值已打印，重新统计 */
        BSP_UART_SendString(text);
    }

    /* ---------- 黑匣子（2026-11-23，方案 B） ----------
     * 1) 每 100 ms 记一条到 RAM：只碰内存，零停顿；
     * 2) 只在"整车已停"时整块刷进 Flash —— 车本来就不动，
     *    150~250 ms 的擦写停顿无害，正常跑起来一秒都不停；
     * 3) DumpStep 每拍最多发 4 行，不阻塞 5 ms 控制环。
     * 导出：蓝牙发 bb → 云台板转 0x03(op=5) → 这里逐行发 "BB,..." 文本。 */
    if ((uint32_t)(now_ms - s_bb_ms) >= CHASSIS_BLACKBOX_PERIOD_MS) {
        BlackboxSample_t bs;
        LineSensor_t bb_line = LineSensor_Get();
        DistanceSensor_t bb_dist = DistanceSensor_Get();

        s_bb_ms = now_ms;
        bs.t_ms = (uint16_t)now_ms;
        bs.dev = VisionLink_Get().yaw_dev_tenth_deg;
        bs.out_l = (int16_t)Motor_GetLeftOutput();
        bs.out_r = (int16_t)Motor_GetRightOutput();
        bs.sp_l = Motor_GetLeftSpeed();
        bs.sp_r = Motor_GetRightSpeed();
        bs.dist_cm = bb_dist.distance_cm;
        bs.pos = (int8_t)bb_line.position;
        bs.st = s_branch_code;
        Blackbox_Record(&bs);
    }

    /* 安全时刻落盘：整车停（蓝牙/按键 stop）或十字路口停（st=13）
     * 只落一次，等车重新动起来再解锁，避免每拍都擦 Flash。 */
    if ((s_remote_stop != 0U) || (s_branch_code == 13U)) {
        if (s_bb_flushed == 0U) {
            s_bb_flushed = 1U;
            (void)Blackbox_Flush();
        }
    } else {
        s_bb_flushed = 0U;
    }

    Blackbox_DumpStep();

    /* ---------- 波形输出（VOFA+ FireWater，2026-11-23） ----------
     * 开关是参数 wave（id 18）：蓝牙发 `set wave 1` 打开、`set wave 0` 关闭。
     * 打开后每隔 CHASSIS_WAVE_PERIOD_MS 发一行**纯数字**，逗号分隔、\n 结尾，
     * 这正是 VOFA+ 的 FireWater 协议，选 FireWater 控件即可出波形。
     * 8 个通道顺序（在 VOFA+ 里按这个顺序命名）：
     *   dev 云台偏角(0.1°) / pos 循迹偏差 / outL outR 占空比 /
     *   spL spR 实测速度‰ / dist 距离cm / st 分支码
     * 全部是整数，不用 %f —— 软浮点 printf 一次要几毫秒，会把 5 ms 控制环拖垮。 */
    if (Params_Get(PARAM_ID_WAVE) != 0) {
        if ((uint32_t)(now_ms - s_wave_ms) >= CHASSIS_WAVE_PERIOD_MS) {
            char wave[64];
            LineSensor_t line = LineSensor_Get();
            DistanceSensor_t dist = DistanceSensor_Get();

            s_wave_ms = now_ms;
            (void)snprintf(wave, sizeof(wave), "%d,%d,%d,%d,%d,%d,%u,%u\n",
                           (int)VisionLink_Get().yaw_dev_tenth_deg,
                           (int)line.position,
                           (int)Motor_GetLeftOutput(),
                           (int)Motor_GetRightOutput(),
                           (int)Motor_GetLeftSpeed(),
                           (int)Motor_GetRightSpeed(),
                           (unsigned)dist.distance_cm,
                           (unsigned)s_branch_code);
            BSP_UART_SendString(wave);
        }
    }

    /* 记录本拍耗时：串口行里的 lp= 就是它过去 1 s 的最大值 */
    ChassisTask_MarkStage(8U, now_ms);
}

ChassisMode_t ChassisTask_GetMode(void)
{
    return s_mode;
}
