#include "chassis_task.h"

#include <stddef.h>
#include <stdio.h>

#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "chassis_config.h"
#include "Delay.h"
#include "distance_sensor.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "oled.h"
#include "vision_link.h"

/* STOP 的确认时间必须短于链路超时：否则「连续看到人」还没攒满，
 * 链路就先判超时（fresh=0），永远退不出搜索。 */
#if (CHASSIS_SEARCH_HOLD_MS >= CHASSIS_LINK_TIMEOUT_MS)
#error "CHASSIS_SEARCH_HOLD_MS must be < CHASSIS_LINK_TIMEOUT_MS"
#endif

/* 当前模式 */
static ChassisMode_t s_mode = CHASSIS_MODE_LINE_TRACK;

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
}

/* 进入搜索：优先朝人最后出现的方向转；偏角在死区内则沿用上次的方向。 */
static void ChassisTask_EnterSearch(void)
{
    if (s_last_dev_tenth > (int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG) {
        s_search_dir = 1;
    } else if (s_last_dev_tenth < -(int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG) {
        s_search_dir = -1;
    } else {
        /* 偏角太小或没有历史：保留上一次的 s_search_dir（初值 +1） */
    }

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
        return (int32_t)s_search_dir * (int32_t)CHASSIS_SEARCH_SPIN_DUTY;
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
    /* TODO(循迹实现时)：
     *   1) 若 LineSensor 报告脱线 -> Motor_SetSpeed(0, 0) 并原地等待；
     *   2) 否则按线位置计算差速（注意左右轮方向的实测验证）；
     *   3) 输出必须限幅，并依赖 Motor_SetSpeed 内部的斜率限制。
     *
     * 现在：保持停车。 */
    Motor_SetSpeed(0, 0);
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
    int32_t steer = 0;
    int32_t forward_speed = 0;
    int32_t left;
    int32_t right;
    const int32_t speed_max = (int32_t)CHASSIS_FOLLOW_SPEED_MAX;
    const int32_t duty_max = (int32_t)CHASSIS_MOTOR_DUTY_MAX;

    /* ---------- 0. 近距急停（最高优先级，迟滞） ----------
     * 有效距离 < STOP 时整车停（前进/转向全清零）；回到 >= RESUME 才解除。
     * 搜索态同样适用：离得太近就停车不转。
     * 本保护与 CHASSIS_FOLLOW_REQUIRE_DISTANCE 是叠加关系，不改变后者行为。 */
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
        Motor_SetSpeed(0, 0);
        return;
    }

    /* ---------- 1. 断链：停车，搜索状态清零 ---------- */
    /* IsReceiving() 只看帧是否超时；IsFresh() 额外要求 detected=1。
     * 两者必须分开，否则「线松了」会被误当成「人丢了」而原地转圈。 */
    if (VisionLink_IsReceiving() == 0U) {
        ChassisTask_ResetSearch();
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
         * 保留一次清零，防止任何漏掉的路径把旧搜索计时带进跟随。 */
        ChassisTask_ResetSearch();
        s_last_dev_tenth = link.yaw_dev_tenth_deg;

        /* ---------- 3.1 转向：云台偏角 ---------- */
        dev_tenth = link.yaw_dev_tenth_deg;

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
        } else {
#if (CHASSIS_FOLLOW_REQUIRE_DISTANCE != 0)
            /* 测不到有效距离，但视觉已确认前方有人：
             * 不前进也不后退（不知道多远就不敢开），只按云台偏角「原地转向对准人」。
             *
             * 转多快**不看偏角大小**，只看偏角方向：
             *   偏角为正 -> 一个方向转，偏角为负 -> 反方向转，速度恒为
             *   CHASSIS_SEARCH_SPIN_DUTY（与「丢人转圈找人」同速，250‰）。
             * 原因：对准是「转过去」这个动作，不是「按偏差比例追」，
             *   用增益会让大偏角时甩得很快、小偏角时又磨蹭。
             * 偏角落进死区（已对准）-> steer 归零 -> 车停住等人。
             * 一旦超声恢复有效，下一拍就走上面的距离闭环，无缝接回跟随。
             * 原地转不前进，撞不到东西；近距急停（<25cm）仍在最前面兜底。 */
            if (dev_tenth > 0) {
                steer = (int32_t)CHASSIS_SEARCH_SPIN_DUTY;
            } else if (dev_tenth < 0) {
                steer = -(int32_t)CHASSIS_SEARCH_SPIN_DUTY;
            } else {
                steer = 0;
            }
            steer = steer * (int32_t)CHASSIS_FOLLOW_STEER_SIGN;
            forward_speed = 0;
#else
            /* 退化成定速前进（仅调试用） */
            forward_speed = (int32_t)CHASSIS_FOLLOW_BASE_SPEED;
#endif
        }
    } else {
        /* ---------- 4. 在线但丢人：确认后原地转圈搜索 ---------- */
        s_lost_ms += (uint32_t)CHASSIS_CONTROL_PERIOD_MS;

        /* 确认期内先停车：单帧/几帧漏检不该让车突然转圈 */
        if (s_lost_ms < (uint32_t)CHASSIS_SEARCH_CONFIRM_MS) {
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

/* ------------------------------------------------------------ 内部：显示 -- */

/* 把 text 补空格到 16 列后整行写出，避免切换模式时残留旧字符 */
static void ChassisTask_ShowPaddedLine(uint8_t line, const char *text)
{
    char buffer[17];
    uint8_t i = 0U;

    while (text[i] != '\0' && i < 16U) {
        buffer[i] = text[i];
        i++;
    }
    while (i < 16U) {
        buffer[i] = ' ';
        i++;
    }
    buffer[16] = '\0';

    OLED_ShowString(line, 1U, buffer);
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
    char text[17];

    /* ---------- 第 1 行：模式 + 链路状态 ---------- */
    if (s_mode == CHASSIS_MODE_LINE_TRACK) {
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

        (void)sprintf(text, "Dev:%+04d", (int)dev_deg);
        ChassisTask_ShowPaddedLine(2U, text);
    }

    /* ---------- 第 3 行：超声波距离 ---------- */
    if (DistanceSensor_IsValid() != 0U) {
        (void)sprintf(text, "Dist:%03ucm", (unsigned)DistanceSensor_GetCm());
    } else {
        (void)sprintf(text, "Dist:---cm");
    }
    ChassisTask_ShowPaddedLine(3U, text);

    /* ---------- 第 4 行：左右轮实际输出 ---------- */
    OLED_ShowString(4U, 1U, "L:");
    OLED_ShowSignedNum(4U, 3U, (int32_t)Motor_GetLeftOutput(), 3U);
    OLED_ShowString(4U, 8U, "R:");
    OLED_ShowSignedNum(4U, 10U, (int32_t)Motor_GetRightOutput(), 3U);
}

/* ------------------------------------------------------------ 对外接口 -- */

void ChassisTask_Init(void)
{
    uint32_t now_ms;

    /* 1) 电机先初始化：BSP 末尾会让电机滑行，确保上电不动 */
    Motor_Init();

    /* 2) 编码器（只做诊断，不参与控制） */
    BSP_Encoder_Init();

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
    ChassisTask_ResetFollowState();

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

    /* ---------- 2. 超声波（内部按 60 ms 节流触发） ---------- */
    DistanceSensor_Poll();

    /* ---------- 3. 循迹采样 ---------- */
    LineSensor_Update();

    /* ---------- 4. 按键消抖与模式切换 ---------- */
    ChassisTask_PollKey(now_ms);

    /* ---------- 5. 按模式分派 ---------- */
    switch (s_mode) {
    case CHASSIS_MODE_LINE_TRACK:
        ChassisTask_UpdateLineTrack();
        break;
    case CHASSIS_MODE_FOLLOW:
        ChassisTask_UpdateFollow();
        break;
    default:
        s_mode = CHASSIS_MODE_LINE_TRACK;
        Motor_Stop();
        break;
    }

    /* ---------- 6. 节流显示 ---------- */
    if ((uint32_t)(now_ms - s_last_display_ms) >= CHASSIS_DISPLAY_PERIOD_MS) {
        s_last_display_ms = now_ms;
        ChassisTask_RefreshDisplay();
    }

    /* ---------- 7. 节流串口回传 ---------- */
    if ((uint32_t)(now_ms - s_last_debug_ms) >= CHASSIS_DEBUG_PERIOD_MS) {
        char text[144];
        LineSensor_t line = LineSensor_Get();
        DistanceSensor_t dist = DistanceSensor_Get();
        int16_t enc_left = BSP_Encoder_GetLeftDelta();
        int16_t enc_right = BSP_Encoder_GetRightDelta();

        s_last_debug_ms = now_ms;
        /* 用 snprintf 而不是 sprintf：这一行字段多，极端情况下（全是最长数值）
         * 可能超出缓冲，sprintf 会直接越界踩栈。 */
        (void)snprintf(text, sizeof(text),
                      "C: mode=%d raw=%02X lost=%d out=%d/%d dev=%d link=%d "
                      "dist=%u/%u us=%u enc=%d/%d err=%lu srch=%d\r\n",
                      (int)s_mode,
                      (unsigned)line.raw_levels,
                      (int)line.lost,
                      (int)Motor_GetLeftOutput(),
                      (int)Motor_GetRightOutput(),
                      (int)VisionLink_GetYawDevTenthDeg(),
                      (int)VisionLink_IsFresh(),
                      (unsigned)dist.distance_cm,
                      (unsigned)dist.valid,
                      (unsigned)dist.echo_us,
                      (int)enc_left,
                      (int)enc_right,
                      (unsigned long)VisionLink_Get().checksum_error_count,
                      (int)s_search_state);
        BSP_UART_SendString(text);
    }
}

ChassisMode_t ChassisTask_GetMode(void)
{
    return s_mode;
}
