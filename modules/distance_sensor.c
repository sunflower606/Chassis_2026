#include "distance_sensor.h"

#include "bsp_ultrasonic.h"
#include "chassis_config.h"
#include "Delay.h"

static DistanceSensor_t s_distance;

/* 上一次触发时刻，ms */
static uint32_t s_last_trigger_ms = 0U;

/* 1 = 已触发、还在等回波（用于超时判定） */
static uint8_t s_waiting_echo = 0U;

/* ---------------- 滤波与保持状态（2026-11-14 新增） ---------------- */

/* 环形缓冲区：只存「通过野值门限」的原始读数 */
static uint16_t s_ring[CHASSIS_DISTANCE_MEDIAN_WINDOW];
static uint8_t s_ring_count = 0U;   /* 当前有效样本数，最多 MEDIAN_WINDOW */
static uint8_t s_ring_index = 0U;   /* 下一个要写的位置 */

static uint16_t s_last_raw_cm = 0U; /* 上一次被接受的原始读数（野值门限的参考） */
static uint8_t s_has_value = 0U;    /* 1 = 至少接受过一次读数 */
static uint8_t s_reject_streak = 0U;/* 连续被野值门限拒绝的次数 */
static uint32_t s_last_ok_ms = 0U;  /* 最近一次「接受」的时刻，ms */

/* 从未采到过读数时对外报告的 age，取一个明显很大的值 */
#define DISTANCE_AGE_NEVER  0xFFFFU

/** @brief 对环形缓冲区里的样本取中值（样本数 ≤ 5，直接插入排序）。 */
static uint16_t DistanceSensor_Median(void)
{
    uint16_t tmp[CHASSIS_DISTANCE_MEDIAN_WINDOW];
    uint8_t n = s_ring_count;
    uint8_t i;
    uint8_t j;

    if (n == 0U) {
        return 0U;
    }

    for (i = 0U; i < n; i++) {
        tmp[i] = s_ring[i];
    }

    for (i = 1U; i < n; i++) {
        uint16_t key = tmp[i];
        j = i;
        while (j > 0U && tmp[j - 1U] > key) {
            tmp[j] = tmp[j - 1U];
            j--;
        }
        tmp[j] = key;
    }

    return tmp[n / 2U];
}

/** @brief 把一个新读数送进滤波器（含野值门限与中值）。 */
static void DistanceSensor_AcceptRaw(uint16_t raw_cm, uint32_t now_ms)
{
    /* 门限参考的是「上一条被接受的读数」，不是滤波输出，
     * 否则滤波滞后会让正常的新值也被判成野值。 */
    if (s_has_value != 0U && s_reject_streak < CHASSIS_DISTANCE_JUMP_FORCE) {
        uint16_t diff;

        if (raw_cm >= s_last_raw_cm) {
            diff = (uint16_t)(raw_cm - s_last_raw_cm);
        } else {
            diff = (uint16_t)(s_last_raw_cm - raw_cm);
        }

        if (diff > CHASSIS_DISTANCE_JUMP_CM) {
            /* 单次跳太远：当野值丢弃，保持旧值 */
            s_reject_streak++;
            s_distance.rejected_count++;
            return;
        }
    }

    /* 连续被拒到上限：说明不是偶发干扰，是目标真的换了位置（或真的跑远了）。
     * 清空历史，避免中值被旧样本拖住。 */
    if (s_reject_streak >= CHASSIS_DISTANCE_JUMP_FORCE) {
        s_ring_count = 0U;
        s_ring_index = 0U;
    }

    s_reject_streak = 0U;
    s_last_raw_cm = raw_cm;

    s_ring[s_ring_index] = raw_cm;
    s_ring_index = (uint8_t)((s_ring_index + 1U) % CHASSIS_DISTANCE_MEDIAN_WINDOW);
    if (s_ring_count < CHASSIS_DISTANCE_MEDIAN_WINDOW) {
        s_ring_count++;
    }

    s_distance.distance_cm = DistanceSensor_Median();
    s_distance.valid_count++;
    s_has_value = 1U;
    s_last_ok_ms = now_ms;
}

/** @brief 每个控制周期都刷新一次「可用 / 新鲜 / 年龄」三个标志。 */
static void DistanceSensor_RefreshFlags(uint32_t now_ms)
{
    uint32_t age = (uint32_t)(now_ms - s_last_ok_ms);

    if (s_has_value == 0U) {
        s_distance.valid = 0U;
        s_distance.fresh = 0U;
        s_distance.age_ms = DISTANCE_AGE_NEVER;
        return;
    }

    s_distance.age_ms = (age > (uint32_t)DISTANCE_AGE_NEVER) ? DISTANCE_AGE_NEVER
                                                             : (uint16_t)age;
    s_distance.fresh = (age <= CHASSIS_DISTANCE_FRESH_MS) ? 1U : 0U;
    s_distance.valid = (age <= CHASSIS_DISTANCE_HOLD_MS) ? 1U : 0U;
}

void DistanceSensor_Init(void)
{
    uint8_t i;

    s_distance.distance_cm = 0U;
    s_distance.echo_us = 0U;
    s_distance.valid = 0U;
    s_distance.fresh = 0U;
    s_distance.age_ms = DISTANCE_AGE_NEVER;
    s_distance.last_update_ms = 0U;
    s_distance.measure_count = 0U;
    s_distance.valid_count = 0U;
    s_distance.invalid_count = 0U;
    s_distance.rejected_count = 0U;

    for (i = 0U; i < CHASSIS_DISTANCE_MEDIAN_WINDOW; i++) {
        s_ring[i] = 0U;
    }
    s_ring_count = 0U;
    s_ring_index = 0U;
    s_last_raw_cm = 0U;
    s_has_value = 0U;
    s_reject_streak = 0U;
    s_last_ok_ms = 0U;

    s_last_trigger_ms = Delay_GetMs();
    s_waiting_echo = 0U;

    BSP_Ultrasonic_Init();
}

void DistanceSensor_Poll(void)
{
    uint32_t now_ms = Delay_GetMs();
    uint16_t echo_us = 0U;

    /* ---------- 1. 到周期就发一次触发 ---------- */
    if ((uint32_t)(now_ms - s_last_trigger_ms) >= CHASSIS_ULTRASONIC_PERIOD_MS) {
        s_last_trigger_ms = now_ms;
        s_waiting_echo = 1U;
        s_distance.measure_count++;

        BSP_Ultrasonic_Trigger();
    }

    /* ---------- 2. 有回波就换算、过滤 ---------- */
    if (BSP_Ultrasonic_TakeEchoUs(&echo_us) != 0U) {
        uint32_t distance_cm = (uint32_t)echo_us / CHASSIS_ULTRASONIC_US_PER_CM;

        s_waiting_echo = 0U;
        s_distance.echo_us = echo_us;
        s_distance.last_update_ms = now_ms;

        /* 量程判定：小于 MIN 是盲区读数不可信，大于 MAX 已超出模块能力。
         * 超量程不算「被接受的采样」，保持计时不刷新。 */
        if (distance_cm >= (uint32_t)CHASSIS_ULTRASONIC_MIN_CM &&
            distance_cm <= (uint32_t)CHASSIS_ULTRASONIC_MAX_CM) {
            DistanceSensor_AcceptRaw((uint16_t)distance_cm, now_ms);
        } else {
            s_distance.invalid_count++;
        }
    }
    /* ---------- 3. 等太久没有回波：本次作废，但不清空已保持的值 ---------- */
    else if (s_waiting_echo != 0U &&
             (uint32_t)(now_ms - s_last_trigger_ms) > CHASSIS_ULTRASONIC_ECHO_TIMEOUT_MS) {
        s_waiting_echo = 0U;
        s_distance.invalid_count++;

        /* 关键：只捕到上升沿时，捕获极性停在下降沿，
         * 不复位的话下一次测量永远接不上（表现为「从此再也没有距离」）。 */
        BSP_Ultrasonic_ResetEcho();
    } else {
        /* 还在等回波：本拍什么都不做 */
    }

    /* ---------- 4. 无论本拍有没有新数据，都要刷新「还能不能用」 ---------- */
    DistanceSensor_RefreshFlags(now_ms);
}

DistanceSensor_t DistanceSensor_Get(void)
{
    return s_distance;
}

uint16_t DistanceSensor_GetCm(void)
{
    return s_distance.distance_cm;
}

uint8_t DistanceSensor_IsValid(void)
{
    return s_distance.valid;
}

uint8_t DistanceSensor_IsFresh(void)
{
    return s_distance.fresh;
}

uint16_t DistanceSensor_GetAgeMs(void)
{
    return s_distance.age_ms;
}
