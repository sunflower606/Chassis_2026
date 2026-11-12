#include "distance_sensor.h"

#include "bsp_ultrasonic.h"
#include "chassis_config.h"
#include "Delay.h"

static DistanceSensor_t s_distance;

/* 上一次触发时刻，ms */
static uint32_t s_last_trigger_ms = 0U;

/* 1 = 已触发、还在等回波（用于超时判定） */
static uint8_t s_waiting_echo = 0U;

void DistanceSensor_Init(void)
{
    s_distance.distance_cm = 0U;
    s_distance.echo_us = 0U;
    s_distance.valid = 0U;
    s_distance.measure_count = 0U;
    s_distance.valid_count = 0U;
    s_distance.invalid_count = 0U;

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

    /* ---------- 2. 有回波就换算 ---------- */
    if (BSP_Ultrasonic_TakeEchoUs(&echo_us) != 0U) {
        uint32_t distance_cm = (uint32_t)echo_us / CHASSIS_ULTRASONIC_US_PER_CM;

        s_waiting_echo = 0U;
        s_distance.echo_us = echo_us;
        s_distance.last_update_ms = now_ms;

        /* 量程判定：小于 MIN 是盲区读数不可信，大于 MAX 已超出模块能力 */
        if (distance_cm >= (uint32_t)CHASSIS_ULTRASONIC_MIN_CM &&
            distance_cm <= (uint32_t)CHASSIS_ULTRASONIC_MAX_CM) {
            s_distance.distance_cm = (uint16_t)distance_cm;
            s_distance.valid = 1U;
            s_distance.valid_count++;
        } else {
            s_distance.valid = 0U;
            s_distance.invalid_count++;
        }

        return;
    }

    /* ---------- 3. 等太久没有回波：判无效 ---------- */
    if (s_waiting_echo != 0U &&
        (uint32_t)(now_ms - s_last_trigger_ms) > CHASSIS_ULTRASONIC_ECHO_TIMEOUT_MS) {
        s_waiting_echo = 0U;
        s_distance.valid = 0U;
        s_distance.invalid_count++;

        /* 关键：只捕到上升沿时，捕获极性停在下降沿，
         * 不复位的话下一次测量永远接不上（表现为「从此再也没有距离」）。 */
        BSP_Ultrasonic_ResetEcho();
    }
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
