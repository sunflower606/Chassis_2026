#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

/* ============================================================================
 * Module：超声波测距（HC-SR04，用于模式 2 的保持车距）
 *
 * 职责：
 *   1) 按固定周期触发测量；
 *   2) 把回波微秒换算成厘米；
 *   3) 量程 / 超时判定，向上只报「有效距离」。
 *
 * 上层（chassis_task）只关心 GetCm() 和 IsValid()：
 *   太远 -> 前进，太近 -> 后退，无效 -> 不做前后动作。
 *
 * 测量周期 60 ms 是 HC-SR04 的硬要求：两次测量间隔小于 60 ms 时，
 * 上一次的余波会被当成这一次的回波，读数会突然变小（看起来像「人突然变近」）。
 * ========================================================================== */

#include <stdint.h>

/** @brief 测距快照。 */
typedef struct {
    uint16_t distance_cm;       /* 最近一次**有效**距离，单位 cm */
    uint16_t echo_us;           /* 最近一次回波宽度，单位 us（诊断用） */
    uint8_t valid;              /* 1 = 最近一次测量在量程内且未超时 */
    uint32_t last_update_ms;    /* 最近一次取到回波的时刻，ms */
    uint32_t measure_count;     /* 触发次数 */
    uint32_t valid_count;       /* 有效结果次数 */
    uint32_t invalid_count;     /* 无效（超时 / 超量程）次数 */
} DistanceSensor_t;

/** @brief 初始化超声波 BSP。 */
void DistanceSensor_Init(void);

/** @brief 每个控制周期调用一次：到周期就触发，并取回波 / 判超时。 */
void DistanceSensor_Poll(void);

/** @brief 取快照（值拷贝）。 */
DistanceSensor_t DistanceSensor_Get(void);

/** @return 最近一次有效距离，单位 cm；无效时返回值无意义，需先看 IsValid。 */
uint16_t DistanceSensor_GetCm(void);

/** @return 1 = 当前距离可参与控制；0 = 无效（超时或超量程）。 */
uint8_t DistanceSensor_IsValid(void);

#endif /* DISTANCE_SENSOR_H */
