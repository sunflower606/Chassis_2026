#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

/* ============================================================================
 * Module：超声波测距（HC-SR04，用于模式 2 的保持车距）
 *
 * 职责：
 *   1) 按固定周期触发测量；
 *   2) 把回波微秒换算成厘米；
 *   3) 量程 / 超时判定；
 *   4) **中值滤波 + 野值剔除 + 短时保持**（2026-11-14 新增）。
 *
 * 上层（chassis_task）只关心 GetCm() 和 IsValid()：
 *   太远 -> 前进，太近 -> 后退，无效 -> 停止前进、改原地对准。
 *
 * 为什么要有「保持」：
 *   实测一次 11.5 s 的采样里，39% 的读数是无效的，而且不是零散丢，
 *   是成片丢（最长连续 1.8 s 没有回波）。原实现在「一次没回波」的下一拍
 *   就把距离判为无效，上层立刻 forward=0 切「原地对准」，
 *   表现为「走两步停一下、还扭一下头」。
 *   现在改为：丢帧期间沿用最后一次通过筛选的值，最多保持
 *   CHASSIS_DISTANCE_HOLD_MS；超过这个时间才对外报无效。
 *
 * 两级过滤：
 *   野值门限  —— 与上次接受的读数相差 > CHASSIS_DISTANCE_JUMP_CM 就丢弃
 *                （人 60 ms 走不了 40 cm，超过必是干扰或串扰）；
 *   中值滤波  —— 最近 N 次接受的读数取中值，抹掉偶发的半格跳变。
 *
 * 测量周期 60 ms 是 HC-SR04 的硬要求：两次测量间隔小于 60 ms 时，
 * 上一次的余波会被当成这一次的回波，读数会突然变小（看起来像「人突然变近」）。
 * ========================================================================== */

#include <stdint.h>

/** @brief 测距快照。 */
typedef struct {
    uint16_t distance_cm;       /* 滤波后的距离，单位 cm（保持期间就是最后有效值） */
    uint16_t echo_us;           /* 最近一次回波宽度，单位 us（诊断用） */
    uint8_t valid;              /* 1 = 当前距离可参与控制（新鲜值或保持中的值） */
    uint8_t fresh;              /* 1 = 最近 FRESH_MS 内拿到过新采样（未进保持） */
    uint16_t age_ms;            /* 距上次「被接受的采样」过去多少 ms（诊断用） */
    uint32_t last_update_ms;    /* 最近一次取到回波的时刻，ms */
    uint32_t measure_count;     /* 触发次数 */
    uint32_t valid_count;       /* 有效结果次数（量程内且通过筛选） */
    uint32_t invalid_count;     /* 无效次数（超时 / 超量程） */
    uint32_t rejected_count;    /* 被野值门限丢弃的次数 */
} DistanceSensor_t;

/** @brief 初始化超声波 BSP 与滤波器。 */
void DistanceSensor_Init(void);

/** @brief 每个控制周期调用一次：到周期就触发，并取回波 / 判超时 / 刷新标志。 */
void DistanceSensor_Poll(void);

/** @brief 取快照（值拷贝）。 */
DistanceSensor_t DistanceSensor_Get(void);

/** @return 滤波后的距离，单位 cm；无效时返回值无意义，需先看 IsValid。 */
uint16_t DistanceSensor_GetCm(void);

/** @return 1 = 当前距离可参与控制（含「保持中」的值，最长 HOLD_MS）。 */
uint8_t DistanceSensor_IsValid(void);

/** @return 1 = 最近一次采样是新鲜的有效值（未进入保持）。 */
uint8_t DistanceSensor_IsFresh(void);

/** @return 距上次被接受的采样过去多少 ms；从未采到过时返回一个很大的值。 */
uint16_t DistanceSensor_GetAgeMs(void);

#endif /* DISTANCE_SENSOR_H */
