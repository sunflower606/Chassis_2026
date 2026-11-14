#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/* ============================================================================
 * Module：差速底盘电机驱动
 *
 * 硬件：后双轮差速驱动（后轴两个 MG513 霍尔编码器电机 + H 桥）
 *
 * 本模块负责四件事：
 *   1) 目标值限幅（CHASSIS_MOTOR_DUTY_MIN / MAX）；
 *   2) **轮速闭环**：目标速度 -> PI（编码器反馈）-> 占空比，
 *      见 modules/speed_loop.c；CHASSIS_SPEED_LOOP_ENABLE=0 时退回开环；
 *   3) 斜率限制（每拍最多变化 CHASSIS_MOTOR_RAMP_PER_TICK），
 *      避免突然满占空比造成电流冲击、以及停车点头；
 *   4) 调用 BSP 输出到 TIM3 + H 桥方向脚。
 *
 * ⚠️ 单位约定：**正 = 后退、负 = 前进**（实机验证过的极性）。
 * ========================================================================== */

#include <stdint.h>

/* 初始化电机 PWM（上电即滑行，不动作） */
void Motor_Init(void);

/**
 * @brief 设置左右轮**目标速度**（内部闭环后输出占空比）。
 * @param left_per_mille  左轮目标速度，千分比，正 = 后退、负 = 前进。
 * @param right_per_mille 右轮目标速度，同上。
 *
 * @note 每个控制周期调用一次。闭环每隔 CHASSIS_SPEED_LOOP_DIV 拍算一次，
 *       斜率限制按「每次调用」计算，所以调用周期必须稳定等于
 *       CHASSIS_CONTROL_PERIOD_MS。
 */
void Motor_SetSpeed(int16_t left_per_mille, int16_t right_per_mille);

/** @brief 立即滑行：清零目标与输出、清 PI 积分，H 桥关断（不等斜率）。安全路径专用。 */
void Motor_Stop(void);

/** @return 左轮当前**实际输出**的千分比（不是目标值）。 */
int16_t Motor_GetLeftOutput(void);

/** @return 右轮当前**实际输出**的千分比（不是目标值）。 */
int16_t Motor_GetRightOutput(void);

/** @return 左轮**实测速度**（‰，正 = 后退），来自编码器。 */
int16_t Motor_GetLeftSpeed(void);

/** @return 右轮**实测速度**（‰，正 = 后退），来自编码器。 */
int16_t Motor_GetRightSpeed(void);

#endif /* MOTOR_DRIVER_H */
