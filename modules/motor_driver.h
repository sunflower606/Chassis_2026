#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/* ============================================================================
 * Module：差速底盘电机驱动
 *
 * 硬件：后双轮差速驱动（后轴两个 MG513 霍尔编码器电机 + H 桥）
 *
 * 本模块负责三件事：
 *   1) 目标值限幅（CHASSIS_MOTOR_DUTY_MIN / MAX）；
 *   2) 斜率限制（每拍最多变化 CHASSIS_MOTOR_RAMP_PER_TICK），
 *      避免突然满占空比造成电流冲击、以及停车点头；
 *   3) 调用 BSP 输出到 TIM3 + H 桥方向脚。
 *
 * 编码器目前不参与闭环，只做诊断（见 bsp/bsp_encoder.h）。
 * 要做速度闭环时，在这里加一层「目标速度 -> 实测速度」的 PID 即可。
 * ========================================================================== */

#include <stdint.h>

/* 初始化电机 PWM（上电即滑行，不动作） */
void Motor_Init(void);

/**
 * @brief 设置左右轮目标速度（内部会斜率限制后立即输出）。
 * @param left_per_mille  左轮，千分比，-1000 ~ +1000。
 * @param right_per_mille 右轮，千分比，-1000 ~ +1000。
 *
 * @note 每个控制周期调用一次。斜率限制按「每次调用」计算，
 *       所以调用周期必须稳定等于 CHASSIS_CONTROL_PERIOD_MS。
 */
void Motor_SetSpeed(int16_t left_per_mille, int16_t right_per_mille);

/** @brief 立即滑行：清零目标与输出，H 桥关断（不等斜率）。安全路径专用。 */
void Motor_Stop(void);

/** @return 左轮当前**实际输出**的千分比（不是目标值）。 */
int16_t Motor_GetLeftOutput(void);

/** @return 右轮当前**实际输出**的千分比（不是目标值）。 */
int16_t Motor_GetRightOutput(void);

#endif /* MOTOR_DRIVER_H */
