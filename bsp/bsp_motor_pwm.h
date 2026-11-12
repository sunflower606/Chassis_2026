#ifndef BSP_MOTOR_PWM_H
#define BSP_MOTOR_PWM_H

/* ============================================================================
 * BSP：TB6612 双路直流电机 PWM + 方向控制（TIM3 双通道）
 *
 * 电气层负责「符号转方向脚、幅值转占空比」，上层只给带符号的千分比速度：
 *     +1000 = 全速正转，0 = 滑行，-1000 = 全速反转
 *
 * 本层限幅是最后一道保护：任何入参都会被夹到 [-1000, +1000]。
 *
 * TB6612 通道对应：A 通道 = 左轮（PWMA/AIN1/AIN2），B 通道 = 右轮
 * （PWMB/BIN1/BIN2）。STBY 在 Init 里拉高，否则整片待机、电机不动。
 *
 * 引脚见 config/pin_config.h（BOARD_TB6612_* 一段）。
 * ========================================================================== */

#include <stdint.h>

/* 初始化 TIM3 双通道 PWM 与 4 个方向脚，并让电机处于滑行状态 */
void BSP_Motor_Init(void);

/**
 * @brief 设置左电机速度。
 * @param speed_per_mille 千分比速度，-1000 ~ +1000；正为前进，负为后退。
 */
void BSP_Motor_SetLeft(int16_t speed_per_mille);

/**
 * @brief 设置右电机速度。
 * @param speed_per_mille 千分比速度，-1000 ~ +1000；正为前进，负为后退。
 */
void BSP_Motor_SetRight(int16_t speed_per_mille);

/** @brief 两路电机同时滑行（H 桥关断，占空比归零）。 */
void BSP_Motor_Stop(void);

#endif /* BSP_MOTOR_PWM_H */
