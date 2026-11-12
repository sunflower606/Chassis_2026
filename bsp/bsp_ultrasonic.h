#ifndef BSP_ULTRASONIC_H
#define BSP_ULTRASONIC_H

/* ============================================================================
 * BSP：HC-SR04 超声波测距（Trig 输出 + Echo EXTI 双边沿捕获）
 *
 * 引脚：Trig = PB10（推挽输出），Echo = PB11（EXTI 线 11，双边沿中断）
 *
 * 为什么不用 GPIO 轮询：
 *   控制环 5 ms 一轮，用轮询计时分辨率就只有 5 ms；
 *   声速下 1 ms 折合约 17 cm，误差完全没法做距离闭环。
 *   Echo 走 EXTI 中断 + TIM1 1 MHz 自由运行时基，1 个计数 = 1 us
 *   = 0.017 cm，数量级完全不同。
 *
 * 测量是非阻塞的：
 *   1) 上层调 BSP_Ultrasonic_Trigger() 发 10 us 触发脉冲（阻塞仅 10 us）；
 *   2) Echo 上升沿 -> 中断记下时刻，同时把捕获极性翻成下降沿；
 *   3) Echo 下降沿 -> 中断算出脉宽，置「有新数据」标志；
 *   4) 上层用 TakeEchoUs() 取走脉宽。
 *
 * 上层（modules/distance_sensor.c）负责换算成 cm、超时与量程判定。
 * ========================================================================== */

#include <stdint.h>

/** @brief 初始化 Trig 输出、Echo 输入、TIM1 输入捕获与中断。 */
void BSP_Ultrasonic_Init(void);

/**
 * @brief 发一次触发脉冲（高电平 BOARD_ULTRASONIC_TRIG_PULSE_US 微秒）。
 * @note 会清掉上一次未取走的回波结果。阻塞约 10 us。
 */
void BSP_Ultrasonic_Trigger(void);

/**
 * @brief 取一次已完成的回波宽度。
 * @param echo_us 取到数据时写入脉宽，单位 us。
 * @return 1 = 取到了新数据；0 = 还没有结果。
 *
 * @note 取走后标志清零，所以同一次回波只会被返回一次。
 */
uint8_t BSP_Ultrasonic_TakeEchoUs(uint16_t *echo_us);

/**
 * @brief 放弃本次测量，把捕获状态复位到「等上升沿」。
 *
 * 用于超时：如果只捕到上升沿、没有回波下降沿（无障碍物 / 超出量程），
 * 捕获极性会停在下降沿，下一次测量就再也接不上 —— 必须复位。
 */
void BSP_Ultrasonic_ResetEcho(void);

#endif /* BSP_ULTRASONIC_H */
