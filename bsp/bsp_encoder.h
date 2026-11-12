#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

/* ============================================================================
 * BSP：双轮霍尔编码器（TIM2 / TIM4 硬件编码器模式，4 倍频计数）
 *
 * 引脚：
 *   左轮 -> TIM2_CH1 = PA0（A 相），TIM2_CH2 = PA1（B 相）  ← 默认映射，不重映射
 *   右轮 -> TIM4_CH1 = PB6（A 相），TIM4_CH2 = PB7（B 相）
 *
 * ⚠️ PA0/PA1 是 TTa（不容忍 5V）：编码器已确认按 **3.3V 供电**，安全。
 *   以后换编码器时不要换成只能 5V 供电的型号；真要用 5V，需把循迹 CH6/CH7
 *   从 PA15/PB3 挪走、编码器改到 PA15/PB3（FT），并恢复 TIM2 部分重映射。
 *
 * 仍然必须关 JTAG（保留 SWD）：循迹 CH6/CH7（PA15/PB3）与按键 STOP（PB4）要用。
 *
 * 目前用途：**只做诊断**（OLED/串口看计数），尚未参与速度闭环。
 * 做速度闭环时用 BSP_Encoder_GetLeftDelta 在一个固定周期内累计即可。
 * ========================================================================== */

#include <stdint.h>

/**
 * @brief 初始化两路编码器。
 *
 * 内部会做两件容易忘但必须做的事：
 *   1) 打开 AFIO 时钟；
 *   2) 关闭 JTAG（保留 SWD），否则 PA15/PB3/PB4 不能当普通 IO。
 *
 * 注意：**不做** GPIO_PartialRemap1_TIM2 —— 左轮编码器就在 TIM2 默认的
 * PA0/PA1 上，一旦重映射，读数恒为 0 且不报错。
 */
void BSP_Encoder_Init(void);

/**
 * @brief 取左轮自上次调用以来的计数增量。
 * @return 有符号增量（正负取决于转动方向与 A/B 相接线顺序）。
 * @note 用 int16 相减处理计数回绕，两次调用间隔不能超过 32767 个计数。
 */
int16_t BSP_Encoder_GetLeftDelta(void);

/** @brief 取右轮自上次调用以来的计数增量（同上）。 */
int16_t BSP_Encoder_GetRightDelta(void);

/**
 * @brief 直接读左轮计数器原始值。
 * @note 用于标定「转一圈多少计数」：手转输出轴一整圈，看这个值的差值。
 */
int16_t BSP_Encoder_ReadLeftCount(void);

/** @brief 直接读右轮计数器原始值。 */
int16_t BSP_Encoder_ReadRightCount(void);

#endif /* BSP_ENCODER_H */
