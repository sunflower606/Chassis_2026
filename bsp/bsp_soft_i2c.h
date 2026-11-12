#ifndef BSP_SOFT_I2C_H
#define BSP_SOFT_I2C_H

/* ============================================================================
 * BSP：软件 I2C 的电平操作（开漏输出）
 *
 * 只有本层碰 GPIO；modules/oled.c 用下面三个接口拼 I2C 时序，
 * 不知道引脚号、也不需要写寄存器。
 *
 * 为什么用开漏（Out_OD）：I2C 是多器件总线，必须靠「线与」工作；
 * 推挽输出会与从机驱动冲突，大电流可能损坏引脚。开漏下写 1 只是释放总线，
 * 实际高电平由模块板载上拉电阻提供。
 * ========================================================================== */

#include <stdint.h>

/* 初始化 SCL/SDA 为开漏输出，并释放总线（两线都置高） */
void BSP_SoftI2C_Init(void);

/** @brief 驱动 SCL 电平。level 非 0 为高（释放），0 为低（下拉）。 */
void BSP_SoftI2C_WriteScl(uint8_t level);

/** @brief 驱动 SDA 电平。level 非 0 为高（释放），0 为低（下拉）。 */
void BSP_SoftI2C_WriteSda(uint8_t level);

#endif /* BSP_SOFT_I2C_H */
