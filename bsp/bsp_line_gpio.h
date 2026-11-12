#ifndef BSP_LINE_GPIO_H
#define BSP_LINE_GPIO_H

/* ============================================================================
 * BSP：红外循迹传感器输入（**7 路**，GPIO 上拉输入）
 *
 * 本层只做「读电平」，不判断黑白、不判线。
 * 极性转换与判线逻辑在 modules/line_sensor.c（那边才知道
 * 传感器是高有效还是低有效）。
 *
 * 7 路允许跨端口：当前 CH1~CH5 在 GPIOA，CH6/CH7 在 GPIOB，
 * 由 .c 里的引脚表统一处理，上层拿到的仍是连续位图。
 * ========================================================================== */

#include <stdint.h>

#include "board_config.h"

/** @brief 循迹通道数，等于 board_config.h 的 BOARD_LINE_CHANNEL_COUNT（当前 7）。 */
#define BSP_LINE_CHANNEL_COUNT      BOARD_LINE_CHANNEL_COUNT

/* 初始化全部通道为带上拉输入（各占用到的端口都会开时钟） */
void BSP_Line_Init(void);

/**
 * @brief 读全部通道电平的原始快照。
 * @return 位图：bit0 = CH1（最左）… bit6 = CH7（最右）；
 *         对应位为 1 表示该通道引脚为**高电平**。
 *
 * @note 一次调用把 7 路都读一遍，返回的是同一时刻的近似快照，可直接用于判线。
 */
uint8_t BSP_Line_ReadRaw(void);

#endif /* BSP_LINE_GPIO_H */
