#ifndef OLED_H
#define OLED_H

/* ============================================================================
 * Module：0.96 寸 OLED（SSD1306，128x64，软件 I2C）
 *
 * 只提供字符/数字显示，不做图形。行 1~4、列 1~16（8x16 字模）。
 * 引脚与开漏配置在 bsp/bsp_soft_i2c.c，本模块不直接碰 GPIO。
 *
 * 注意：刷一次 4 行约 3~5 ms（软件 I2C），必须节流调用，
 *      不要放进 10 ms 控制环的每一拍里。
 * ========================================================================== */

#include <stdint.h>

/**
 * @brief 上电初始化：延时、配置引脚、下发 SSD1306 初始化序列、清屏。
 * @note 必须在任何其他 OLED_* 调用之前执行。
 */
void OLED_Init(void);

void OLED_Clear(void);

/**
 * @brief 在指定位置显示一个 ASCII 字符。
 * @param Line   行号，1 ~ 4。
 * @param Column 列号，1 ~ 16。
 * @param Char   字符；必须 >= 空格（0x20），否则字库索引越界。
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);

/** @brief 显示字符串。行/列范围同 OLED_ShowChar。 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

/**
 * @brief 显示无符号十进制数（固定宽度，前导补 0）。
 * @param Length 数字位数（不含符号位）。
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief 显示带符号十进制数。
 * @param Length 数字位数（**不含符号位**，实际占用 Length + 1 列）。
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

/** @brief 显示十六进制数（Length 为位数）。 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/** @brief 显示二进制数（Length 为位数）。 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief 屏是否在线（最近一次 I2C 寻址有没有收到应答）。
 * @retval 1 = 有应答；0 = 没供电 / 没接好 / 地址不对 / 总线卡死。
 */
uint8_t OLED_IsPresent(void);

#endif /* OLED_H */
