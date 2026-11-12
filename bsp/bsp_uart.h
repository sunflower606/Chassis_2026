#ifndef BSP_UART_H
#define BSP_UART_H

/* ============================================================================
 * BSP：主机/调试串口（USART1，中断接收 + 环形缓冲）
 *
 * 本板用 USART1 做两件事：
 *   ① 与云台板交换人体偏差数据（模式 2「纯人体跟随」需要）
 *   ② 调试打印
 *
 * 层次约定：本文件是唯一允许操作 USART1 的地方。
 *          modules/vision_link.c 只通过下面这组接口取字节。
 * ========================================================================== */

#include <stdint.h>

/* 初始化 USART1：PA9 = TX，PA10 = RX，115200-8-N-1，开启 RXNE 中断 */
void BSP_UART_Init(void);

/** @brief 阻塞发送一个字节（等 TXE 标志）。 */
void BSP_UART_SendByte(uint8_t byte);

/** @brief 阻塞发送以 '\0' 结尾的字符串。 */
void BSP_UART_SendString(const char *text);

/** @return 接收缓冲里未读字节数。 */
uint16_t BSP_UART_GetRxCount(void);

/**
 * @brief 查看队首第 offset 个字节，不出队。
 * @return 1 = 取到；0 = 没有这么多字节。
 */
uint8_t BSP_UART_PeekAt(uint16_t offset, uint8_t *byte);

/** @brief 从队首丢弃 count 个字节（超过现有数量按实际丢）。 */
void BSP_UART_Skip(uint16_t count);

/** @return 一个字节并出队；空时返回 0xFF。 */
uint8_t BSP_UART_ReadByte(void);

#endif /* BSP_UART_H */
