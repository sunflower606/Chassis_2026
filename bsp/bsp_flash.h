#ifndef BSP_FLASH_H
#define BSP_FLASH_H

/* ============================================================================
 * BSP：片内 Flash 读写（只用于「参数存盘」这一件事）
 *
 * 为什么不直接引 StdPeriph 的 stm32f10x_flash.c：
 *   本工程 Drivers 目录只挑了用得到的 5 个外设 .c（misc/gpio/rcc/tim/usart），
 *   为了一个 40 字节的参数存储去导一整套 flash 驱动不划算，
 *   而擦除/编程的寄存器序列很短（解锁 → 擦/写 → 等 BSY → 上锁），
 *   自己写 60 行反而更容易看懂、更容易审。
 *
 * 硬件约束（STM32F103C8T6，64 KB Flash，页大小 1 KB）：
 *   - 只能整页擦除，所以参数区独占最后一页 0x0800FC00 ~ 0x0800FFFF；
 *   - 编程单位是**半字（16 bit）**，地址必须 2 字节对齐；
 *   - 擦写期间 CPU 从 Flash 取指会被暂停，所以不要在控制环里调用。
 *
 * 安全：擦写结束一定重新上锁；任何一步超时都返回 0，调用方据此报错，
 *       绝不"以为写进去了"。
 * ========================================================================== */

#include <stdint.h>

/** @brief 参数区所在页的起始地址（64 KB 型号的最后一页）。 */
#define BSP_FLASH_PARAM_PAGE_ADDR   0x0800FC00UL

/**
 * @brief 擦除整页。
 * @param page_addr 页起始地址，必须是 1 KB 边界
 * @return 1 = 成功，0 = 失败（超时或写保护）
 */
uint8_t BSP_Flash_ErasePage(uint32_t page_addr);

/**
 * @brief 连续编程若干个半字（从 addr 开始，addr 必须 2 字节对齐）。
 * @param addr   起始地址
 * @param half_words 半字缓冲区
 * @param count  半字个数
 * @return 1 = 成功，0 = 失败
 */
uint8_t BSP_Flash_ProgramHalfWords(uint32_t addr,
                                   const uint16_t *half_words,
                                   uint16_t count);

/**
 * @brief 读一个半字（Flash 可以像内存一样直接读，这里只是给调用方一个统一入口）。
 */
uint16_t BSP_Flash_ReadHalfWord(uint32_t addr);

#endif /* BSP_FLASH_H */
