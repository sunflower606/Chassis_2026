#include "bsp_flash.h"

#include "stm32f10x.h"

/* ---- FLASH 寄存器位（不引 stm32f10x_flash.h，避免多带一个驱动） ---- */
#define BF_CR_PG          0x00000001UL
#define BF_CR_PER         0x00000002UL
#define BF_CR_STRT        0x00000040UL
#define BF_CR_LOCK        0x00000080UL

#define BF_SR_BSY         0x00000001UL
#define BF_SR_PGERR       0x00000004UL
#define BF_SR_WRPRTERR    0x00000010UL
#define BF_SR_EOP         0x00000020UL

#define BF_KEY1           0x45670123UL
#define BF_KEY2           0xCDEF89ABUL

/* 一轮等待的上限。72 MHz 下一半字编程约 40 us、页擦除约 20 ms，
 * 这里给 0x000FFFFF 次空转（远大于需求），只是防止死等。 */
#define BF_WAIT_LIMIT     0x000FFFFFUL

static uint8_t Flash_WaitNotBusy(void)
{
    uint32_t guard = BF_WAIT_LIMIT;

    while ((FLASH->SR & BF_SR_BSY) != 0UL) {
        if (guard == 0UL) {
            return 0U;
        }
        guard--;
    }

    /* PGERR / WRPRTERR 任一置位都算失败 */
    if ((FLASH->SR & (BF_SR_PGERR | BF_SR_WRPRTERR)) != 0UL) {
        return 0U;
    }

    return 1U;
}

static void Flash_Unlock(void)
{
    if ((FLASH->CR & BF_CR_LOCK) != 0UL) {
        FLASH->KEYR = BF_KEY1;
        FLASH->KEYR = BF_KEY2;
    }
}

static void Flash_Lock(void)
{
    FLASH->CR |= BF_CR_LOCK;
}

static void Flash_ClearFlags(void)
{
    FLASH->SR = BF_SR_EOP | BF_SR_PGERR | BF_SR_WRPRTERR;
}

uint8_t BSP_Flash_ErasePage(uint32_t page_addr)
{
    uint8_t ok;

    Flash_Unlock();
    Flash_ClearFlags();

    FLASH->CR |= BF_CR_PER;
    FLASH->AR = page_addr;
    FLASH->CR |= BF_CR_STRT;

    ok = Flash_WaitNotBusy();

    FLASH->CR &= ~BF_CR_PER;
    Flash_ClearFlags();
    Flash_Lock();

    return ok;
}

uint8_t BSP_Flash_ProgramHalfWords(uint32_t addr,
                                   const uint16_t *half_words,
                                   uint16_t count)
{
    uint16_t i;

    if ((half_words == 0) || (count == 0U)) {
        return 0U;
    }

    Flash_Unlock();
    Flash_ClearFlags();

    for (i = 0U; i < count; i++) {
        /* PG 只置一次也可以，但每写半字重设更不容易出错 */
        FLASH->CR |= BF_CR_PG;
        *(volatile uint16_t *)(addr + ((uint32_t)i * 2UL)) = half_words[i];

        if (Flash_WaitNotBusy() == 0U) {
            FLASH->CR &= ~BF_CR_PG;
            Flash_ClearFlags();
            Flash_Lock();
            return 0U;
        }

        FLASH->CR &= ~BF_CR_PG;
    }

    Flash_ClearFlags();
    Flash_Lock();

    return 1U;
}

uint16_t BSP_Flash_ReadHalfWord(uint32_t addr)
{
    return *(volatile const uint16_t *)addr;
}
