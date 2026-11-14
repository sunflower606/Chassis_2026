#include "bsp_soft_i2c.h"

#include "board_config.h"

void BSP_SoftI2C_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(BOARD_OLED_GPIO_CLK, ENABLE);

    gpio_init.GPIO_Pin = BOARD_OLED_SCL_PIN | BOARD_OLED_SDA_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_OD;   /* 开漏：I2C 的硬性要求 */
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;   /* I2C 只需要 100k~400k，50MHz 会在长线上振铃 */
    GPIO_Init(BOARD_OLED_GPIO, &gpio_init);

    /* 释放总线，等上拉把两条线拉到高电平 */
    BSP_SoftI2C_WriteScl(1U);
    BSP_SoftI2C_WriteSda(1U);
}

void BSP_SoftI2C_WriteScl(uint8_t level)
{
    GPIO_WriteBit(BOARD_OLED_GPIO, BOARD_OLED_SCL_PIN, (BitAction)(level ? Bit_SET : Bit_RESET));
}

void BSP_SoftI2C_WriteSda(uint8_t level)
{
    GPIO_WriteBit(BOARD_OLED_GPIO, BOARD_OLED_SDA_PIN, (BitAction)(level ? Bit_SET : Bit_RESET));
}

uint8_t BSP_SoftI2C_ReadSda(void)
{
    /* 开漏输出下写 1 = 释放，引脚电平由模块上拉决定，故读输入数据寄存器 */
    return (GPIO_ReadInputDataBit(BOARD_OLED_GPIO, BOARD_OLED_SDA_PIN) != Bit_RESET) ? 1U : 0U;
}
