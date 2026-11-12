#include "bsp_line_gpio.h"

/* ---------------------------------------------------------------- 引脚表
 * 数组下标就是位图里的位号：索引 0 -> bit0 -> CH1（最左）。
 * 顺序改错会导致「明明线在左边，程序以为在右边」，改这里时务必对着实物从左到右数。
 *
 * 允许跨端口：现在 CH1~CH6 在 GPIOA，CH7 在 GPIOB。
 * ---------------------------------------------------------------- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} LinePin_t;

static const LinePin_t s_line_pins[BSP_LINE_CHANNEL_COUNT] = {
    { BOARD_LINE_GPIO,   BOARD_LINE_CH1_PIN },   /* bit0，最左 */
    { BOARD_LINE_GPIO,   BOARD_LINE_CH2_PIN },   /* bit1 */
    { BOARD_LINE_GPIO,   BOARD_LINE_CH3_PIN },   /* bit2 */
    { BOARD_LINE_GPIO,   BOARD_LINE_CH4_PIN },   /* bit3，正中 */
    { BOARD_LINE_GPIO,   BOARD_LINE_CH5_PIN },   /* bit4 */
    { BOARD_LINE_GPIO,   BOARD_LINE_CH6_PIN },   /* bit5 */
    { BOARD_LINE_B_GPIO, BOARD_LINE_B_CH7_PIN },  /* bit6，最右 */
};

/* 位图用一个字节装，通道数超过 8 就装不下了 —— 超过时这里直接编译报错 */
typedef char line_pin_bit_width_ok[(BSP_LINE_CHANNEL_COUNT <= 8U) ? 1 : -1];

void BSP_Line_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    /* 两个端口组的时钟一起开，OR 在一起不会有副作用 */
    RCC_APB2PeriphClockCmd(BOARD_LINE_GPIO_CLK | BOARD_LINE_B_GPIO_CLK, ENABLE);

    /* 上拉输入：传感器未驱动时保持高电平，不会浮空误判 */
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    /* CH1~CH6：GPIOA（CH6 = PA15，JTAG 脚，已在 bsp_encoder.c 里关掉 JTAG） */
    gpio_init.GPIO_Pin = BOARD_LINE_CH1_PIN | BOARD_LINE_CH2_PIN |
                         BOARD_LINE_CH3_PIN | BOARD_LINE_CH4_PIN |
                         BOARD_LINE_CH5_PIN | BOARD_LINE_CH6_PIN;
    GPIO_Init(BOARD_LINE_GPIO, &gpio_init);

    /* CH7：GPIOB（PB3，JTAG 脚，同样靠关 JTAG 释放） */
    gpio_init.GPIO_Pin = BOARD_LINE_B_CH7_PIN;
    GPIO_Init(BOARD_LINE_B_GPIO, &gpio_init);
}

uint8_t BSP_Line_ReadRaw(void)
{
    uint8_t raw = 0U;
    uint8_t i;

    /* 逐个通道读，位号由表的下标决定，写错位的机会比手写 7 个 if 小 */
    for (i = 0U; i < BSP_LINE_CHANNEL_COUNT; i++) {
        if (GPIO_ReadInputDataBit(s_line_pins[i].port, s_line_pins[i].pin) != Bit_RESET) {
            raw |= (uint8_t)(1U << i);
        }
    }

    return raw;
}
