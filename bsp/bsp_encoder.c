#include "bsp_encoder.h"

#include "board_config.h"

/* 上一次读到的计数器值，用于算增量 */
static int16_t s_last_left = 0;
static int16_t s_last_right = 0;

/* 配置一个定时器为编码器模式。
 * 编码器模式下不需要手动配置输入捕获，TIM_EncoderInterfaceConfig 会
 * 把两个通道都设成输入并自动处理方向（正转加、反转减）。 */
static void BSP_Encoder_ConfigTimer(TIM_TypeDef *tim, uint16_t prescaler)
{
    TIM_TimeBaseInitTypeDef time_base;

    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Prescaler = prescaler;
    time_base.TIM_Period = 0xFFFFU;     /* 16 位满量程，减少回绕次数 */
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(tim, &time_base);

    /* TI12：A/B 两相都参与计数（4 倍频）；上下沿都计，得到最高分辨率 */
    TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_SetCounter(tim, 0U);
    TIM_Cmd(tim, ENABLE);
}

void BSP_Encoder_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    /* AFIO 必须先开：关闭 JTAG 靠它 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* 关闭 JTAG、保留 SWD：释放 PA15 / PB3 / PB4 当普通 IO。
     * 不关的话 PA15/PB3 被 JTAG 占用，循迹 CH6/CH7 与按键 STOP 读不到电平
     * —— 而且不报错。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 注意：**不要**做 GPIO_PartialRemap1_TIM2。
     * 左轮编码器接在 PA0/PA1，正是 TIM2_CH1/CH2 的默认映射；
     * 一旦启用部分重映射，通道会被挪到 PA15/PB3，PA0/PA1 上的编码器就读不到了。 */

    RCC_APB1PeriphClockCmd(BOARD_ENC_LEFT_TIM_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(BOARD_ENC_RIGHT_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_ENC_LEFT_A_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_ENC_LEFT_B_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_ENC_RIGHT_GPIO_CLK, ENABLE);

    /* 编码器输出是推挽/开集电平信号，MCU 侧浮空输入即可 */
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;

    gpio_init.GPIO_Pin = BOARD_ENC_LEFT_A_PIN;
    GPIO_Init(BOARD_ENC_LEFT_A_GPIO, &gpio_init);

    gpio_init.GPIO_Pin = BOARD_ENC_LEFT_B_PIN;
    GPIO_Init(BOARD_ENC_LEFT_B_GPIO, &gpio_init);

    gpio_init.GPIO_Pin = BOARD_ENC_RIGHT_A_PIN | BOARD_ENC_RIGHT_B_PIN;
    GPIO_Init(BOARD_ENC_RIGHT_GPIO, &gpio_init);

    /* 预分频 0：每个边沿都计，分辨率最高 */
    BSP_Encoder_ConfigTimer(BOARD_ENC_LEFT_TIM, 0U);
    BSP_Encoder_ConfigTimer(BOARD_ENC_RIGHT_TIM, 0U);

    s_last_left = 0;
    s_last_right = 0;
}

int16_t BSP_Encoder_ReadLeftCount(void)
{
    return (int16_t)(TIM_GetCounter(BOARD_ENC_LEFT_TIM) & 0xFFFFU);
}

int16_t BSP_Encoder_ReadRightCount(void)
{
    return (int16_t)(TIM_GetCounter(BOARD_ENC_RIGHT_TIM) & 0xFFFFU);
}

int16_t BSP_Encoder_GetLeftDelta(void)
{
    int16_t now = BSP_Encoder_ReadLeftCount();
    /* int16 相减天然处理 16 位回绕 */
    int16_t delta = (int16_t)(now - s_last_left);

    s_last_left = now;

    return delta;
}

int16_t BSP_Encoder_GetRightDelta(void)
{
    int16_t now = BSP_Encoder_ReadRightCount();
    int16_t delta = (int16_t)(now - s_last_right);

    s_last_right = now;

    return delta;
}
