#include "bsp_ultrasonic.h"

#include "board_config.h"
#include "Delay.h"

/* ============================================================================
 * 实现说明
 *
 * 这套接法下 Echo 接在 PB11，而本板**没有空闲的定时器捕获通道**
 * （TIM2/TIM4 归编码器、TIM3 出 PWM、TIM1 的捕获脚 PA8~PA11 被循迹占用），
 * 所以测量方式改成：
 *   1) TIM1 配成 1 MHz 自由运行时基（不需要任何引脚），
 *      每 65536 us 溢出一次，溢出中断里累加高位；
 *   2) EXTI 线 11 配成**双边沿中断**：
 *      上升沿 -> 记下回波起点，把触发条件切成下降沿；
 *      下降沿 -> 算出脉宽，把触发条件切回上升沿。
 *
 * 为什么不用 GPIO 轮询：控制环 5 ms 一轮，1 ms 误差折合 17 cm。
 *
 * 为什么 EXTI 用寄存器写而不是 SPL：
 *   本工程 Drivers 里没有 stm32f10x_exti.c（只有 .h），
 *   而 BSP 层本来就是唯一允许直接碰寄存器的层，所以直接写 AFIO/EXTI。
 * ========================================================================== */

/* EXTI 线掩码 */
#define ULTRASONIC_EXTI_MASK    (1UL << BOARD_ULTRASONIC_ECHO_EXTI_LINE)

/* 捕获状态机 */
static volatile uint8_t  s_wait_rise = 1U;      /* 1 = 等上升沿，0 = 等下降沿 */
static volatile uint32_t s_rise_us = 0U;        /* 回波起点时刻，us */
static volatile uint16_t s_width_us = 0U;       /* 算好的脉宽，us */
static volatile uint8_t  s_have_echo = 0U;      /* 1 = 有新脉宽可取 */

/* TIM1 自由运行时基的高位（每 65536 us 加一次） */
static volatile uint32_t s_us_high = 0U;

/* ------------------------------------------------------------------------- */
/* 读当前时刻（us，32 位）
 *
 * 16 位计数器 + 高位组合，必须避免「读高位 -> 计数器溢出 -> 读计数器」
 * 这种撕裂。做法是读两遍高位，不一致就重读。 */
static uint32_t BSP_Ultrasonic_GetTimeUs(void)
{
    uint32_t high_first;
    uint32_t high_second;
    uint32_t counter;

    do {
        high_first = s_us_high;
        counter = (uint32_t)TIM_GetCounter(BOARD_ULTRASONIC_TIM);
        high_second = s_us_high;
    } while (high_first != high_second);

    return high_second + counter;
}

/* 切换 EXTI 触发边沿：rising != 0 用上升沿，否则下降沿 */
static void BSP_Ultrasonic_SetEdge(uint8_t rising)
{
    if (rising != 0U) {
        EXTI->RTSR |= ULTRASONIC_EXTI_MASK;
        EXTI->FTSR &= ~ULTRASONIC_EXTI_MASK;
    } else {
        EXTI->RTSR &= ~ULTRASONIC_EXTI_MASK;
        EXTI->FTSR |= ULTRASONIC_EXTI_MASK;
    }
}

/* ------------------------------------------------------------------------- */
void BSP_Ultrasonic_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    TIM_TimeBaseInitTypeDef time_base;
    NVIC_InitTypeDef nvic_init;
    uint32_t exticr_index;
    uint32_t exticr_shift;

    RCC_APB2PeriphClockCmd(BOARD_ULTRASONIC_TRIG_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_ULTRASONIC_ECHO_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_ULTRASONIC_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    /* EXTI 挂靠需要 */

    /* ---- Trig：推挽输出，初始低 ---- */
    gpio_init.GPIO_Pin = BOARD_ULTRASONIC_TRIG_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BOARD_ULTRASONIC_TRIG_GPIO, &gpio_init);
    GPIO_ResetBits(BOARD_ULTRASONIC_TRIG_GPIO, BOARD_ULTRASONIC_TRIG_PIN);

    /* ---- Echo：浮空输入（模块推挽输出主动驱动） ---- */
    gpio_init.GPIO_Pin = BOARD_ULTRASONIC_ECHO_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BOARD_ULTRASONIC_ECHO_GPIO, &gpio_init);

    /* ---- TIM1：1 MHz 自由运行，只做时基 ---- */
    TIM_TimeBaseStructInit(&time_base);
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Prescaler = BOARD_ULTRASONIC_TIM_PRESCALER;
    time_base.TIM_Period = BOARD_ULTRASONIC_TIM_PERIOD;
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(BOARD_ULTRASONIC_TIM, &time_base);

    s_us_high = 0U;
    TIM_SetCounter(BOARD_ULTRASONIC_TIM, 0U);
    TIM_ClearITPendingBit(BOARD_ULTRASONIC_TIM, TIM_IT_Update);
    TIM_ITConfig(BOARD_ULTRASONIC_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(BOARD_ULTRASONIC_TIM, ENABLE);

    nvic_init.NVIC_IRQChannel = BOARD_ULTRASONIC_TIM_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic_init.NVIC_IRQChannelSubPriority = 1U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    /* ---- EXTI：把 11 号线挂到 GPIOB（PB11），双边沿（先上升） ---- */
    exticr_index = BOARD_ULTRASONIC_ECHO_EXTI_LINE / 4U;
    exticr_shift = (BOARD_ULTRASONIC_ECHO_EXTI_LINE % 4U) * 4U;

    AFIO->EXTICR[exticr_index] &= ~(0xFUL << exticr_shift);
    AFIO->EXTICR[exticr_index] |=
        ((uint32_t)BOARD_ULTRASONIC_ECHO_EXTI_PORT_SRC << exticr_shift);

    EXTI->IMR |= ULTRASONIC_EXTI_MASK;      /* 解除屏蔽 */
    s_wait_rise = 1U;
    s_have_echo = 0U;
    s_width_us = 0U;
    BSP_Ultrasonic_SetEdge(1U);

    EXTI->PR = ULTRASONIC_EXTI_MASK;        /* 写 1 清挂起 */

    nvic_init.NVIC_IRQChannel = BOARD_ULTRASONIC_ECHO_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic_init.NVIC_IRQChannelSubPriority = 2U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

void BSP_Ultrasonic_Trigger(void)
{
    /* 上一次结果作废，避免上层拿到过期脉宽 */
    s_have_echo = 0U;

    /* 上一次没捕到下降沿时先把状态复位，否则这次接不上 */
    if (s_wait_rise == 0U) {
        BSP_Ultrasonic_ResetEcho();
    }

    GPIO_SetBits(BOARD_ULTRASONIC_TRIG_GPIO, BOARD_ULTRASONIC_TRIG_PIN);
    Delay_us(BOARD_ULTRASONIC_TRIG_PULSE_US);
    GPIO_ResetBits(BOARD_ULTRASONIC_TRIG_GPIO, BOARD_ULTRASONIC_TRIG_PIN);
}

uint8_t BSP_Ultrasonic_TakeEchoUs(uint16_t *echo_us)
{
    if (s_have_echo == 0U) {
        return 0U;
    }

    if (echo_us != 0) {
        *echo_us = s_width_us;
    }

    s_have_echo = 0U;

    return 1U;
}

void BSP_Ultrasonic_ResetEcho(void)
{
    s_wait_rise = 1U;
    s_have_echo = 0U;
    BSP_Ultrasonic_SetEdge(1U);
    EXTI->PR = ULTRASONIC_EXTI_MASK;
}

/* ------------------------------------------------------------ 中断服务 ---- */

/* TIM1 溢出：每 65536 us 一次，累加时基高位 */
void BOARD_ULTRASONIC_TIM_IRQHandler(void)
{
    if (TIM_GetITStatus(BOARD_ULTRASONIC_TIM, TIM_IT_Update) != RESET) {
        s_us_high += 65536UL;
        TIM_ClearITPendingBit(BOARD_ULTRASONIC_TIM, TIM_IT_Update);
    }
}

/* EXTI 线 11：回波上升沿 / 下降沿 */
void BOARD_ULTRASONIC_ECHO_IRQHandler(void)
{
    if ((EXTI->PR & ULTRASONIC_EXTI_MASK) == 0UL) {
        return;
    }

    /* 写 1 清挂起（否则会反复进中断） */
    EXTI->PR = ULTRASONIC_EXTI_MASK;

    if (s_wait_rise != 0U) {
        /* 上升沿：回波开始 */
        s_rise_us = BSP_Ultrasonic_GetTimeUs();
        s_wait_rise = 0U;
        BSP_Ultrasonic_SetEdge(0U);
    } else {
        /* 下降沿：回波结束 */
        uint32_t now_us = BSP_Ultrasonic_GetTimeUs();
        uint32_t width_us = now_us - s_rise_us;

        /* 超过 65535 us 说明是异常长脉冲（或时基撕裂），钳住即可，
         * 上层量程判定会把它当无效值丢掉。 */
        s_width_us = (width_us > 0xFFFFUL) ? 0xFFFFU : (uint16_t)width_us;
        s_wait_rise = 1U;
        s_have_echo = 1U;
        BSP_Ultrasonic_SetEdge(1U);
    }
}
