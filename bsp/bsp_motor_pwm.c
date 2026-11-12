#include "bsp_motor_pwm.h"

#include "board_config.h"

/* 把带符号速度夹到 [-DUTY_MAX, +DUTY_MAX] */
static int16_t BSP_Motor_Clamp(int16_t speed_per_mille)
{
    if (speed_per_mille > (int16_t)BOARD_MOTOR_DUTY_MAX) {
        return (int16_t)BOARD_MOTOR_DUTY_MAX;
    }
    if (speed_per_mille < -(int16_t)BOARD_MOTOR_DUTY_MAX) {
        return -(int16_t)BOARD_MOTOR_DUTY_MAX;
    }

    return speed_per_mille;
}

/* 方向脚 / STBY：普通推挽输出 */
static void BSP_Motor_OutPinInit(GPIO_TypeDef *gpio, uint16_t pin)
{
    GPIO_InitTypeDef init;

    init.GPIO_Pin = pin;
    init.GPIO_Mode = GPIO_Mode_Out_PP;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &init);
}

/* PWM 脚：复用推挽输出 */
static void BSP_Motor_PwmPinInit(GPIO_TypeDef *gpio, uint16_t pin)
{
    GPIO_InitTypeDef init;

    init.GPIO_Pin = pin;
    init.GPIO_Mode = GPIO_Mode_AF_PP;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &init);
}

void BSP_Motor_Init(void)
{
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef oc_init;

    RCC_APB1PeriphClockCmd(BOARD_MOTOR_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_TB6612_GPIOA_CLK | BOARD_TB6612_GPIOB_CLK, ENABLE);

    /* 两个调速脚 */
    BSP_Motor_PwmPinInit(BOARD_TB6612_PWMA_GPIO, BOARD_TB6612_PWMA_PIN);
    BSP_Motor_PwmPinInit(BOARD_TB6612_PWMB_GPIO, BOARD_TB6612_PWMB_PIN);

    /* 四个方向脚 + STBY */
    BSP_Motor_OutPinInit(BOARD_TB6612_AIN1_GPIO, BOARD_TB6612_AIN1_PIN);
    BSP_Motor_OutPinInit(BOARD_TB6612_AIN2_GPIO, BOARD_TB6612_AIN2_PIN);
    BSP_Motor_OutPinInit(BOARD_TB6612_BIN1_GPIO, BOARD_TB6612_BIN1_PIN);
    BSP_Motor_OutPinInit(BOARD_TB6612_BIN2_GPIO, BOARD_TB6612_BIN2_PIN);
    BSP_Motor_OutPinInit(BOARD_TB6612_STBY_GPIO, BOARD_TB6612_STBY_PIN);

    /* STBY 拉高：不拉高整片待机，PWM 给多少都不转（最容易漏的一根线） */
    GPIO_WriteBit(BOARD_TB6612_STBY_GPIO, BOARD_TB6612_STBY_PIN, Bit_SET);

    TIM_InternalClockConfig(BOARD_MOTOR_TIM);

    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Prescaler = BOARD_MOTOR_TIM_PRESCALER;
    time_base.TIM_Period = BOARD_MOTOR_TIM_PERIOD;
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(BOARD_MOTOR_TIM, &time_base);

    TIM_OCStructInit(&oc_init);
    oc_init.TIM_OCMode = TIM_OCMode_PWM1;
    oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
    oc_init.TIM_OutputState = TIM_OutputState_Enable;
    oc_init.TIM_Pulse = 0U;

    /* CH1 -> PWMA，CH2 -> PWMB */
    TIM_OC1Init(BOARD_MOTOR_TIM, &oc_init);
    TIM_OC2Init(BOARD_MOTOR_TIM, &oc_init);

    TIM_OC1PreloadConfig(BOARD_MOTOR_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(BOARD_MOTOR_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BOARD_MOTOR_TIM, ENABLE);

    TIM_Cmd(BOARD_MOTOR_TIM, ENABLE);

    /* 上电先滑行，避免上电瞬间车子窜出去 */
    BSP_Motor_Stop();
}

void BSP_Motor_SetLeft(int16_t speed_per_mille)
{
    int16_t speed = BSP_Motor_Clamp(speed_per_mille);
    uint16_t duty;

    if (speed > 0) {
        /* A 通道正转：AIN1 高、AIN2 低 */
        GPIO_WriteBit(BOARD_TB6612_AIN1_GPIO, BOARD_TB6612_AIN1_PIN, Bit_SET);
        GPIO_WriteBit(BOARD_TB6612_AIN2_GPIO, BOARD_TB6612_AIN2_PIN, Bit_RESET);
        duty = (uint16_t)speed;
    } else if (speed < 0) {
        /* A 通道反转：AIN1 低、AIN2 高 */
        GPIO_WriteBit(BOARD_TB6612_AIN1_GPIO, BOARD_TB6612_AIN1_PIN, Bit_RESET);
        GPIO_WriteBit(BOARD_TB6612_AIN2_GPIO, BOARD_TB6612_AIN2_PIN, Bit_SET);
        duty = (uint16_t)(-speed);
    } else {
        /* 滑行：两脚同低，H 桥关断 */
        GPIO_WriteBit(BOARD_TB6612_AIN1_GPIO, BOARD_TB6612_AIN1_PIN, Bit_RESET);
        GPIO_WriteBit(BOARD_TB6612_AIN2_GPIO, BOARD_TB6612_AIN2_PIN, Bit_RESET);
        duty = 0U;
    }

    TIM_SetCompare1(BOARD_MOTOR_TIM, duty);
}

void BSP_Motor_SetRight(int16_t speed_per_mille)
{
    int16_t speed = BSP_Motor_Clamp(speed_per_mille);
    uint16_t duty;

    if (speed > 0) {
        /* B 通道正转：BIN1 高、BIN2 低 */
        GPIO_WriteBit(BOARD_TB6612_BIN1_GPIO, BOARD_TB6612_BIN1_PIN, Bit_SET);
        GPIO_WriteBit(BOARD_TB6612_BIN2_GPIO, BOARD_TB6612_BIN2_PIN, Bit_RESET);
        duty = (uint16_t)speed;
    } else if (speed < 0) {
        GPIO_WriteBit(BOARD_TB6612_BIN1_GPIO, BOARD_TB6612_BIN1_PIN, Bit_RESET);
        GPIO_WriteBit(BOARD_TB6612_BIN2_GPIO, BOARD_TB6612_BIN2_PIN, Bit_SET);
        duty = (uint16_t)(-speed);
    } else {
        GPIO_WriteBit(BOARD_TB6612_BIN1_GPIO, BOARD_TB6612_BIN1_PIN, Bit_RESET);
        GPIO_WriteBit(BOARD_TB6612_BIN2_GPIO, BOARD_TB6612_BIN2_PIN, Bit_RESET);
        duty = 0U;
    }

    TIM_SetCompare2(BOARD_MOTOR_TIM, duty);
}

void BSP_Motor_Stop(void)
{
    /* 四脚同低 = 滑行；STBY 保持高，随时可以再起步 */
    GPIO_WriteBit(BOARD_TB6612_AIN1_GPIO, BOARD_TB6612_AIN1_PIN, Bit_RESET);
    GPIO_WriteBit(BOARD_TB6612_AIN2_GPIO, BOARD_TB6612_AIN2_PIN, Bit_RESET);
    GPIO_WriteBit(BOARD_TB6612_BIN1_GPIO, BOARD_TB6612_BIN1_PIN, Bit_RESET);
    GPIO_WriteBit(BOARD_TB6612_BIN2_GPIO, BOARD_TB6612_BIN2_PIN, Bit_RESET);

    TIM_SetCompare1(BOARD_MOTOR_TIM, 0U);
    TIM_SetCompare2(BOARD_MOTOR_TIM, 0U);
}
