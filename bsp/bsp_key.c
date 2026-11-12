#include "bsp_key.h"

#include "board_config.h"

/* 每个键一行：端口 + 引脚。下标就是 BspKeyId_t 的取值 */
typedef struct {
    GPIO_TypeDef *gpio;
    uint16_t pin;
} BspKeyPin_t;

static const BspKeyPin_t s_keys[BSP_KEY_COUNT] = {
    { BOARD_KEY_MODE_GPIO,  BOARD_KEY_MODE_PIN  },
    { BOARD_KEY_START_GPIO, BOARD_KEY_START_PIN },
    { BOARD_KEY_STOP_GPIO,  BOARD_KEY_STOP_PIN  },
};

/* 编译期检查：数组长度必须与 BSP_KEY_COUNT 一致（多写或少写会报错） */
typedef char bsp_key_count_ok[
    (sizeof(s_keys) / sizeof(s_keys[0]) == BSP_KEY_COUNT) ? 1 : -1];

void BSP_Key_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    uint8_t i;

    RCC_APB2PeriphClockCmd(BOARD_KEY_GPIO_CLK, ENABLE);

    gpio_init.GPIO_Mode = GPIO_Mode_IPU;     /* 上拉：松开为高，按下为低 */
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    for (i = 0U; i < BSP_KEY_COUNT; i++) {
        gpio_init.GPIO_Pin = s_keys[i].pin;
        GPIO_Init(s_keys[i].gpio, &gpio_init);
    }
}

uint8_t BSP_Key_IsDown(BspKeyId_t key)
{
    if ((uint32_t)key >= BSP_KEY_COUNT) {
        return 0U;
    }

    if (GPIO_ReadInputDataBit(s_keys[key].gpio, s_keys[key].pin) == Bit_RESET) {
        return 1U;
    }

    return 0U;
}
