#ifndef BSP_KEY_H
#define BSP_KEY_H

/* ============================================================================
 * BSP：三个独立按键（上拉输入，低电平有效）
 *
 * 只提供「原始按下状态」，消抖与边沿检测在 application/chassis_task.c 里做
 * （那边才知道按键代表什么业务含义）。
 *
 * 哪个脚对应哪个键，看 config/pin_config.h 的 BOARD_KEY_MODE/START/STOP_PIN；
 * 本层只按下标读，改引脚不影响这里。
 * ========================================================================== */

#include <stdint.h>

/* 按键个数（与 pin_config.h 里的三条 BOARD_KEY_*_PIN 一一对应） */
#define BSP_KEY_COUNT 3U

typedef enum {
    BSP_KEY_MODE = 0,   /* 中键：切换模式 */
    BSP_KEY_START = 1,  /* 左键：启动 / 暂停 */
    BSP_KEY_STOP = 2    /* 右键：停车 */
} BspKeyId_t;

/* 初始化三个按键为上拉输入（低电平有效） */
void BSP_Key_Init(void);

/**
 * @brief 读取指定按键当前是否被按下。
 * @param key 按键编号（BspKeyId_t），越界返回 0。
 * @return 1 = 按下（引脚低电平）；0 = 未按下。
 * @note 未做消抖，直接轮询会读到抖动。
 */
uint8_t BSP_Key_IsDown(BspKeyId_t key);

#endif /* BSP_KEY_H */
