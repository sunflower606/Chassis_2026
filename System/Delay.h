#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

/* 启动 1 ms SysTick 时基（Delay_us / Delay_GetMs 会自动补调用，可重复调用） */
void Delay_Init(void);

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

/* 取上电以来经过的**真实**毫秒数（SysTick 1 ms 中断累加，见 Delay.c 顶部说明） */
uint32_t Delay_GetMs(void);

#endif
