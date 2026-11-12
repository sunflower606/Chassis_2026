#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

/* 取上电以来经过的毫秒数（由 Delay_us 内部累加，见 Delay.c 顶部说明） */
uint32_t Delay_GetMs(void);

#endif
