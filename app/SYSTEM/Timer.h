// Timer.h
#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void TIM6_Init(void);
void TIM7_Init(void);
void TIM6_SetPeriodMs(uint16_t period_ms);

/* 10ms 心跳，由 TIM6 中断递增 */
extern volatile uint32_t g_tick10ms;
extern volatile uint32_t g_tim7_ms;

#ifdef __cplusplus
}
#endif
#endif
