#ifndef __SPL_DELAY_H__
#define __SPL_DELAY_H__
#include <stdint.h>

void SPL_Delay_ms(uint32_t ms);   // 始终忙等（RTOS 下也不让出）
void SPL_Delay_us(uint32_t us);   // 基于 TIM7->CNT（1MHz）
void SPL_Delay_s(uint32_t s);

#endif
