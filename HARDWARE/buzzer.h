#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f4xx.h"

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

/**
 * 简单阻塞式响一下（单位 ms）
 * 上电阶段可以用 Delay_ms / SPL_Delay
 */
void Buzzer_Beep(uint32_t ms);

/**
 * 看门狗复位时用的提示音模式（比如两短一长）
 */
void Buzzer_Pattern_WatchdogReset(void);

#endif
