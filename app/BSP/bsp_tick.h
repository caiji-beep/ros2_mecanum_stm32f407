/* bsp_tick.h */
#ifndef __BSP_TICK_H
#define __BSP_TICK_H

#include "stm32f4xx.h"

void     BSP_Tick_Init(void);
uint32_t BSP_Tick_GetUs(void);
float    BSP_Tick_GetDt(float *last_tick);

#endif
