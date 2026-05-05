#ifndef __BSP_EXTI_H
#define __BSP_EXTI_H

#include "stm32f4xx.h"

void ICM20948_EXTI_Init(void);
uint8_t ICM20948_EXTI_IsIntPinHigh(void);

#endif
