#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>
#include "stm32f4xx.h"

void BSP_ADC_Init(void);
uint16_t BSP_ADC_ReadRaw(void);


#endif
