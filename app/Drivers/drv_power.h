#ifndef __DRV_POWER_H
#define __DRV_POWER_H

#include <stdint.h>
#include "stm32f4xx.h"


void DrvPower_Init(void);
uint32_t DrvPower_GetVinMv(void);

#endif
