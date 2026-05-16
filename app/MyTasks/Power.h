#ifndef __POWER_H
#define __POWER_H

#include <stdint.h>

typedef enum
{
    POWER_STATUS_NORMAL = 0,
    POWER_STATUS_WARN,
    POWER_STATUS_CUTOFF,
    POWER_STATUS_ADC_FAULT
} PowerStatus_t;

void Power_Task(void *pvParameters);
uint32_t Power_GetVinMv(void);
PowerStatus_t Power_GetStatus(void);

#endif
