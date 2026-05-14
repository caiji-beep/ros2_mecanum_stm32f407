#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t gUart2RxQ;
extern QueueHandle_t gUart3RxQ;
extern TaskHandle_t gCtrlTask_Handle;
extern TaskHandle_t gUartTask_Handle;
extern TaskHandle_t gDisplayTask_Handle;
extern QueueHandle_t gsem_OLED_handle;
extern QueueSetHandle_t gqueueset_handle;
extern TaskHandle_t gTelemetryTask_Handle;
extern TaskHandle_t gIMUTask_Handle;
extern uint8_t g_key1_pressed;

void freertos_start(void);

#endif 
