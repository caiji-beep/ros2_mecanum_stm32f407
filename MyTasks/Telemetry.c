/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-04-27 20:39:18
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\MyTasks\Telemetry.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "freertos_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Uart.h"
#include "Telemetry.h"
#include "Usartx.h"
#include "Usartx.h"
#include "Can.h"
#include "LED.h"
TelemetryData_t g_telemetry_data;
extern ICM20948_ProcessedData_t IMU_processed;
extern ICM20948_RawData_t IMU_data;
void Telemetry_Task(void *pvParameters)
{
    const TickType_t T = pdMS_TO_TICKS(100);
    TickType_t last = xTaskGetTickCount();
    //static int led = 0;
    for (;;)
    {
        vTaskDelayUntil(&last, T); // 固定周期执行
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待 Ctrl_Task 通知
        float wA = g_telemetry_data.wA;
        float wB = g_telemetry_data.wB;
        float wC = g_telemetry_data.wC;
        float wD = g_telemetry_data.wD;
        // if (led)
        //     LED1_OFF();
        // else
        //     LED1_ON();
        // led = !led;
        Serial3_SendMeasPacket(wA, wB, wC, wD);
        ICM20948_ProcessedData_t buffer;
        taskENTER_CRITICAL();
        buffer = IMU_processed;
        taskEXIT_CRITICAL();
        IMU_CAN_SendAll(&buffer);
    }
}
