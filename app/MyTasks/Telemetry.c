/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-05 16:46:49
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
#include "IMU.h"
TelemetryData_t g_telemetry_data;

void Telemetry_Task(void *pvParameters)
{
    const TickType_t T = pdMS_TO_TICKS(100);
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&last, T); // 固定周期执行
        
        float wA = g_telemetry_data.wA;
        float wB = g_telemetry_data.wB;
        float wC = g_telemetry_data.wC;
        float wD = g_telemetry_data.wD;

        Serial3_SendMeasPacket(wA, wB, wC, wD);
        {
            IMU_State_t imu_state_buffer;
            if (IMU_GetSnapshot(&imu_state_buffer))
            {
                IMU_CAN_SendAll(&imu_state_buffer);
            }
        }
    }
}
