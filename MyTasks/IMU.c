/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 20:13:10
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-12-04 22:14:44
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\MyTasks\IMU.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "freertos_demo.h"
#include "FreeRTOS.h"
#include "IMU_Icm20948.h"
#include "IMU.h"
#include "Telemetry.h"
#include "LED.h"

extern ICM20948_RawData_t      IMU_data;
extern ICM20948_Offset_t       IMU_offset;
extern ICM20948_ProcessedData_t IMU_processed;

void IMU_Task(void *pvParameters)
{
    const TickType_t T = pdMS_TO_TICKS(10);
    TickType_t last = xTaskGetTickCount();
    //static int led = 0;
    int fail_streak = 0;
    for (;;)
    {
        vTaskDelayUntil(&last, T); // 固定周期执行
        // if (led)
        //     LED1_OFF();
        // else
        //     LED1_ON();
        // led = !led;
        //ICM20948_ReadData(&IMU_data);
        if(ICM20948_ReadData(&IMU_data))
        {
            LED1_OFF();
            fail_streak = 0;
            ICM20948_Process(&IMU_data, &IMU_offset, &IMU_processed, IMU_dt);
        }
        else
        {
            LED1_ON();
            fail_streak++;
            if(fail_streak >= 5)
            {
                I2C2_BusRecovery();
                ICM20948_Init();    // 重新配置 IMU 寄存器
                fail_streak = 0;
            }
        }
    }
}