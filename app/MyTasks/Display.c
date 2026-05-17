/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-17 21:10:59
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\MyTasks\Display.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f4xx.h"
#include "freertos_demo.h"
#include "main.h"
#include "OLED.h"
#include "Encoder.h"
#include "semphr.h"
#include "LED.h"
#include "IMU.h"
#include "robot_state.h"
#include "drv_power.h"

extern QueueHandle_t gsem_OLED_handle;
extern  uint8_t g_key1_pressed;

#define DISPLAY_ACTIVE_PERIOD_MS 110U
#define DISPLAY_IDLE_PERIOD_MS   1000U

static uint8_t oled_page = 0;

static void OLED_Show_mps2(uint8_t line, uint8_t col, float v_mps)
{
    int sign = (v_mps >= 0.0f) ? 1 : -1;
    float av = v_mps * sign;

    // 放大1000并四舍五入
    int32_t v_x1000 = (int32_t)(av * 1000.0f + 0.5f);

    // 限幅到 99.999（避免整数溢出）
    if (v_x1000 > 99999)
        v_x1000 = 99999;

    int32_t ip = v_x1000 / 1000; // 整数部分（0..99）
    int32_t fp = v_x1000 % 1000; // 小数部分（0..999）

    // 符号
    OLED_ShowChar(line, col, (sign > 0) ? '+' : '-'); // col

    // 整数2位（会前导零，比如 00、07、12）
    OLED_ShowNum(line, col + 1, (uint32_t)ip, 2); // col+1..2

    // 小数点
    OLED_ShowChar(line, col + 3, '.'); // col+3

    // 小数3位（前导零）
    OLED_ShowNum(line, col + 4, (uint32_t)fp, 3); // col+4..6

    // 单位
    OLED_ShowString(line, col + 7, "m/s"); // col+7..
}

static void OLED_ShowSpeeds_mps(void)
{
    OLED_ShowString(1, 1, "A:");
    OLED_Show_mps2(1, 3, Encoder_Speed_mps(ENC_A));

    OLED_ShowString(2, 1, "B:");
    OLED_Show_mps2(2, 3, Encoder_Speed_mps(ENC_B));

    OLED_ShowString(3, 1, "C:");
    OLED_Show_mps2(3, 3, Encoder_Speed_mps(ENC_C));

    OLED_ShowString(4, 1, "D:");
    OLED_Show_mps2(4, 3, Encoder_Speed_mps(ENC_D));
}
static void OLED_ShowAngleDeg(uint8_t line, uint8_t col, float deg)
{
    int sign = (deg >= 0.0f) ? 1 : -1;
    float av = deg * sign;

    int32_t v_x100 = (int32_t)(av * 100.0f + 0.5f);
    if (v_x100 > 9999) v_x100 = 9999;  // 限制在 99.99

    int32_t ip = v_x100 / 100;  // 整数
    int32_t fp = v_x100 % 100;  // 小数

    // ±XX.XX
    OLED_ShowChar(line, col, (sign > 0) ? '+' : '-');
    OLED_ShowNum (line, col+1, (uint32_t)ip, 2);
    OLED_ShowChar(line, col+3, '.');
    OLED_ShowNum (line, col+4, (uint32_t)fp, 2);
}



/* 页面1：IMU 姿态（roll/pitch/yaw） */
static void OLED_ShowIMUPage(void)
{
    IMU_State_t imu_data_copy;

    // 复制一份，防止 IMU_Task 正在更新
    if (!IMU_GetSnapshot(&imu_data_copy))
    {
        return;
    }

    if (!imu_data_copy.data_valid)
    {
        OLED_ShowString(1, 1, "IMU fault");
        OLED_ShowNum(2, 1, imu_data_copy.last_error, 3);
        OLED_ShowNum(3, 1, imu_data_copy.read_fail_count, 5);
        return;
    }

    // 标题（可选）
    OLED_ShowString(1,1,"IMU:");
    OLED_ShowNum(1,5,DrvPower_GetVinMv(),5);
    OLED_ShowString(1,10,"mV");

    // 第2行 Roll
    OLED_ShowString(2,1,"R:");
    OLED_ShowAngleDeg(2,3, imu_data_copy.roll_deg);
    OLED_ShowString(2,10,"deg");

    // 第3行 Pitch
    OLED_ShowString(3,1,"P:");
    OLED_ShowAngleDeg(3,3, imu_data_copy.pitch_deg);
    OLED_ShowString(3,10,"deg");

    // 第4行 Yaw
    OLED_ShowString(4,1,"Y:");
    OLED_ShowAngleDeg(4,3, imu_data_copy.yaw_deg);
    OLED_ShowString(4,10,"deg");
}

void Display_Task(void *pvParameters)
{
    // printf("Display_Task started\r\n");
    TickType_t to = xTaskGetTickCount();
    //static int led = 0;
    while (1)
    {
        TickType_t period = pdMS_TO_TICKS(((g_robot_state == ROBOT_STATE_IDLE) ||
                                           (g_robot_state == ROBOT_STATE_ERROR)) ?
                                          DISPLAY_IDLE_PERIOD_MS :
                                          DISPLAY_ACTIVE_PERIOD_MS);

        vTaskDelayUntil(&to, period);
        // printf("Display!\n");
        // 心跳：翻转一个 LED
        // if (led)
        //     LED1_OFF();
        // else
        //     LED1_ON();
        // led = !led;

        if (xSemaphoreTake(gsem_OLED_handle, portMAX_DELAY) == pdTRUE)
        {
            if (g_key1_pressed == 1)
            {
                g_key1_pressed = 0;
                oled_page = (oled_page + 1) % 2;
                OLED_Clear();
            }
            if (oled_page == 0)
            {
                OLED_ShowSpeeds_mps();
            }
            else if (oled_page == 1)
            {
                OLED_ShowIMUPage();
            }
            //OLED_ShowSpeeds_mps();
            //OLED_ShowIMUPage();
            xSemaphoreGive(gsem_OLED_handle);
        }
    }
}
