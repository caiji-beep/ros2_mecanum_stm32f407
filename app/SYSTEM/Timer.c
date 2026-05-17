/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-10-27 20:41:36
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-11-21 13:10:26
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\rtos_ros2_mecanum\SYSTEM\Timer.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f4xx.h"
#include "Encoder.h"
#include "CTRL.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_demo.h"

extern TaskHandle_t gCtrlTask_Handle;

volatile uint32_t g_tick10ms = 0;
volatile uint32_t g_tim7_ms = 0;

void TIM6_SetPeriodMs(uint16_t period_ms)
{
    uint32_t arr;

    if (period_ms == 0U)
    {
        period_ms = 1U;
    }

    /* TIM6 PSC=8399 gives a 10 kHz counter clock, so one count is 0.1 ms. */
    arr = ((uint32_t)period_ms * 10U) - 1U;
    if (arr > 0xFFFFU)
    {
        arr = 0xFFFFU;
    }

    TIM_Cmd(TIM6, DISABLE);
    TIM_SetAutoreload(TIM6, (uint16_t)arr);
    TIM_SetCounter(TIM6, 0U);
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);

    // APB1定时器时钟 = 84 MHz（常见配置）
    // 目标周期 T = (PSC+1)*(ARR+1) / 84e6 = 10ms
    // 取 PSC=8399, ARR=99 -> T = 8400*100 / 84e6 = 0.01 s
    tb.TIM_Prescaler = 8399;
    tb.TIM_Period = 99;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6, &tb);

    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM6, ENABLE);
}

void TIM7_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);

    // APB1定时器时钟 = 84 MHz（常见配置）
    // 目标周期 T = (PSC+1)*(ARR+1) / 84e6 = 1ms
    tb.TIM_Prescaler = 83;
    tb.TIM_Period = 999;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &tb);

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM7, ENABLE);
}


void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        if(gCtrlTask_Handle != NULL)
        {
            // 给 Ctrl_Task 发一个“+1 通知”
            vTaskNotifyGiveFromISR(gCtrlTask_Handle, &pxHigherPriorityTaskWoken); // 通知任务
        }
        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

        

        // Encoder_Sample(0.01f);   // 精确10ms采样
        // g_tick10ms++; // 可选：给主循环一个节拍

        // Encoder_Sample(SC_Ts); // 10ms 编码器采样
        // if (g_mode == MODE_VEL)
        // {
        //     SC_Step(); // 只有速度环模式才运行 PI 并下发 PWM  // 10ms 速度环
        // }
        // SC_Step();
    }
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) // 检测指定的TIM中断发生与否:TIM 中断源
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update); // 清除TIMx的中断待处理位:TIM 中断源
        // 在这里添加用户代码
        g_tim7_ms++;
    }
}
