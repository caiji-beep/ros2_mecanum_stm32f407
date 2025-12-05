/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-12-03 23:54:37
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\backup\rtos_ros2_mecanum\2025112303 - 副本\MyTasks\Uart.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "stm32f4xx.h"
#include "freertos_demo.h"
#include "main.h"
#include "queue.h"
#include <stdio.h>
#include <stdarg.h>
#include "CTRL.h"
#include "PWM.h"
#include <string.h>
#include "Uart.h"
#include "robot_state.h"
#include "Telemetry.h"
#include <math.h>
#include "Usartx.h"
#include "LED.h"

extern QueueHandle_t gUart2RxQ;
extern QueueHandle_t gUart3RxQ;
extern QueueSetHandle_t gqueueset_handle;

extern uint8_t Serial2_RXData;
extern volatile uint8_t Serial2_RXFlag;

extern volatile uint16_t Serial3_RXData;
extern volatile uint8_t Serial3_RXFlag;

void Uart_Task(void *pvParameters)
{
    QueueSetMemberHandle_t xActivatedMember;
    uint8_t rx_byte;
    uint8_t processed = 0;
    //  printf("[UART] Uart_Task started\r\n");
    // static int led = 0;
    while (1)
    {
        // 1. 阻塞等待，直到任一队列有数据
        xActivatedMember = xQueueSelectFromSet(gqueueset_handle, portMAX_DELAY);
        // if (led)
        //     LED1_OFF();
        // else
        //     LED1_ON();
        // led = !led;
        if (xActivatedMember == gUart2RxQ)
        {
            if (xQueueReceive(gUart2RxQ, &rx_byte, 0) == pdTRUE)
            {
                // printf("[UART2] waiting rx...\r\n");
                HandleCommand(rx_byte); // uart2
                processed++;
            }
        }
        else if (xActivatedMember == gUart3RxQ)
        {
            if (xQueueReceive(gUart3RxQ, &rx_byte, 0) == pdTRUE)
            {
                // printf("[UART3] waiting rx...\r\n");
                Serial3_ParsePacket(rx_byte); // uart3
                processed++;
            }
        }
        if(processed >= 32)
        {
            processed = 0;
            vTaskDelay(1); // 让出CPU
        }
    }
}

/* ========= 串口2中断服务函数 ========= */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        // printf("USART2_IRQHandler\r\n");
        Serial2_RXData = USART_ReceiveData(USART2);
        // printf("IRQ RX: 0x%02X '%c'\r\n", Serial_RXData, Serial_RXData);
        Serial2_RXFlag = 1;
        // xQueueSendToBackFromISR(gUart2RxQ, &Serial_RXData, NULL);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (gUart2RxQ)
        {
            xQueueSendFromISR(gUart2RxQ, &Serial2_RXData, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ========= 串口3中断服务函数 ========= */

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        // printf("USART3_IRQHandler\r\n");
        uint8_t data = USART_ReceiveData(USART3);
        Serial3_RXFlag = 1;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (gUart3RxQ)
        {
            xQueueSendFromISR(gUart3RxQ, &data, &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //Serial3_ParsePacket(data); // 解析协议
    }
}
