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

extern QueueHandle_t gUart2RxQ;
extern QueueHandle_t gUart3RxQ;
extern QueueSetHandle_t gqueueset_handle;

extern volatile CtrlMode g_mode;
extern volatile Robot_state g_robot_state;
extern volatile TickType_t g_nav_last_rx_tick;
extern volatile uint8_t g_nav_cmd_alive;

#define SERIAL3_RX_DMA_BUF_SIZE 256


// 使用 AC5 专属的 __attribute__((at(地址))) 语法
// 强制分配到普通 SRAM 的靠后位置 (比如 0x20001000)，避开前面的变量
//__attribute__((at(0x20001000)))
static uint8_t s_serial3_rx_dma_buf[SERIAL3_RX_DMA_BUF_SIZE];

static volatile uint16_t s_serial3_rx_dma_old_pos = 0;

float vA_mps, vB_mps, vC_mps, vD_mps;

static ParserState s_parser_state = STATE_HDR0;
// static uint8_t s_packet_buf[20]; // 存储接收到的数据包
static uint8_t s_data_index = 0;
static uint8_t s_expected_len = 0;

/* ========= 全局变量 ========= */
uint8_t Serial2_RXData;
volatile uint8_t Serial2_RXFlag;

volatile uint16_t Serial3_RXData;
volatile uint8_t Serial3_RXFlag;
volatile uint32_t g_serial3_rx_ok_count;
volatile uint32_t g_serial3_crc_error_count;
volatile uint32_t g_serial3_dma_drain_count;
volatile uint32_t g_serial3_dma_byte_count;
volatile uint32_t g_serial3_idle_irq_count;
volatile uint32_t g_serial3_nav_apply_count;
volatile uint32_t g_serial3_guard_drop_count;
volatile uint16_t g_serial3_dma_pos_dbg;
volatile uint16_t g_serial3_dma_old_pos_dbg;
volatile uint16_t g_serial3_ore_error_count;

#define SERIAL3_WHEEL_SPEED_LIMIT_MPS 1.50f

static float Serial3_ClampWheelSpeed(float v_mps)
{
    if (v_mps > SERIAL3_WHEEL_SPEED_LIMIT_MPS)
        return SERIAL3_WHEEL_SPEED_LIMIT_MPS;
    if (v_mps < -SERIAL3_WHEEL_SPEED_LIMIT_MPS)
        return -SERIAL3_WHEEL_SPEED_LIMIT_MPS;
    return v_mps;
}

/* ========= 初始化 ========= */
void Serial2_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  // Enable the gpio clock  //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // Enable the Usart clock //使能USART时钟

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      // 输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 高速50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);            // 初始化

    // UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    // Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    // Subpriority //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // Enable the IRQ channel //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // Initialize the VIC register with the specified parameters
    // 根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    // USART Initialization Settings 初始化设置
    USART_InitStructure.USART_BaudRate = 9600;                                      // Port rate //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // The word length is 8 bit data format //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // A stop bit //一个停止
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // Prosaic parity bits //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No hardware data flow control //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // Sending and receiving mode //收发模式
    USART_Init(USART2, &USART_InitStructure);                                       // Initialize serial port 2 //初始化串口2

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(USART2, ENABLE);                     // Enable serial port 2 //使能串口2
    // printf("[UART2] Serial_Init done\r\n");
}

/* ========= 发送接口 ========= */
void Serial2_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        ;
}
void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial2_SendByte(Array[i]);
    }
}

void Serial2_SendString(char *String)
{
    uint16_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial2_SendByte(String[i]);
    }
}
uint32_t Serial2_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result = Result * X;
    }
    return Result;
}
void Serial2_SendNum(uint32_t Num, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        Serial2_SendByte(Num / Serial2_Pow(10, Length - i - 1) % 10 + 0x30);
    }
}
// int fputc(int ch, FILE *f)
// {
//     Serial2_SendByte(ch);
//     return ch;
// }
void Serial2_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial2_SendString(String);
}
uint8_t Serial_GetRXFlag(void)
{
    if (Serial2_RXFlag == 1)
    {
        Serial2_RXFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t Serial2_GetRXData(void)
{
    return Serial2_RXData;
}

/* =============== blue串口命令处理（把行为名字化） =============== */
void HandleCommand(uint8_t cmd)
{

    // printf("[UART2] RX: 0x%02X '%c'\r\n", Serial_RXData, Serial_RXData);
    switch (cmd)
    {

    case 'i': // idle 停车
        Robot_EnterState(ROBOT_STATE_IDLE);
        break;

    case 'm': // manual 手动模式
        Robot_EnterState(ROBOT_STATE_TELEOP);
        break;

    case 'n': // nav 导航模式
        Robot_EnterState(ROBOT_STATE_NAV);
        break;

    case 'p': // pid test 速度环测试模式
        Robot_EnterState(ROBOT_STATE_PID_TEST);
        break;

    case 'e': // error 急停
        Robot_EnterState(ROBOT_STATE_ERROR);
        break;
        // case 0x30: /* '0'：切到速度环，使用目标 vA~vD（m/s） */
        //     // printf("[UART2] CMD: mode=VEL\n");
        //     g_mode = MODE_VEL;
        //     SC_SetTargets4(0.2f, 0.2f, 0.2f, 0.2f);
        //     break;

    case 0x31: /* '1'：前进 */
               // printf("[UART2] CMD: mode=MANUAL\n");
        if (g_robot_state == ROBOT_STATE_TELEOP)
        {
            Set_Pwm(MANUAL_PWM, MANUAL_PWM, MANUAL_PWM, MANUAL_PWM);
        }
        break;

    case 0x32: /* '2'：后退 */
        // printf("[UART2] CMD: mode=MANUAL\n");
        if (g_robot_state == ROBOT_STATE_TELEOP)
        {
            Set_Pwm(-MANUAL_PWM, -MANUAL_PWM, -MANUAL_PWM, -MANUAL_PWM);
        }
        break;

    case 0x33: /* '3'：水平左移 */
        // printf("[UART2] CMD: mode=MANUAL\n");
        if (g_robot_state == ROBOT_STATE_TELEOP)
        {
            Set_Pwm(MANUAL_PWM, -MANUAL_PWM, MANUAL_PWM, -MANUAL_PWM);
        }
        break;

    case 0x34: /* '4'：水平右移 */
        // printf("[UART2] CMD: mode=MANUAL\n");
        if (g_robot_state == ROBOT_STATE_TELEOP)
        {
            Set_Pwm(-MANUAL_PWM, MANUAL_PWM, -MANUAL_PWM, MANUAL_PWM);
        }
        break;

    case 0x35: /* '5'：原地左转 */
        // printf("[UART2] CMD: mode=MANUAL\n");
        if (g_robot_state == ROBOT_STATE_TELEOP)
        {
            Set_Pwm(MANUAL_PWM, -MANUAL_PWM, -MANUAL_PWM, MANUAL_PWM);
        }
        break;

    case 0x36: /* '6'：原地右转 */
        // printf("[UART2] CMD: mode=MANUAL\n");
        if (g_robot_state == ROBOT_STATE_TELEOP)
        {
            Set_Pwm(-MANUAL_PWM, MANUAL_PWM, MANUAL_PWM, -MANUAL_PWM);
        }
        break;

    default: /* 其它：停车（保持手动模式不变更） */
        if (g_robot_state == ROBOT_STATE_PID_TEST)
        {
            // 退回 IDLE
            Robot_EnterState(ROBOT_STATE_IDLE);
        }
        else
        {
            Set_Pwm(0, 0, 0, 0);
            SC_SetTargets4(0.0f, 0.0f, 0.0f, 0.0f);
        }
        break;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 添加函数原型声明
void Serial3_SendArray(uint8_t *Array, uint16_t Length);
void Serial3_SendByte(uint8_t Byte);
void Serial3_SendFloat(float value);
void Serial3_SendMeasPacket(float w1, float w2, float w3, float w4);
void Serial3_ParsePacket(uint8_t data);
uint16_t Serial3_CRC16(const uint8_t *data, size_t len);

/* ========= 串口3初始化 ========= */
void Serial3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能GPIOD和USART3时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // 配置PD8和PD9为复用功能
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); // TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); // RX

    // 配置GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 优先级低于串口2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 配置USART3
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    // 使能接收中断
    //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    // 使能USART3
    

    /*配置DMA DMA1_Stream1 DMA_Channel_4*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Stream1);
    while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE)
    {
    }

    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)s_serial3_rx_dma_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = SERIAL3_RX_DMA_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1, ENABLE);

    /*清idle中断标志位*/
    volatile uint32_t tmp;
    tmp = USART3->SR;
    tmp = USART3->DR;
    (void)tmp;

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

/* ========= 串口3发送字节 ========= */
void Serial3_SendByte(uint8_t Byte)
{
    USART_SendData(USART3, Byte);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
        ; // 等待发送完毕
}

/* ========= 串口3发送数组 ========= */
void Serial3_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial3_SendByte(Array[i]);
    }
}

/* ========= 串口3发送浮点数 ========= */
void Serial3_SendFloat(float value)
{
    uint8_t *p = (uint8_t *)&value;
    Serial3_SendArray(p, 4);
}

/* ========= 获取串口3接收标志 ========= */
uint8_t Serial3_GetRXFlag(void)
{
    if (Serial3_RXFlag == 1)
    {
        Serial3_RXFlag = 0;
        return 1;
    }
    return 0;
}

void Serial3_RxDmaDrain(void)
{
    //清楚ore,如果 ORE 不清除，DMA 会永久停止接收
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        volatile uint32_t tmp;
        tmp = USART3->SR;
        tmp = USART3->DR;
        (void)tmp; // 先读 SR 再读 DR 即可清除 ORE 标志位

        g_serial3_ore_error_count++;
    }
    uint16_t pos = SERIAL3_RX_DMA_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);

    

    if (pos >= SERIAL3_RX_DMA_BUF_SIZE)
    {
        pos = 0;
    }

    g_serial3_dma_drain_count++;
    g_serial3_dma_pos_dbg = pos;
    g_serial3_dma_old_pos_dbg = s_serial3_rx_dma_old_pos;

    while (s_serial3_rx_dma_old_pos != pos)
    {
        Serial3_ParsePacket(s_serial3_rx_dma_buf[s_serial3_rx_dma_old_pos]);
        g_serial3_dma_byte_count++;

        s_serial3_rx_dma_old_pos++;
        if (s_serial3_rx_dma_old_pos >= SERIAL3_RX_DMA_BUF_SIZE)
        {
            s_serial3_rx_dma_old_pos = 0;
        }
    }

    g_serial3_dma_old_pos_dbg = s_serial3_rx_dma_old_pos;
}


void Serial3_ParsePacket(uint8_t data)
{
    static uint8_t packet[22];
    static uint8_t index = 0;

    switch (s_parser_state)
    {
    case STATE_HDR0:
        if (data == 0xAA)
        {
            s_parser_state = STATE_HDR1;
            packet[0] = data;
            index = 1;
        }
        break;

    case STATE_HDR1:
        if (data == 0x55)
        {
            s_parser_state = STATE_ID;
            packet[1] = data;
            index = 2;
        }
        else
        {
            s_parser_state = STATE_HDR0;
        }
        break;

    case STATE_ID:
        if (data == 0x01)
        { // 命令包
            s_parser_state = STATE_LEN;
            packet[2] = data;
            index = 3;
        }
        else
        {
            s_parser_state = STATE_HDR0; // 不是命令包，重置
        }
        break;

    case STATE_LEN:
        if (data == 0x10)
        { // 数据长度16字节
            s_parser_state = STATE_DATA;
            packet[3] = data;
            index = 4;
            s_expected_len = 16;
            s_data_index = 0;
        }
        else
        {
            s_parser_state = STATE_HDR0; // 长度不符，重置
        }
        break;

    case STATE_DATA:
        packet[index++] = data;
        s_data_index++;

        if (s_data_index >= s_expected_len)
        {
            s_parser_state = STATE_CRC0;
        }
        break;

    case STATE_CRC0:
        packet[index++] = data;
        s_parser_state = STATE_CRC1;
        break;

    case STATE_CRC1:
        packet[index] = data;

        // 验证CRC
        uint16_t calc_crc = Serial3_CRC16(packet, 20); // 包头+数据
        uint16_t recv_crc = packet[20] | (packet[21] << 8);

        if (calc_crc == recv_crc)
        {
            g_serial3_rx_ok_count++;
            // CRC校验通过，处理数据
            float speeds[4];
            memcpy(speeds, &packet[4], sizeof(speeds));
            vA_mps = speeds[0] * (WHEEL_DIAMETER_M / 2.0f);
            vB_mps = speeds[1] * (WHEEL_DIAMETER_M / 2.0f);
            vC_mps = speeds[2] * (WHEEL_DIAMETER_M / 2.0f);
            vD_mps = speeds[3] * (WHEEL_DIAMETER_M / 2.0f);
            vA_mps = Serial3_ClampWheelSpeed(vA_mps);
            vB_mps = Serial3_ClampWheelSpeed(vB_mps);
            vC_mps = Serial3_ClampWheelSpeed(vC_mps);
            vD_mps = Serial3_ClampWheelSpeed(vD_mps);

            if (fabsf(vA_mps) < 0.02f)
            {
                vA_mps = 0.0f;
            }
            if (fabsf(vB_mps) < 0.02f)
            {
                vB_mps = 0.0f;
            }
            if (fabsf(vC_mps) < 0.02f)
            {
                vC_mps = 0.0f;
            }
            if (fabsf(vD_mps) < 0.02f)
            {
                vD_mps = 0.0f;
            }

            // if (g_robot_state != ROBOT_STATE_NAV)
            // {
            //     Robot_EnterState(ROBOT_STATE_NAV);
            // }
            if (g_robot_state == ROBOT_STATE_NAV && g_mode == MODE_VEL)
            {
                SC_SetTargets4(vA_mps, vB_mps, vC_mps, vD_mps);
                g_nav_cmd_alive = 1;
                g_serial3_nav_apply_count++;
                g_nav_last_rx_tick = xTaskGetTickCount(); // 刷新nav2看门狗
            }

            else
            {
                g_serial3_guard_drop_count++;
            }

            // SC_SetTargets4(vA_mps, vB_mps, vC_mps, vD_mps);
            // g_mode = MODE_VEL;
        }
        else
        {
            g_serial3_crc_error_count++;
        }

        s_parser_state = STATE_HDR0; // 重置状态机
        break;
    }
}

uint16_t Serial3_CRC16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

void Serial3_SendMeasPacket(float w1, float w2, float w3, float w4)
{
    uint8_t buf[22]; // 4字节头 + 16字节数据 + 2字节CRC

    // 包头
    buf[0] = 0xAA; // HDR0
    buf[1] = 0x55; // HDR1
    buf[2] = 0x02; // ID_MEAS
    buf[3] = 0x10; // LEN_BODY

    // 打包4个浮点数
    memcpy(&buf[4], &w1, 4); // 小端序
    memcpy(&buf[8], &w2, 4);
    memcpy(&buf[12], &w3, 4);
    memcpy(&buf[16], &w4, 4);

    // 计算CRC16
    uint16_t crc = Serial3_CRC16(buf, 20);
    buf[20] = crc & 0xFF; // 发送CRC也是小端序
    buf[21] = (crc >> 8) & 0xFF;

    // 发送数据包
    Serial3_SendArray(buf, 22);
}

// /* ========= 串口2中断服务函数 ========= */
// void USART2_IRQHandler(void)
// {
//     if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
//     {
//         USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//         // printf("USART2_IRQHandler\r\n");
//         Serial2_RXData = USART_ReceiveData(USART2);
//         // printf("IRQ RX: 0x%02X '%c'\r\n", Serial_RXData, Serial_RXData);
//         Serial2_RXFlag = 1;
//         // xQueueSendToBackFromISR(gUart2RxQ, &Serial_RXData, NULL);

//         BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//         if (gUart2RxQ)
//         {
//             xQueueSendFromISR(gUart2RxQ, &Serial2_RXData, &xHigherPriorityTaskWoken);
//         }
//         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     }
// }

// /* ========= 串口3中断服务函数 ========= */

// void USART3_IRQHandler(void)
// {
//     if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
//     {
//         USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//         // printf("USART3_IRQHandler\r\n");
//         uint8_t data = USART_ReceiveData(USART3);
//         Serial3_RXFlag = 1;

//         BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//         if (gUart3RxQ)
//         {
//             xQueueSendFromISR(gUart3RxQ, &data, &xHigherPriorityTaskWoken);
//         }

//         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//         // Serial3_ParsePacket(data); // 解析协议
//     }
// }
