/*
 * 重定向 printf 到 USART1 (PA9/PA10)，用于 Type-C 调试串口
 */
#include "stm32f4xx.h"
#include <stdio.h>

#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

void _sys_exit(int x) { (void)x; while (1) {} }
void _ttywrch(int ch) { (void)ch; }

/* USART1 初始化：PA9/PA10，纯发送即可，先不搞中断 */
static void USART1_DebugInit(uint32_t baud)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/* printf 最终会调用这里 */
int fputc(int ch, FILE *f)
{
    (void)f;

    // 换行自动补 \r\n，可有可无
    if (ch == '\n')
    {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = '\r';
    }

    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)ch;

    return ch;
}

void debug_printf_init(void)
{
    USART1_DebugInit(115200);
}
