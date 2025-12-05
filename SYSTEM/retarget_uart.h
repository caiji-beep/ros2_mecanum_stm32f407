
#ifndef __DEBUG_PRINTF_H
#define __DEBUG_PRINTF_H

#include "stm32f4xx.h"
#include <stdio.h>

/**
 * @brief 初始化调试串口（USART1）并重定向 printf
 * @param baud 串口波特率（例如 115200）
 */
void debug_printf_init(void);

#endif /* __DEBUG_PRINTF_H */
