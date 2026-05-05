/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-05 16:43:57
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\USER\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <stddef.h>
#include "stm32f4xx.h"
#include "freertos_demo.h"
#include "retarget_uart.h"
#include "SPL_Delay.h"
#include "Can.h"
#include "Timer.h"
#include "Usartx.h"
#include "Key.h"
#include "OLED.h"
#include "Can.h"
#include "LED.h"
#include "SPL_Delay.h"
#include "bsp_soft_i2c.h"
#include "icm20948.h"
#include "bsp_exti.h"
#include "imu_fusion.h"
#include "bsp_tick.h"

extern CanRxMsg MyCan_RxMsg;
extern uint8_t MyCan_RxFlag;

uint8_t KeyNum;

int main(void)
{
	debug_printf_init();
	printf("Hello FreeRTOS + ROS2 Mecanum Robot!2025120102\n");
	freertos_start();

	while (1)
	{
	}
}
