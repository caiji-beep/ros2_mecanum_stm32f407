/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-12-02 20:45:27
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\HARDWARE\Key.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "SPL_Delay.h"
void Key_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // 使能系统配置控制器时钟，用于外部中断线配置
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* 输入脚无影响   */
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // 下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);


	// NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; // 比 FreeRTOS 可中断优先级高一点点即可
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) == 0)
	{
		// SPL_Delay_ms(20);
		// while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) == 0);
		// SPL_Delay_ms(20);
		KeyNum = 1;
	}
	return KeyNum;
}



