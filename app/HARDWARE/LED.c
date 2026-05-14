/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-12-02 23:08:12
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\HARDWARE\LED.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f4xx.h"
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_SetBits(GPIOE,GPIO_Pin_8);

}
void LED1_ON(void)
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_8);
}
void LED1_OFF(void)
{
	GPIO_SetBits(GPIOE,GPIO_Pin_8);
}
void LED1_Turn(void)
{
	if(GPIO_ReadOutputDataBit(GPIOE,GPIO_Pin_8)==0)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_8);
	}
	else
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	}
}



