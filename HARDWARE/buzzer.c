#include "stm32f4xx.h"
#include "SPL_Delay.h"

#define Buzzer_PIN GPIO_Pin_8 // PA8

void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA时钟
    GPIO_InitStructure.GPIO_Pin = Buzzer_PIN;             // buzzer对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         // 普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                // 初始化GPIO
}
void Buzzer_On(void)
{
    GPIO_SetBits(GPIOA, Buzzer_PIN);
}
void Buzzer_Off(void)
{
    GPIO_ResetBits(GPIOA, Buzzer_PIN);
}

void Buzzer_Beep(uint32_t ms)
{
    Buzzer_On();
    SPL_Delay_ms(ms);
    Buzzer_Off();
}

void Buzzer_Pattern_WatchdogReset(void)
{
    Buzzer_Beep(80);
    SPL_Delay_ms(80);
    Buzzer_Beep(80);
    SPL_Delay_ms(120);
    Buzzer_Beep(200);
}
