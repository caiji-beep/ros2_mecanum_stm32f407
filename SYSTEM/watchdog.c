#include "stm32f4xx.h"
#include "watchdog.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_iwdg.h"

void IWDG_Init(uint16_t timeout_ms)
{
    RCC_LSICmd(ENABLE);                           // 使能LSI
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET); // 等待LSI准备好
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG_PR和IWDG_RLR寄存器的写访问
    IWDG_SetPrescaler(IWDG_Prescaler_64);         // 设置预分频器为64

    uint16_t reload = timeout_ms / 2;
    if(reload > 0x0FFF) reload = 0x0FFF;         // 最大重载值为0x0FFF
    IWDG_SetReload(reload);                       // 设置重载值    
    IWDG_ReloadCounter();                         // 重新加载计数器
    IWDG_Enable();                                // 启动看门狗
}

void IWDG_Feed(void)
{
    IWDG_ReloadCounter(); // 重新加载计数器
}