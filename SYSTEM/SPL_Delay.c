#include "stm32f4xx.h"
#include "Timer.h"

/* 读毫秒计数（TIM7 更新中断里自增） */
static inline uint32_t millis_TIM7(void)
{
	extern volatile uint32_t g_tim7_ms;
	return g_tim7_ms;
}

/* ---- 毫秒忙等（不关中断 → SysTick 能正常进 → 可观察 RR） ---- */
void SPL_Delay_ms(uint32_t ms)
{
	uint32_t start = millis_TIM7();
	while ((uint32_t)(millis_TIM7() - start) < ms) 
	{
		__asm volatile("nop");    // 防止被优化掉
	}
}

/* ---- 微秒忙等（基于 TIM7->CNT，前提：TIM7 计数 = 1MHz） ---- */
static void SPL_Delay_us_one_shot(uint16_t us) // 单次 <= 1000
{
	if (us == 0)
		return;
	uint16_t s = TIM7->CNT; // 0..999
	while (1)
	{
		uint16_t n = TIM7->CNT; // 0..999
		uint16_t d = (n >= s) ? (n - s) : (uint16_t)(1000U - s + n);
		if (d >= us)
			break;
		__asm volatile("nop");
	}
}

void SPL_Delay_us(uint32_t us)
{
	while (us >= 1000U)
	{
		SPL_Delay_ms(1);
		us -= 1000U;
	}
	SPL_Delay_us_one_shot((uint16_t)us);
}

void SPL_Delay_s(uint32_t s)
{
	while (s--)
		SPL_Delay_ms(1000);
}




