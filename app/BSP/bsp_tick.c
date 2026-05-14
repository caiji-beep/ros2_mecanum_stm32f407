/* bsp_tick.c */
#include "bsp_tick.h"

void BSP_Tick_Init(void)
{
    /* 开启 DWT 周期计数器，零配置 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t BSP_Tick_GetUs(void)
{
    /* CPU 周期数 / CPU频率(MHz) = 微秒 */
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

float BSP_Tick_GetDt(float *last_tick)
{
    float now = (float)BSP_Tick_GetUs() / 1000000.0f;
    float dt = now - *last_tick;
    *last_tick = now;

    if (dt <= 0.0f || dt > 0.5f)
        dt = 0.01f;

    return dt;
}
