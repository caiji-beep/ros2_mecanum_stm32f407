/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-14 15:18:52
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-14 19:04:38
 * @FilePath: \firmware\app\Drivers\drv_power.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "drv_power.h"
#include "bsp_adc.h"

#define ADC_VREF_MV        3300UL
#define ADC_FULL_SCALE     4095UL

#define VIN_DIV_GAIN       11UL
#define VIN_SAMPLE_COUNT   16

void DrvPower_Init(void)
{
    BSP_ADC_Init();
}

uint16_t DrvPower_GetVinRaw(void)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < VIN_SAMPLE_COUNT; i++)
    {
        sum += BSP_ADC_ReadRaw();
    }

    return (uint16_t)(sum / VIN_SAMPLE_COUNT);
}

uint32_t DrvPower_GetVinMv(void)
{
    uint32_t raw = DrvPower_GetVinRaw();

    return raw * ADC_VREF_MV * VIN_DIV_GAIN / ADC_FULL_SCALE;
}