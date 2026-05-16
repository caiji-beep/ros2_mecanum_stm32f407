#include "bsp_adc.h"

void BSP_ADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    // 1. 开 GPIOB 时钟，因为 PB0 在 GPIOB 上
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // 2. 开 ADC1 时钟，ADC1 挂在 APB2 总线上
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // 3. PB0 配成模拟输入
    // 手册要求 ADC/DAC 对应引脚要配置为 Analog
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 4. ADC 公共配置
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//168/2/4
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // 5. ADC1 基本配置
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;              // 单通道，不扫描
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;        // 单次转换
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;                 // 规则序列只有 1 个转换
    ADC_Init(ADC1, &ADC_InitStructure);

    // 6. 规则序列 Rank1 选择 ADC_Channel_8
    // PB0 = ADC_Channel_8
    ADC_RegularChannelConfig(
        ADC1,
        ADC_Channel_8,
        1,
        ADC_SampleTime_84Cycles
    );

    // 7. 使能 ADC1
    ADC_Cmd(ADC1, ENABLE);
}
uint16_t BSP_ADC_ReadRaw(void)
{
    uint32_t timeout = 100000;

    // 清除转换结束标志
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    // 软件启动转换，对应手册里的 SWSTART
    ADC_SoftwareStartConv(ADC1);

    // 等待 EOC，转换结束
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        if (--timeout == 0)
        {
            return 0;
        }
    }

    // 读取 ADC_DR
    return ADC_GetConversionValue(ADC1);

}