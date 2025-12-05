#include "stm32f4xx.h"
#include "main.h"
#include "Encoder.h"

/* --- 内部状态 --- */
static TIM_TypeDef* const s_tim[4] = { TIM2, TIM3, TIM4, TIM5 };
static uint32_t       s_period[4];        /* 各定时器自动重装值 */
static int32_t        s_last_cnt[4];      /* 上次读数（带符号扩展）*/
static int32_t        s_delta[4];         /* 本周期增量（脉冲）*/
static float          s_speed_mps[4];     /* 本周期线速度（m/s）*/
static int8_t s_dir[4] = { +1, +1, +1, +1 }; // 默认都正向


void Encoder_SetDir(EncoderId id, int8_t dir)
{
    if ((int)id >= 0 && (int)id < 4)
        s_dir[(int)id] = (dir >= 0) ? +1 : -1;
}

static __inline int32_t read_cnt32(TIM_TypeDef *TIMx)
{
    /* 读 CNT 并做有符号扩展（16位或32位） */
    if (TIMx == TIM3 || TIMx == TIM4) {
        /* 16-bit timer */
        uint16_t c = (uint16_t)TIMx->CNT;
        return (int32_t)c;
    } else {
        /* 32-bit timer TIM2/TIM5 */
        return (int32_t)TIMx->CNT;
    }
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef        g;
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef       ic;

    /* 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2  | RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM4  | RCC_APB1Periph_TIM5, ENABLE);

    /* GPIO 公共配置：AF 上拉 */
    g.GPIO_Mode  = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    /* TIM2  PA15 / PB3  —— 注意：PB3 默认是 SWO，如用到 SWO 需停用 */
    g.GPIO_Pin = GPIO_Pin_15; GPIO_Init(GPIOA, &g);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    g.GPIO_Pin = GPIO_Pin_3;  GPIO_Init(GPIOB, &g);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,  GPIO_AF_TIM2);

    /* TIM3  PB4 / PB5 */
    g.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; GPIO_Init(GPIOB, &g);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    /* TIM4  PB6 / PB7 */
    g.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; GPIO_Init(GPIOB, &g);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

    /* TIM5  PA0 / PA1 */
    g.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; GPIO_Init(GPIOA, &g);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

    /* 定时器基准配置（分别设置 period） */
    tb.TIM_Prescaler     = 0;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;

    /* 输入滤波，抗抖（0..15，对应采样周期加权，6～8较合适） */
    TIM_ICStructInit(&ic);
    ic.TIM_ICFilter = 6;

    /* --- TIM2: 32-bit --- */
    tb.TIM_Period = 0xFFFFFFFF; s_period[ENC_A] = tb.TIM_Period + 1U;
    TIM_TimeBaseInit(TIM2, &tb);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM2, &ic);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);

    /* --- TIM3: 16-bit --- */
    tb.TIM_Period = 0xFFFF;     s_period[ENC_B] = tb.TIM_Period + 1U;
    TIM_TimeBaseInit(TIM3, &tb);
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM3, &ic);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);

    /* --- TIM4: 16-bit --- */
    tb.TIM_Period = 0xFFFF;     s_period[ENC_C] = tb.TIM_Period + 1U;
    TIM_TimeBaseInit(TIM4, &tb);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM4, &ic);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);

    /* --- TIM5: 32-bit --- */
    tb.TIM_Period = 0xFFFFFFFF; s_period[ENC_D] = tb.TIM_Period + 1U;
    TIM_TimeBaseInit(TIM5, &tb);
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM5, &ic);
    TIM_SetCounter(TIM5, 0);
    TIM_Cmd(TIM5, ENABLE);

    /* 初值 */
    for (int i = 0; i < 4; ++i) {
        s_last_cnt[i]  = read_cnt32(s_tim[i]);
        s_delta[i]     = 0;
        s_speed_mps[i] = 0.0f;
    }
}

static void compute_one(int idx, float dt_s)
{
    TIM_TypeDef *T = s_tim[idx];
    uint32_t period = s_period[idx];

    int32_t now  = read_cnt32(T);
    int32_t diff = now - s_last_cnt[idx];
    s_last_cnt[idx] = now;

    /* 溢出处理：若跨过周期一半，按回绕修正 */
    int32_t half = (int32_t)(period >> 1);
    if (diff >  half) diff -= (int32_t)period;
    if (diff < -half) diff += (int32_t)period;
	
	
	diff *= s_dir[idx];          // ★ 每轮独立方向修正
    s_delta[idx] = diff;

    /* 四倍频：每个 A/B 沿都计数 → 一圈脉冲数为 PPR*4 */
    const float rev = (float)diff / (float)(PPR * 4U);
    s_speed_mps[idx] = (rev * WHEEL_CIRCUM_M) / dt_s;
}

void Encoder_Sample(float dt_s)
{
    if (dt_s <= 0.0f) return;
    compute_one(ENC_A, dt_s);
    compute_one(ENC_B, dt_s);
    compute_one(ENC_C, dt_s);
    compute_one(ENC_D, dt_s);
}

float Encoder_Speed_mps(EncoderId id)
{
    return s_speed_mps[(int)id];
}

float Encoder_Speed_rpm(EncoderId id)
{
    /* m/s -> rps -> rpm */
    float rps = s_speed_mps[(int)id] / WHEEL_CIRCUM_M;
    return rps * 60.0f;
}
float Encoder_Speed_radps(EncoderId id)
{
    /* m/s -> rad/s: ω = v / r */
    return s_speed_mps[(int)id] / (WHEEL_DIAMETER_M / 2.0f);
}

int32_t Encoder_Delta(EncoderId id)
{
    return s_delta[(int)id];
}

void Encoder_ResetCounters(void)
{
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
    TIM_SetCounter(TIM5, 0);
    for (int i = 0; i < 4; ++i) {
        s_last_cnt[i]  = 0;
        s_delta[i]     = 0;
        s_speed_mps[i] = 0.0f;
    }
}

uint32_t Encoder_GetCNT(int idx)
{
    if (idx < 0 || idx >= 4) return 0;
    return s_tim[idx]->CNT;
}
