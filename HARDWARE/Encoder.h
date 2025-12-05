#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx.h"
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ENC_A = 0, /* TIM2:  PA15 / PB3 */
    ENC_B = 1, /* TIM3:  PB4  / PB5 */
    ENC_C = 2, /* TIM4:  PB6  / PB7 */
    ENC_D = 3  /* TIM5:  PA0  / PA1 */
} EncoderId;


void Encoder_SetDir(EncoderId id, int8_t dir);
void Encoder_Init(void);

/* 周期采样：dt_s 为本次采样间隔（单位秒），例如 0.01f 表示10ms */
void Encoder_Sample(float dt_s);

/* 读取速度 */
float Encoder_Speed_mps(EncoderId id);
float Encoder_Speed_rpm(EncoderId id);
float Encoder_Speed_radps(EncoderId id); 

/* 读取原始差计数（本周期内增量，带正负） */
int32_t Encoder_Delta(EncoderId id);

/* 清零计数器（不会影响速度状态）*/
void Encoder_ResetCounters(void);
uint32_t Encoder_GetCNT(int idx);

#ifdef __cplusplus
}
#endif
#endif
