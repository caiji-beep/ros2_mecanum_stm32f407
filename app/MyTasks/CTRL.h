/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-11-22 16:36:09
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-11-22 17:56:18
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\MyTasks\CTRL.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CTRL_H
#define __CTRL_H

#include <stdint.h>
#include "main.h"
#include "Encoder.h"

// PI + 前馈参数（每轮独立，先给一套默认值，后面可分别调）
typedef struct
{
    float Kp;
    float Ki;
    float Kff; // 前馈：m/s -> PWM 的近似比例
} SC_Gains;

typedef struct
{
    // gains
    float Kp, Ki, Kff;
    // states
    float integ;    // 积分器
    float v_target; // 目标速度 m/s
    float v_meas;   // 实测 m/s
    int u_pwm;      // 当前 PWM 输出（限幅后）
} SC_One;


void  SC_Init(void);
void  SC_SetGains(EncoderId id, const SC_Gains* g);  // 可逐轮设置
void  SC_SetTarget_mps(EncoderId id, float v_mps);   // 上层给目标速度（m/s）
float SC_GetTarget_mps(EncoderId id);
float SC_GetMeas_mps(EncoderId id);
int   SC_GetPWM(EncoderId id);

// 主循环或中断里每 10ms 调一次
void  SC_Step(void);

// 一键设 4 轮目标（方便麦轮逆解后下发）
static inline void SC_SetTargets4(float vA, float vB, float vC, float vD) {
    SC_SetTarget_mps(ENC_A, vA);
    SC_SetTarget_mps(ENC_B, vB);
    SC_SetTarget_mps(ENC_C, vC);
    SC_SetTarget_mps(ENC_D, vD);
}

#endif
