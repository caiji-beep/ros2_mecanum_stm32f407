/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-11-22 16:36:10
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-12-01 21:09:35
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\USER\main.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"

/*--------Motor_A control pins--------*/
#define PWM_PORTA1 GPIOB      // PWMA
#define PWM_PIN_A1 GPIO_Pin_8 // PWMA
#define PWMA1 TIM10->CCR1     // PWMA

#define PWM_PORTA2 GPIOB      // PWMA
#define PWM_PIN_A2 GPIO_Pin_9 // PWMA
#define PWMA2 TIM11->CCR1     // PWMA
/*------------------------------------*/

/*--------Motor_B control pins--------*/
#define PWM_PORTB1 GPIOE      // PWMB
#define PWM_PIN_B1 GPIO_Pin_5 // PWMB
#define PWMB1 TIM9->CCR1      // PWMB

#define PWM_PORTB2 GPIOE      // PWMB
#define PWM_PIN_B2 GPIO_Pin_6 // PWMB
#define PWMB2 TIM9->CCR2      // PWMB

/*------------------------------------*/

/*--------Motor_C control pins--------*/
#define PWM_PORTC1 GPIOE      // PWMC
#define PWM_PIN_C1 GPIO_Pin_9 // PWMC
#define PWMC1 TIM1->CCR1      // PWMC

#define PWM_PORTC2 GPIOE       // PWMC
#define PWM_PIN_C2 GPIO_Pin_11 // PWMC
#define PWMC2 TIM1->CCR2       // PWMC

/*------------------------------------*/

/*--------Motor_D control pins--------*/
#define PWM_PORTD1 GPIOE       // PWMD
#define PWM_PIN_D1 GPIO_Pin_13 // PWMD
#define PWMD1 TIM1->CCR3       // PWMD

#define PWM_PORTD2 GPIOE       // PWMD
#define PWM_PIN_D2 GPIO_Pin_14 // PWMD
#define PWMD2 TIM1->CCR4       // PWMD

/*========== 电机调速接口（带PWM限幅） ==========*/
#define PWM_MAX 16799 //
#define PWM_MIN 0     // 最小值

#define MANUAL_PWM 2500 // 手动模式下的PWM值    2500

/* 机械参数 */
#define ENCODER_LINES 11U                /* 编码器线数（电机轴）*/
#define GEAR_RATIO 30U                   /* 减速比 */
#define PPR (ENCODER_LINES * GEAR_RATIO) /* 输出轴每圈脉冲数 */
#define WHEEL_DIAMETER_M 0.08f           /* 80 mm */
#define WHEEL_CIRCUM_M (3.1415926f * WHEEL_DIAMETER_M)

#define SC_Ts (0.01f)                 // 控制周期 10ms（与TIM6一致）
#define SC_PWM_MAX (16799)            // 你的 PWM 最大值（按实际定时器ARR来）
#define SC_SLEW_PER_TICK (SC_PWM_MAX) // 每个周期 PWM 变化最大步幅（斜坡限幅）
#define SC_I_MIN (-8000.0f)           // 积分限幅
#define SC_I_MAX (8000.0f)
#define SC_OUT_MIN (-SC_PWM_MAX)
#define SC_OUT_MAX (SC_PWM_MAX)

#endif
