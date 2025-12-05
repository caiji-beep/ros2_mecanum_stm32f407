/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-10-27 20:41:36
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-10-27 20:59:55
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\rtos_ros2_mecanum\HARDWARE\PWM.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __PWM_H
#define __PWM_H




void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM9_PWM_Init(u16 arr,u16 psc);
void TIM10_PWM_Init(u16 arr,u16 psc);
void TIM11_PWM_Init(u16 arr,u16 psc);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);

#endif
