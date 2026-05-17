/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-11-22 16:36:10
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-11-23 14:34:08
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\USER\robot_state.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "stm32f4xx.h"
#include "main.h"
#include "PWM.h"
#include "robot_state.h"
#include "CTRL.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Timer.h"

#define ROBOT_CTRL_ACTIVE_PERIOD_MS 10U
#define ROBOT_CTRL_IDLE_PERIOD_MS   100U

volatile Robot_state g_robot_state = ROBOT_STATE_NAV;
volatile CtrlMode    g_mode = MODE_VEL;
volatile TickType_t g_nav_last_rx_tick = 0; // Nav2 软件看门狗
volatile uint8_t    g_nav_cmd_alive    = 0;  // 0=还没收到过 Nav2 命令；1=已激活，看门狗生效

void Robot_EnterState(Robot_state new_state)
{

    g_robot_state = new_state;

    switch (new_state)
    {
    case ROBOT_STATE_IDLE:
        TIM6_SetPeriodMs(ROBOT_CTRL_IDLE_PERIOD_MS);
        g_mode = MODE_MANUAL;
        Set_Pwm(0, 0, 0, 0);
        SC_SetTargets4(0.0f, 0.0f, 0.0f, 0.0f);
        g_nav_cmd_alive = 0;
        break;

    case ROBOT_STATE_TELEOP:
        TIM6_SetPeriodMs(ROBOT_CTRL_ACTIVE_PERIOD_MS);
        // 手动遥控：蓝牙 / 键盘直接给 PWM
        g_mode = MODE_MANUAL;
        Set_Pwm(0, 0, 0, 0);   // 进入时先停一脚，防止从别的模式跳过来时冲一下
        SC_SetTargets4(0.0f, 0.0f, 0.0f, 0.0f);
        g_nav_cmd_alive = 0;
        break;

    case ROBOT_STATE_NAV:
        TIM6_SetPeriodMs(ROBOT_CTRL_ACTIVE_PERIOD_MS);
        // 导航模式：只用速度环 + 上位机目标速度
        g_mode = MODE_VEL;
        SC_SetTargets4(0.0f, 0.0f, 0.0f, 0.0f);
        g_nav_cmd_alive = 0;  // 仅仅进入 NAV，还没收到 Nav2 命令，看门狗先不生效
        break;
    case ROBOT_STATE_PID_TEST:
        TIM6_SetPeriodMs(ROBOT_CTRL_ACTIVE_PERIOD_MS);
        // 速度环测试模式：只用速度环 + 固定目标速度
        g_mode = MODE_VEL;
        SC_SetTargets4(0.2f, 0.2f, 0.2f, 0.2f); // 测试用固定速度
        g_nav_cmd_alive = 0;
        break;

    case ROBOT_STATE_ERROR:
        TIM6_SetPeriodMs(ROBOT_CTRL_IDLE_PERIOD_MS);
        g_mode = MODE_MANUAL;
        Set_Pwm(0, 0, 0, 0);
        SC_SetTargets4(0.0f, 0.0f, 0.0f, 0.0f);
        g_nav_cmd_alive = 0;  // // 退出 nav，顺便关掉看门狗
        break;
    }
}

