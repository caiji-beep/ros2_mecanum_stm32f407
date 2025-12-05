/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-12-01 19:17:57
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\backup\rtos_ros2_mecanum\2025112303 - 副本\SYSTEM\robot_state.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef __ROBOT_STATE_H
#define __ROBOT_STATE_H
#include "FreeRTOS.h"

typedef enum
{
    MODE_VEL = 0,
    MODE_MANUAL = 1
} CtrlMode;

typedef enum
{
    ROBOT_STATE_IDLE,
    ROBOT_STATE_TELEOP,   // 手动键盘/蓝牙
    ROBOT_STATE_NAV,      // ROS2 Nav2 导航
    ROBOT_STATE_PID_TEST,     // 速度环测试
    ROBOT_STATE_ERROR
} Robot_state;

extern volatile Robot_state g_robot_state;
extern volatile CtrlMode    g_mode;
extern float vA_mps, vB_mps, vC_mps, vD_mps;

// Nav2 看门狗：最后一次收到合法速度包的时间戳
extern volatile TickType_t g_nav_last_rx_tick;
extern volatile uint8_t    g_nav_cmd_alive;

void Robot_EnterState(Robot_state new_state);



#endif
