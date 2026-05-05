/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 23:06:05
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-05 16:47:45
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\HARDWARE\Can.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx_can.h"
#include "IMU.h"

extern CanRxMsg MyCan_RxMsg;
extern uint8_t MyCan_RxFlag;

void CAN1_Init(void);
uint8_t CAN1_Send(CanTxMsg* TxMessage);
/* 0x180 accel(mg), 0x181 gyro(mrad/s), 0x182 attitude(mrad), 0x184 status.
 * Status flags: bit0 valid, bit1 calibrated, bit2 recovering, bit3 fault.
 */
void IMU_CAN_SendAll(const IMU_State_t *p);

#endif
