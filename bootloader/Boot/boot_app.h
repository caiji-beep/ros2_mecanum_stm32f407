/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-15 09:55:15
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-15 09:57:22
 * @FilePath: \firmware\bootloader\Boot\boot_app.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BOOT_APP_H
#define __BOOT_APP_H

#include "stm32f4xx.h"

typedef enum
{
    BOOT_APP_STATUS_OK = 0,
    BOOT_APP_STATUS_INVALID_SLOT,
    BOOT_APP_STATUS_ERASED_VECTOR,
    BOOT_APP_STATUS_MSP_RANGE,
    BOOT_APP_STATUS_MSP_ALIGNMENT,
    BOOT_APP_STATUS_RESET_THUMB,
    BOOT_APP_STATUS_RESET_RANGE,
    BOOT_APP_STATUS_HSI_TIMEOUT,
    BOOT_APP_STATUS_CLOCK_SWITCH_TIMEOUT,
    BOOT_APP_STATUS_CLOCK_STOP_TIMEOUT
} BootAppStatus_t;

/* Read this variable in the debugger when the Bootloader does not jump. */
extern volatile BootAppStatus_t g_boot_last_status;

BootAppStatus_t Boot_ValidateApp(uint32_t app_addr);
uint8_t Boot_AppIsValid(uint32_t app_addr);
BootAppStatus_t Boot_JumpToApp(uint32_t app_addr);

#endif
