/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-15 09:54:54
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-15 09:56:45
 * @FilePath: \firmware\bootloader\Boot\boot_config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BOOT_CONFIG_H
#define __BOOT_CONFIG_H

#include "stm32f4xx.h"

/*
    STM32F407VE Flash 512KB:
    0x08000000 ~ 0x0807FFFF

    v0.1 规划：
    Bootloader: 0x08000000 ~ 0x0800BFFF, 48KB
    Boot Param: 0x0800C000 ~ 0x0800FFFF, 16KB
    APP_A:      0x08010000 ~ 0x0803FFFF, 192KB
    APP_B:      0x08040000 ~ 0x0807FFFF, 256KB
*/

#define BOOTLOADER_ADDR        0x08000000UL
#define BOOTLOADER_SIZE        0x0000C000UL
#define BOOTLOADER_END_ADDR    (BOOTLOADER_ADDR + BOOTLOADER_SIZE)

#define BOOT_PARAM_ADDR        0x0800C000UL
#define BOOT_PARAM_SIZE        0x00004000UL
#define BOOT_PARAM_END_ADDR    (BOOT_PARAM_ADDR + BOOT_PARAM_SIZE)

#define APP_A_ADDR             0x08010000UL
#define APP_A_SIZE             0x00030000UL
#define APP_A_END_ADDR         (APP_A_ADDR + APP_A_SIZE)

#define APP_B_ADDR             0x08040000UL
#define APP_B_SIZE             0x00040000UL
#define APP_B_END_ADDR         (APP_B_ADDR + APP_B_SIZE)

#define FLASH_START_ADDR       0x08000000UL
#define FLASH_SIZE             0x00080000UL
#define FLASH_END_ADDR         (FLASH_START_ADDR + FLASH_SIZE)

#define SRAM_START_ADDR        0x20000000UL
#define SRAM_SIZE              0x00020000UL
#define SRAM_END_ADDR          (SRAM_START_ADDR + SRAM_SIZE)

/* Maximum polling count used while returning RCC to a reset-like state. */
#define BOOT_CLOCK_TIMEOUT     1000000UL

/* Keep partition mistakes from becoming runtime Flash corruption. */
#if (BOOTLOADER_END_ADDR != BOOT_PARAM_ADDR)
#error "Bootloader and Boot Param partitions are not contiguous"
#endif

#if (BOOT_PARAM_END_ADDR != APP_A_ADDR)
#error "Boot Param and APP_A partitions are not contiguous"
#endif

#if (APP_A_END_ADDR != APP_B_ADDR)
#error "APP_A and APP_B partitions are not contiguous"
#endif

#if (APP_B_END_ADDR != FLASH_END_ADDR)
#error "APP_B does not end at the configured Flash boundary"
#endif

#endif
