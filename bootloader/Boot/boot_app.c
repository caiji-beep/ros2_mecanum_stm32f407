/**
 * @file boot_app.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-05-15
 * 1. 判断 APP 向量表是否合法
 * 2. 设置 VTOR
 * 3. 设置 MSP
 * 4. 跳转 APP Reset_Handler
 * @copyright Copyright (c) 2026
 *
 */

#include "boot_app.h"
#include "boot_config.h"

typedef void (*AppEntry_t)(void);

static uint8_t Boot_IsAddressInSram(uint32_t addr)
{
    return ((addr >= SRAM_START_ADDR) && (addr <= SRAM_END_ADDR));
}

static uint8_t Boot_GetAppRange(uint32_t app_addr, uint32_t *app_start, uint32_t *app_end)
{
    if (app_addr == APP_A_ADDR)
    {
        *app_start = APP_A_ADDR;
        *app_end = APP_A_ADDR + APP_A_SIZE;
        return 1;
    }

    if (app_addr == APP_B_ADDR)
    {
        *app_start = APP_B_ADDR;
        *app_end = APP_B_ADDR + APP_B_SIZE;
        return 1;
    }

    return 0;
}

static uint8_t Boot_IsResetHandlerInApp(uint32_t addr, uint32_t app_start, uint32_t app_end)
{
    uint32_t real_addr = addr & 0xFFFFFFFEUL;

    return ((real_addr >= app_start) && (real_addr < app_end));
}

uint8_t Boot_AppIsValid(uint32_t app_addr)
{
    uint32_t app_stack;
    uint32_t app_reset_handler;
    uint32_t app_start;
    uint32_t app_end;

    if (Boot_GetAppRange(app_addr, &app_start, &app_end) == 0)
    {
        return 0;
    }

    app_stack = *(__IO uint32_t *)app_addr;
    app_reset_handler = *(__IO uint32_t *)(app_addr + 4);

    if ((app_stack == 0xFFFFFFFFUL) || (app_reset_handler == 0xFFFFFFFFUL))
    {
        return 0;
    }

    if (Boot_IsAddressInSram(app_stack) == 0)
    {
        return 0;
    }

    if (Boot_IsResetHandlerInApp(app_reset_handler, app_start, app_end) == 0)
    {
        return 0;
    }

    /*
        Cortex-M4 只运行 Thumb 指令，Reset_Handler 地址 bit0 应该为 1。
    */
    if ((app_reset_handler & 0x00000001UL) == 0)
    {
        return 0;
    }

    return 1;
}

static void Boot_DeInitBeforeJump(void)
{
    uint8_t i;

    __disable_irq();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    for (i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }
}

void Boot_JumpToApp(uint32_t app_addr)
{
    uint32_t app_stack;
    uint32_t app_reset_handler;
    AppEntry_t app_entry;

    if (Boot_AppIsValid(app_addr) == 0)
    {
        return;
    }

    app_stack = *(__IO uint32_t *)app_addr;
    app_reset_handler = *(__IO uint32_t *)(app_addr + 4);

    Boot_DeInitBeforeJump();

    SCB->VTOR = app_addr;

    __set_CONTROL(0);

    __DSB();
    __ISB();

    __set_MSP(app_stack);

    __DSB();
    __ISB();

    app_entry = (AppEntry_t)app_reset_handler;

    __enable_irq();

    app_entry();
}
