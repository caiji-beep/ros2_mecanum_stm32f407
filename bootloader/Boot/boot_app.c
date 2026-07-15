#include "boot_app.h"
#include "boot_config.h"

#define BOOT_RCC_INTERRUPT_CLEAR_MASK 0x00FF0000UL
#define BOOT_NVIC_REGISTER_COUNT_MAX  8UL

volatile BootAppStatus_t g_boot_last_status = BOOT_APP_STATUS_OK;

static uint8_t Boot_IsAddressInSram(uint32_t addr)
{
    return ((addr >= SRAM_START_ADDR) && (addr < SRAM_END_ADDR));
}

static uint8_t Boot_GetAppRange(uint32_t app_addr,
                                uint32_t *app_start,
                                uint32_t *app_end)
{
    if ((app_start == 0) || (app_end == 0))
    {
        return 0U;
    }

    if (app_addr == APP_A_ADDR)
    {
        *app_start = APP_A_ADDR;
        *app_end = APP_A_END_ADDR;
        return 1U;
    }

    if (app_addr == APP_B_ADDR)
    {
        *app_start = APP_B_ADDR;
        *app_end = APP_B_END_ADDR;
        return 1U;
    }

    return 0U;
}

BootAppStatus_t Boot_ValidateApp(uint32_t app_addr)
{
    uint32_t app_stack;
    uint32_t app_reset_handler;
    uint32_t reset_address;
    uint32_t app_start;
    uint32_t app_end;

    if (Boot_GetAppRange(app_addr, &app_start, &app_end) == 0U)
    {
        return BOOT_APP_STATUS_INVALID_SLOT;
    }

    app_stack = *(__IO uint32_t *)app_addr;
    app_reset_handler = *(__IO uint32_t *)(app_addr + 4UL);

    if ((app_stack == 0xFFFFFFFFUL) ||
        (app_reset_handler == 0xFFFFFFFFUL))
    {
        return BOOT_APP_STATUS_ERASED_VECTOR;
    }

    if (Boot_IsAddressInSram(app_stack) == 0U)
    {
        return BOOT_APP_STATUS_MSP_RANGE;
    }

    if ((app_stack & 0x7UL) != 0UL)
    {
        return BOOT_APP_STATUS_MSP_ALIGNMENT;
    }

    if ((app_reset_handler & 0x1UL) == 0UL)
    {
        return BOOT_APP_STATUS_RESET_THUMB;
    }

    reset_address = app_reset_handler & 0xFFFFFFFEUL;
    if ((reset_address < app_start) || (reset_address >= app_end))
    {
        return BOOT_APP_STATUS_RESET_RANGE;
    }

    return BOOT_APP_STATUS_OK;
}

uint8_t Boot_AppIsValid(uint32_t app_addr)
{
    return (Boot_ValidateApp(app_addr) == BOOT_APP_STATUS_OK) ? 1U : 0U;
}

static BootAppStatus_t Boot_ReturnClockToHsi(void)
{
    uint32_t timeout;

    RCC->CR |= RCC_CR_HSION;
    timeout = BOOT_CLOCK_TIMEOUT;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0UL)
    {
        if (timeout-- == 0UL)
        {
            return BOOT_APP_STATUS_HSI_TIMEOUT;
        }
    }

    RCC->CFGR &= ~RCC_CFGR_SW;
    timeout = BOOT_CLOCK_TIMEOUT;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
        if (timeout-- == 0UL)
        {
            return BOOT_APP_STATUS_CLOCK_SWITCH_TIMEOUT;
        }
    }

    /* Reset bus prescalers and clock outputs after SYSCLK is safely on HSI. */
    RCC->CFGR = 0x00000000UL;

    RCC->CR &= ~(RCC_CR_PLLON |
                 RCC_CR_PLLI2SON |
                 RCC_CR_HSEON |
                 RCC_CR_CSSON);

    timeout = BOOT_CLOCK_TIMEOUT;
    while ((RCC->CR & (RCC_CR_PLLRDY |
                       RCC_CR_PLLI2SRDY |
                       RCC_CR_HSERDY)) != 0UL)
    {
        if (timeout-- == 0UL)
        {
            return BOOT_APP_STATUS_CLOCK_STOP_TIMEOUT;
        }
    }

    /* HSEBYP may only be changed after HSE is fully stopped. */
    RCC->CR &= ~RCC_CR_HSEBYP;

    /* Disable RCC interrupts and clear every RCC interrupt flag. */
    RCC->CIR = BOOT_RCC_INTERRUPT_CLEAR_MASK;

    return BOOT_APP_STATUS_OK;
}

static BootAppStatus_t Boot_DeInitBeforeJump(void)
{
    uint32_t i;
    uint32_t nvic_register_count;
    BootAppStatus_t status;

    __disable_irq();

    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL = 0UL;

    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    nvic_register_count =
        ((SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) >>
         SCnSCB_ICTR_INTLINESNUM_Pos) + 1UL;
    if (nvic_register_count > BOOT_NVIC_REGISTER_COUNT_MAX)
    {
        nvic_register_count = BOOT_NVIC_REGISTER_COUNT_MAX;
    }

    for (i = 0UL; i < nvic_register_count; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    __DSB();
    __ISB();

    status = Boot_ReturnClockToHsi();
    if (status != BOOT_APP_STATUS_OK)
    {
        return status;
    }

    __set_BASEPRI(0UL);
    __set_FAULTMASK(0UL);
    __set_PSP(0UL);
    __set_CONTROL(0UL);
    __CLREX();

    __DSB();
    __ISB();

    return BOOT_APP_STATUS_OK;
}

/*
 * Keep the final MSP write and branch outside compiler-generated C code.
 * r0 = initial MSP, r1 = Reset_Handler (Thumb bit already checked).
 */
__asm void Boot_BranchToApp(uint32_t app_stack, uint32_t app_reset_handler)
{
    MSR MSP, r0
    DSB
    ISB
    CPSIE i
    BX r1
}

BootAppStatus_t Boot_JumpToApp(uint32_t app_addr)
{
    uint32_t app_stack;
    uint32_t app_reset_handler;
    BootAppStatus_t status;

    status = Boot_ValidateApp(app_addr);
    g_boot_last_status = status;
    if (status != BOOT_APP_STATUS_OK)
    {
        return status;
    }

    app_stack = *(__IO uint32_t *)app_addr;
    app_reset_handler = *(__IO uint32_t *)(app_addr + 4UL);

    status = Boot_DeInitBeforeJump();
    g_boot_last_status = status;
    if (status != BOOT_APP_STATUS_OK)
    {
        return status;
    }

    SCB->VTOR = app_addr;
    __DSB();
    __ISB();

    g_boot_last_status = BOOT_APP_STATUS_OK;
    Boot_BranchToApp(app_stack, app_reset_handler);

    /* Reset_Handler must never return. */
    while (1)
    {
    }
}
