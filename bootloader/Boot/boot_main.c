#include "boot_main.h"
#include "boot_config.h"
#include "boot_app.h"

void Boot_Main(void)
{
    /*
     * Step 1 only: APP_A is the fixed boot target.
     * A/B slot selection will be added after Boot Param is power-loss safe.
     */
    g_boot_last_status = Boot_JumpToApp(APP_A_ADDR);

    /* Stay in Bootloader when APP_A or the handover state is invalid. */
    while (1)
    {
        __NOP();
    }
}
