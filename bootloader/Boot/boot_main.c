/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-15 09:55:54
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-15 10:41:58
 * @FilePath: \firmware\bootloader\Boot\boot_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "boot_main.h"
#include "boot_config.h"
#include "boot_app.h"

static void Boot_Delay(volatile uint32_t count)
{
    while (count--)
    {
        __NOP();
    }
}

void Boot_Main(void)
{
    /*
        v0.1:
        不初始化 CAN
        不擦写 Flash
        不做 OTA
        只检查并跳转 APP_A
    */

    Boot_Delay(8000000);

    if (Boot_AppIsValid(APP_A_ADDR))
    {
        Boot_JumpToApp(APP_A_ADDR);
    }

    /*
        APP_A 无效，停留 Bootloader。
        后续 v0.2 这里进入 CAN IAP。
    */
    while (1)
    {
        Boot_Delay(4000000);
    }
}