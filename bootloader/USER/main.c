/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-14 12:04:15
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-15 09:59:37
 * @FilePath: \firmware\bootloader\USER\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f4xx.h"
#include "boot_main.h"

int main(void)
{
    Boot_Main();

    while (1)
    {
    }
}
