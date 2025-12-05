#ifndef __WATCHDOG_H
#define __WATCHDOG_H

void IWDG_Init(uint16_t timeout_ms);

void IWDG_Feed(void);

#endif
