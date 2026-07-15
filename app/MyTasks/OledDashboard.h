#ifndef __OLED_DASHBOARD_H
#define __OLED_DASHBOARD_H

#include <stdint.h>

typedef enum
{
    OLED_DASHBOARD_PAGE_SELF_CHECK = 0,
    OLED_DASHBOARD_PAGE_OTA,
    OLED_DASHBOARD_PAGE_CHASSIS,
    OLED_DASHBOARD_PAGE_FAULT,
    OLED_DASHBOARD_PAGE_COUNT
} OledDashboardPage_t;

typedef enum
{
    OLED_DASHBOARD_OTA_IDLE = 0,
    OLED_DASHBOARD_OTA_READY,
    OLED_DASHBOARD_OTA_RECEIVING,
    OLED_DASHBOARD_OTA_VERIFYING,
    OLED_DASHBOARD_OTA_DONE,
    OLED_DASHBOARD_OTA_ERROR
} OledDashboardOtaState_t;

void OledDashboard_Init(void);
void OledDashboard_NextPage(void);
void OledDashboard_SetPage(OledDashboardPage_t page);
OledDashboardPage_t OledDashboard_GetPage(void);
void OledDashboard_SetOtaState(OledDashboardOtaState_t state,
                               uint8_t progress_pct,
                               uint8_t error_code);
void OledDashboard_Render(void);
void OledDashboard_Process(void);
void OledDashboard_Tick1ms(void);

#endif
