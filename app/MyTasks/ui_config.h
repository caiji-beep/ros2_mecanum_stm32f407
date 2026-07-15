#ifndef __UI_CONFIG_H
#define __UI_CONFIG_H

#define UI_OLED_HOR_RES 128U
#define UI_OLED_VER_RES 64U
#define UI_OLED_PAGE_COUNT (UI_OLED_VER_RES / 8U)

#ifndef UI_USE_LVGL
#define UI_USE_LVGL 1
#endif

#endif
