#include "ui_config.h"
#include "lv_port_oled_ssd1306.h"

#if UI_USE_LVGL

#include <string.h>
#include "lvgl.h"
#include "OLED.h"

static lv_disp_draw_buf_t s_draw_buf;
static lv_disp_t *s_disp;
static lv_color_t s_lv_buf[UI_OLED_HOR_RES * UI_OLED_VER_RES];
static uint8_t s_oled_frame[UI_OLED_HOR_RES * UI_OLED_PAGE_COUNT];

static uint8_t color_is_on(lv_color_t color)
{
    return (lv_color_brightness(color) > 127U) ? 1U : 0U;
}

static void oled_flush_cb(lv_disp_drv_t *disp_drv,
                          const lv_area_t *area,
                          lv_color_t *color_p)
{
    lv_coord_t x;
    lv_coord_t y;
    lv_coord_t width;

    memset(s_oled_frame, 0, sizeof(s_oled_frame));

    width = (lv_coord_t)(area->x2 - area->x1 + 1);

    for (y = area->y1; y <= area->y2; y++)
    {
        if ((y < 0) || (y >= (lv_coord_t)UI_OLED_VER_RES))
        {
            continue;
        }

        for (x = area->x1; x <= area->x2; x++)
        {
            lv_coord_t rel_x;
            lv_coord_t rel_y;
            lv_color_t color;

            if ((x < 0) || (x >= (lv_coord_t)UI_OLED_HOR_RES))
            {
                continue;
            }

            rel_x = (lv_coord_t)(x - area->x1);
            rel_y = (lv_coord_t)(y - area->y1);
            color = color_p[(rel_y * width) + rel_x];

            if (color_is_on(color) != 0U)
            {
                s_oled_frame[((uint16_t)y >> 3) * UI_OLED_HOR_RES + (uint16_t)x] |=
                    (uint8_t)(1U << ((uint16_t)y & 0x07U));
            }
        }
    }

    OLED_UpdateBuffer(s_oled_frame);
    lv_disp_flush_ready(disp_drv);
}

uint8_t LvPortOledSsd1306_Init(void)
{
    static lv_disp_drv_t disp_drv;

    if (s_disp != NULL)
    {
        return 1U;
    }

    lv_disp_draw_buf_init(&s_draw_buf,
                          s_lv_buf,
                          NULL,
                          UI_OLED_HOR_RES * UI_OLED_VER_RES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf = &s_draw_buf;
    disp_drv.hor_res = UI_OLED_HOR_RES;
    disp_drv.ver_res = UI_OLED_VER_RES;
    disp_drv.flush_cb = oled_flush_cb;
    disp_drv.full_refresh = 1;
    s_disp = lv_disp_drv_register(&disp_drv);

    return (s_disp != NULL) ? 1U : 0U;
}

#else

uint8_t LvPortOledSsd1306_Init(void)
{
    return 0U;
}

#endif
