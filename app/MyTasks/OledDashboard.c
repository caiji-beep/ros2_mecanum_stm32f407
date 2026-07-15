#include "OledDashboard.h"

#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Encoder.h"
#include "CTRL.h"
#include "IMU.h"
#include "Power.h"
#include "robot_state.h"
#include "ui_config.h"

#if UI_USE_LVGL
#include "lvgl.h"
#include "lv_port_oled_ssd1306.h"
#endif

typedef struct
{
    Robot_state robot_state;
    CtrlMode ctrl_mode;
    uint8_t nav_alive;
    uint32_t nav_age_ms;
    uint32_t vin_mv;
    PowerStatus_t power_status;
    IMU_State_t imu;
    uint8_t imu_snapshot_valid;
    float speed_mps[4];
    float target_mps[4];
    int pwm[4];
    OledDashboardOtaState_t ota_state;
    uint8_t ota_progress_pct;
    uint8_t ota_error_code;
} OledDashboardSnapshot_t;

static OledDashboardPage_t s_page = OLED_DASHBOARD_PAGE_SELF_CHECK;
static OledDashboardOtaState_t s_ota_state = OLED_DASHBOARD_OTA_IDLE;
static uint8_t s_ota_progress_pct = 0U;
static uint8_t s_ota_error_code = 0U;

#if UI_USE_LVGL
static lv_obj_t *s_title_label;
static lv_obj_t *s_line_label[3];
static lv_obj_t *s_ota_bar;
static volatile uint8_t s_lvgl_tick_enabled = 0U;
static uint8_t s_lvgl_ready = 0U;
#endif

static const char *robot_state_name(Robot_state state)
{
    switch (state)
    {
    case ROBOT_STATE_IDLE: return "IDLE";
    case ROBOT_STATE_TELEOP: return "TELE";
    case ROBOT_STATE_NAV: return "NAV";
    case ROBOT_STATE_PID_TEST: return "PID";
    case ROBOT_STATE_ERROR: return "ERR";
    default: return "UNK";
    }
}

static const char *ctrl_mode_name(CtrlMode mode)
{
    switch (mode)
    {
    case MODE_VEL: return "VEL";
    case MODE_MANUAL: return "MAN";
    default: return "UNK";
    }
}

static const char *power_status_name(PowerStatus_t status)
{
    switch (status)
    {
    case POWER_STATUS_NORMAL: return "OK";
    case POWER_STATUS_WARN: return "WARN";
    case POWER_STATUS_CUTOFF: return "CUT";
    case POWER_STATUS_ADC_FAULT: return "ADC";
    default: return "UNK";
    }
}

static const char *ota_state_name(OledDashboardOtaState_t state)
{
    switch (state)
    {
    case OLED_DASHBOARD_OTA_IDLE: return "IDLE";
    case OLED_DASHBOARD_OTA_READY: return "READY";
    case OLED_DASHBOARD_OTA_RECEIVING: return "RX";
    case OLED_DASHBOARD_OTA_VERIFYING: return "VERIFY";
    case OLED_DASHBOARD_OTA_DONE: return "DONE";
    case OLED_DASHBOARD_OTA_ERROR: return "ERR";
    default: return "UNK";
    }
}

static uint8_t snapshot_has_fault(const OledDashboardSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return 0U;
    }

    return (uint8_t)((snapshot->robot_state == ROBOT_STATE_ERROR) ||
                     (snapshot->power_status == POWER_STATUS_CUTOFF) ||
                     (snapshot->power_status == POWER_STATUS_ADC_FAULT) ||
                     ((snapshot->imu_snapshot_valid != 0U) && (snapshot->imu.fault != 0U)) ||
                     (snapshot->ota_state == OLED_DASHBOARD_OTA_ERROR));
}

static int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

#if UI_USE_LVGL
static int32_t centi_from_float(float value)
{
    int32_t sign = (value < 0.0f) ? -1 : 1;
    float abs_value = (value < 0.0f) ? -value : value;
    int32_t centi = (int32_t)(abs_value * 100.0f + 0.5f);

    if (centi > 999)
    {
        centi = 999;
    }

    return centi * sign;
}

static uint32_t abs_i32(int32_t value)
{
    return (uint32_t)((value < 0) ? -value : value);
}
#endif

static int max_abs4(int a, int b, int c, int d)
{
    int max_value = abs_int(a);
    int v = abs_int(b);
    if (v > max_value) max_value = v;
    v = abs_int(c);
    if (v > max_value) max_value = v;
    v = abs_int(d);
    if (v > max_value) max_value = v;
    return max_value;
}

static void build_snapshot(OledDashboardSnapshot_t *snapshot)
{
    TickType_t now;

    if (snapshot == NULL)
    {
        return;
    }

    snapshot->robot_state = g_robot_state;
    snapshot->ctrl_mode = g_mode;
    snapshot->nav_alive = g_nav_cmd_alive;
    now = xTaskGetTickCount();
    snapshot->nav_age_ms = (uint32_t)((now - g_nav_last_rx_tick) * portTICK_PERIOD_MS);
    snapshot->vin_mv = Power_GetVinMv();
    snapshot->power_status = Power_GetStatus();
    snapshot->imu_snapshot_valid = IMU_GetSnapshot(&snapshot->imu);

    snapshot->speed_mps[0] = Encoder_Speed_mps(ENC_A);
    snapshot->speed_mps[1] = Encoder_Speed_mps(ENC_B);
    snapshot->speed_mps[2] = Encoder_Speed_mps(ENC_C);
    snapshot->speed_mps[3] = Encoder_Speed_mps(ENC_D);

    snapshot->target_mps[0] = SC_GetTarget_mps(ENC_A);
    snapshot->target_mps[1] = SC_GetTarget_mps(ENC_B);
    snapshot->target_mps[2] = SC_GetTarget_mps(ENC_C);
    snapshot->target_mps[3] = SC_GetTarget_mps(ENC_D);

    snapshot->pwm[0] = SC_GetPWM(ENC_A);
    snapshot->pwm[1] = SC_GetPWM(ENC_B);
    snapshot->pwm[2] = SC_GetPWM(ENC_C);
    snapshot->pwm[3] = SC_GetPWM(ENC_D);

    snapshot->ota_state = s_ota_state;
    snapshot->ota_progress_pct = s_ota_progress_pct;
    snapshot->ota_error_code = s_ota_error_code;
}

#if UI_USE_LVGL
static uint8_t init_lvgl_objects(void)
{
    uint8_t i;
    lv_obj_t *screen = lv_scr_act();

    if (screen == NULL)
    {
        return 0U;
    }

    lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    s_title_label = lv_label_create(screen);
    if (s_title_label == NULL)
    {
        return 0U;
    }
    lv_obj_set_style_text_color(s_title_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_title_label, LV_FONT_DEFAULT, LV_PART_MAIN);
    lv_obj_align(s_title_label, LV_ALIGN_TOP_LEFT, 0, 0);

    for (i = 0U; i < 3U; i++)
    {
        s_line_label[i] = lv_label_create(screen);
        if (s_line_label[i] == NULL)
        {
            return 0U;
        }
        lv_obj_set_style_text_color(s_line_label[i], lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(s_line_label[i], LV_FONT_DEFAULT, LV_PART_MAIN);
        lv_obj_align(s_line_label[i], LV_ALIGN_TOP_LEFT, 0, (lv_coord_t)(16 + (i * 16)));
    }

    s_ota_bar = lv_bar_create(screen);
    if (s_ota_bar == NULL)
    {
        return 0U;
    }
    lv_obj_set_size(s_ota_bar, 120, 6);
    lv_obj_align(s_ota_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_radius(s_ota_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_ota_bar, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_ota_bar, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_color(s_ota_bar, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_ota_bar, 1, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ota_bar, lv_color_white(), LV_PART_INDICATOR);
    lv_bar_set_range(s_ota_bar, 0, 100);
    lv_obj_add_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);

    return 1U;
}

static void render_lvgl(const OledDashboardSnapshot_t *snapshot)
{
    switch (s_page)
    {
    case OLED_DASHBOARD_PAGE_SELF_CHECK:
        lv_obj_add_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text_fmt(s_title_label, "SELF 1/4 %02lu.%luV",
                              (unsigned long)(snapshot->vin_mv / 1000U),
                              (unsigned long)((snapshot->vin_mv % 1000U) / 100U));
        lv_label_set_text_fmt(s_line_label[0], "PWR:%s IMU:%s",
                              power_status_name(snapshot->power_status),
                              (snapshot->imu_snapshot_valid == 0U) ? "NA" :
                              ((snapshot->imu.fault != 0U) ? "ER" :
                               ((snapshot->imu.calibrated == 0U) ? "CA" : "OK")));
        lv_label_set_text_fmt(s_line_label[1], "NAV:%s ENC:OK",
                              snapshot->nav_alive ? "OK" : "WAIT");
        lv_label_set_text_fmt(s_line_label[2], "SYS:%s",
                              snapshot_has_fault(snapshot) ? "FAULT" : "READY");
        break;

    case OLED_DASHBOARD_PAGE_OTA:
        lv_obj_clear_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(s_ota_bar, snapshot->ota_progress_pct, LV_ANIM_OFF);
        lv_label_set_text_fmt(s_title_label, "OTA 2/4 %s",
                              ota_state_name(snapshot->ota_state));
        lv_label_set_text(s_line_label[0], "Boot:v0.1");
        lv_label_set_text_fmt(s_line_label[1], "CAN IAP:%s",
                              (snapshot->ota_state == OLED_DASHBOARD_OTA_IDLE) ? "WAIT" : "RUN");
        lv_label_set_text_fmt(s_line_label[2], "P:%03u%% E:%03u",
                              (unsigned int)snapshot->ota_progress_pct,
                              (unsigned int)snapshot->ota_error_code);
        break;

    case OLED_DASHBOARD_PAGE_CHASSIS:
    {
        int32_t a = centi_from_float(snapshot->speed_mps[0]);
        int32_t b = centi_from_float(snapshot->speed_mps[1]);
        int32_t c = centi_from_float(snapshot->speed_mps[2]);
        int32_t d = centi_from_float(snapshot->speed_mps[3]);

        lv_obj_add_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text_fmt(s_title_label, "BASE 3/4 %s %s",
                              robot_state_name(snapshot->robot_state),
                              ctrl_mode_name(snapshot->ctrl_mode));
        lv_label_set_text_fmt(s_line_label[0], "A%c%lu.%02lu B%c%lu.%02lu",
                              (a < 0) ? '-' : '+',
                              (unsigned long)(abs_i32(a) / 100U),
                              (unsigned long)(abs_i32(a) % 100U),
                              (b < 0) ? '-' : '+',
                              (unsigned long)(abs_i32(b) / 100U),
                              (unsigned long)(abs_i32(b) % 100U));
        lv_label_set_text_fmt(s_line_label[1], "C%c%lu.%02lu D%c%lu.%02lu",
                              (c < 0) ? '-' : '+',
                              (unsigned long)(abs_i32(c) / 100U),
                              (unsigned long)(abs_i32(c) % 100U),
                              (d < 0) ? '-' : '+',
                              (unsigned long)(abs_i32(d) / 100U),
                              (unsigned long)(abs_i32(d) % 100U));
        lv_label_set_text_fmt(s_line_label[2], "PWM:%05d",
                              max_abs4(snapshot->pwm[0], snapshot->pwm[1],
                                       snapshot->pwm[2], snapshot->pwm[3]));
        break;
    }

    case OLED_DASHBOARD_PAGE_FAULT:
    default:
        lv_obj_add_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text_fmt(s_title_label, "FAULT 4/4 %s",
                              snapshot_has_fault(snapshot) ? "ERR" : "NONE");
        lv_label_set_text_fmt(s_line_label[0], "PWR:%s VIN:%02lu",
                              power_status_name(snapshot->power_status),
                              (unsigned long)(snapshot->vin_mv / 1000U));
        lv_label_set_text_fmt(s_line_label[1], "IMU:E%02u F%03u",
                              (unsigned int)(snapshot->imu_snapshot_valid ? snapshot->imu.last_error : 0U),
                              (unsigned int)(snapshot->imu_snapshot_valid ? snapshot->imu.consecutive_failures : 0U));
        lv_label_set_text_fmt(s_line_label[2], "NAV:%s S:%s",
                              snapshot->nav_alive ? ((snapshot->nav_age_ms > 300U) ? "TO" : "OK") : "WAIT",
                              robot_state_name(snapshot->robot_state));
        break;
    }
}
#endif

void OledDashboard_Init(void)
{
#if UI_USE_LVGL
    lv_init();
    if ((LvPortOledSsd1306_Init() != 0U) && (init_lvgl_objects() != 0U))
    {
        s_lvgl_ready = 1U;
        s_lvgl_tick_enabled = 1U;
    }
    else
    {
        s_lvgl_ready = 0U;
        s_lvgl_tick_enabled = 0U;
    }
#endif
}

void OledDashboard_NextPage(void)
{
    s_page = (OledDashboardPage_t)(((uint8_t)s_page + 1U) % (uint8_t)OLED_DASHBOARD_PAGE_COUNT);
}

void OledDashboard_SetPage(OledDashboardPage_t page)
{
    if ((uint8_t)page < (uint8_t)OLED_DASHBOARD_PAGE_COUNT)
    {
        s_page = page;
    }
}

OledDashboardPage_t OledDashboard_GetPage(void)
{
    return s_page;
}

void OledDashboard_SetOtaState(OledDashboardOtaState_t state,
                               uint8_t progress_pct,
                               uint8_t error_code)
{
    if ((uint8_t)state <= (uint8_t)OLED_DASHBOARD_OTA_ERROR)
    {
        s_ota_state = state;
    }
    if (progress_pct > 100U)
    {
        progress_pct = 100U;
    }
    s_ota_progress_pct = progress_pct;
    s_ota_error_code = error_code;
}

void OledDashboard_Render(void)
{
#if UI_USE_LVGL
    OledDashboardSnapshot_t snapshot;

    build_snapshot(&snapshot);

    if (s_lvgl_ready != 0U)
    {
        render_lvgl(&snapshot);
    }
#endif
}

void OledDashboard_Process(void)
{
#if UI_USE_LVGL
    if (s_lvgl_ready != 0U)
    {
        lv_timer_handler();
    }
#endif
}

void OledDashboard_Tick1ms(void)
{
#if UI_USE_LVGL
    if (s_lvgl_tick_enabled != 0U)
    {
        lv_tick_inc(1);
    }
#endif
}
