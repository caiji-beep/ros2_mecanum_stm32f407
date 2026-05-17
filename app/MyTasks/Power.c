#include "Power.h"
#include "FreeRTOS.h"
#include "task.h"
#include "buzzer.h"
#include "drv_power.h"
#include "robot_state.h"

#define POWER_SAMPLE_PERIOD_MS      211U
#define POWER_IDLE_SAMPLE_PERIOD_MS 1000U

#define POWER_WARN_ENTER_MV         9900U
#define POWER_WARN_EXIT_MV          10500U
#define POWER_CUTOFF_ENTER_MV       9600U
#define POWER_EMERGENCY_ENTER_MV    9300U
#define POWER_ADC_FAULT_MAX_MV      1000U

#define POWER_WARN_CONFIRM_COUNT    5U
#define POWER_CUTOFF_CONFIRM_COUNT  5U
#define POWER_EMERGENCY_COUNT       2U
#define POWER_ADC_FAULT_COUNT       3U

static volatile uint32_t s_power_vin_mv = 0U;
static volatile PowerStatus_t s_power_status = POWER_STATUS_NORMAL;

static uint8_t s_warn_count = 0U;
static uint8_t s_cutoff_count = 0U;
static uint8_t s_emergency_count = 0U;
static uint8_t s_adc_fault_count = 0U;
static uint8_t s_fault_latched = 0U;
static uint8_t s_alarm_phase = 0U;

static uint8_t count_up_to(uint8_t value, uint8_t max_value)
{
    if (value < max_value)
    {
        value++;
    }

    return value;
}

static void Power_SetFault(PowerStatus_t status)
{
    s_power_status = status;
    s_fault_latched = 1U;
    Robot_EnterState(ROBOT_STATE_ERROR);
}

static void Power_UpdateStatus(uint32_t vin_mv)
{
    if (vin_mv <= POWER_ADC_FAULT_MAX_MV)
    {
        s_adc_fault_count = count_up_to(s_adc_fault_count, POWER_ADC_FAULT_COUNT);
    }
    else
    {
        s_adc_fault_count = 0U;
    }

    if (vin_mv <= POWER_EMERGENCY_ENTER_MV)
    {
        s_emergency_count = count_up_to(s_emergency_count, POWER_EMERGENCY_COUNT);
    }
    else
    {
        s_emergency_count = 0U;
    }

    if (vin_mv <= POWER_CUTOFF_ENTER_MV)
    {
        s_cutoff_count = count_up_to(s_cutoff_count, POWER_CUTOFF_CONFIRM_COUNT);
    }
    else
    {
        s_cutoff_count = 0U;
    }

    if (vin_mv <= POWER_WARN_ENTER_MV)
    {
        s_warn_count = count_up_to(s_warn_count, POWER_WARN_CONFIRM_COUNT);
    }
    else if (vin_mv >= POWER_WARN_EXIT_MV)
    {
        s_warn_count = 0U;
    }

    if (s_fault_latched != 0U)
    {
        return;
    }

    if (s_adc_fault_count >= POWER_ADC_FAULT_COUNT)
    {
        Power_SetFault(POWER_STATUS_ADC_FAULT);
    }
    else if ((s_emergency_count >= POWER_EMERGENCY_COUNT) ||
             (s_cutoff_count >= POWER_CUTOFF_CONFIRM_COUNT))
    {
        Power_SetFault(POWER_STATUS_CUTOFF);
    }
    else if (s_warn_count >= POWER_WARN_CONFIRM_COUNT)
    {
        s_power_status = POWER_STATUS_WARN;
    }
    else
    {
        s_power_status = POWER_STATUS_NORMAL;
    }
}

static void Power_UpdateBuzzer(PowerStatus_t status)
{
    switch (status)
    {
    case POWER_STATUS_WARN:
        if (s_alarm_phase == 0U)
        {
            Buzzer_On();
        }
        else
        {
            Buzzer_Off();
        }
        s_alarm_phase = (uint8_t)((s_alarm_phase + 1U) % 10U);
        break;

    case POWER_STATUS_CUTOFF:
        if ((s_alarm_phase & 0x01U) == 0U)
        {
            Buzzer_On();
        }
        else
        {
            Buzzer_Off();
        }
        s_alarm_phase++;
        break;

    case POWER_STATUS_ADC_FAULT:
        if ((s_alarm_phase == 0U) || (s_alarm_phase == 2U) || (s_alarm_phase == 4U))
        {
            Buzzer_On();
        }
        else
        {
            Buzzer_Off();
        }
        s_alarm_phase = (uint8_t)((s_alarm_phase + 1U) % 8U);
        break;

    case POWER_STATUS_NORMAL:
    default:
        s_alarm_phase = 0U;
        Buzzer_Off();
        break;
    }
}

void Power_Task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        uint32_t sample_period_ms = (((g_robot_state == ROBOT_STATE_IDLE) ||
                                      (g_robot_state == ROBOT_STATE_ERROR)) &&
                                     (s_power_status == POWER_STATUS_NORMAL)) ?
                                    POWER_IDLE_SAMPLE_PERIOD_MS :
                                    POWER_SAMPLE_PERIOD_MS;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(sample_period_ms));

        s_power_vin_mv = DrvPower_GetVinMv();
        Power_UpdateStatus(s_power_vin_mv);
        Power_UpdateBuzzer(s_power_status);
    }
}

uint32_t Power_GetVinMv(void)
{
    return s_power_vin_mv;
}

PowerStatus_t Power_GetStatus(void)
{
    return s_power_status;
}
