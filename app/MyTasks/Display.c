#include "stm32f4xx.h"
#include "freertos_demo.h"
#include "main.h"
#include "semphr.h"
#include "robot_state.h"
#include "OledDashboard.h"

extern QueueHandle_t gsem_OLED_handle;
extern uint8_t g_key1_pressed;

#define DISPLAY_ACTIVE_PERIOD_MS 110U
#define DISPLAY_IDLE_PERIOD_MS   1000U

void Display_Task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)pvParameters;
    OledDashboard_Init();

    while (1)
    {
        TickType_t period = pdMS_TO_TICKS(((g_robot_state == ROBOT_STATE_IDLE) ||
                                           (g_robot_state == ROBOT_STATE_ERROR)) ?
                                          DISPLAY_IDLE_PERIOD_MS :
                                          DISPLAY_ACTIVE_PERIOD_MS);

        vTaskDelayUntil(&last_wake, period);

        if (xSemaphoreTake(gsem_OLED_handle, portMAX_DELAY) == pdTRUE)
        {
            if (g_key1_pressed == 1U)
            {
                g_key1_pressed = 0U;
                OledDashboard_NextPage();
            }

            OledDashboard_Render();
            OledDashboard_Process();
            xSemaphoreGive(gsem_OLED_handle);
        }
    }
}
