#include "freertos_demo.h"
#include "FreeRTOS.h" //FreeRTOS头文件
#include "task.h"     //FreeRTOS任务头文件
#include "queue.h"    //FreeRTOS队列头文件
#include "semphr.h"
/*需要用的其他头文件*/
#include <stdio.h>
#include "stm32f4xx.h"
#include "OLED.h"
#include "SPL_Delay.h"
#include "PWM.h"
#include "Uart.h"
#include "Encoder.h"
#include "Timer.h"
#include "CTRL.h"
#include "watchdog.h"
#include "buzzer.h"
#include "Usartx.h"
#include "Can.h"
#include "Key.h"
#include "LED.h"
#include "drv_power.h"
#include "Power.h"

/*启动任务的配置*/
#define START_TASK_STK_SIZE 128 // 启动任务栈大小
#define START_TASK_PRIO 1       // 启动任务优先级
TaskHandle_t start_task_Handle; // 启动任务句柄
void start_task(void *pvParameters);

/*CTRL_TASK的配置*/
#define CTRL_STK_SIZE 512
#define CTRL_PRIO 4
TaskHandle_t gCtrlTask_Handle;
void Ctrl_Task(void *pvParameters);

/*UART_TASK的配置*/
#define UART_STK_SIZE 512
#define UART_PRIO 3
TaskHandle_t gUartTask_Handle;
void Uart_Task(void *pvParameters);

/*DISPLAY_TASK的配置*/
#define DISPLAY_STK_SIZE 512
#define DISPLAY_PRIO 2
TaskHandle_t gDisplayTask_Handle;
void Display_Task(void *pvParameters);

/*TELEMETRY_TASK的配置*/
#define TELEMETRY_STK_SIZE 512
#define TELEMETRY_PRIO 2
TaskHandle_t gTelemetryTask_Handle;
void Telemetry_Task(void *pvParameters);

/*IMU_TASK的配置*/
#define IMU_STK_SIZE 512
#define IMU_PRIO 2
TaskHandle_t gIMUTask_Handle;
void IMU_Task(void *pvParameters);

#define POWER_STK_SIZE 256
#define POWER_PRIO 2
TaskHandle_t gPowerTask_Handle;

/*队列句柄*/
QueueHandle_t gUart2RxQ = NULL;
QueueHandle_t gUart3RxQ = NULL;

/*队列集合句柄*/
QueueSetHandle_t gqueueset_handle;

/*OLED句柄*/
QueueHandle_t gsem_OLED_handle;

uint8_t g_key1_pressed = 0;

static Soft_I2C_Bus oled_i2c_bus = {
    .port = GPIOD,
    .scl_pin = GPIO_Pin_14,
    .sda_pin = GPIO_Pin_13};

static void BoardInit(void)
{
    LED_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    Buzzer_Init();
    Key_Init();
    TIM6_Init();
    TIM7_Init();

    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST))
    {
        // 上次是看门狗复位
        Buzzer_Pattern_WatchdogReset();
        RCC_ClearFlag(); // 清除复位标志
    }
    else
    {
        // 正常上电复位
        Buzzer_Beep(500);
    }
    OLED_Init(&oled_i2c_bus);

    /* PWM定时器（Serial_Init 必须在定时器后） */
    TIM1_PWM_Init(16800 - 1, 1 - 1);
    TIM9_PWM_Init(16800 - 1, 1 - 1);
    TIM10_PWM_Init(16800 - 1, 1 - 1);
    TIM11_PWM_Init(16800 - 1, 1 - 1);

    Serial2_Init(); // 串口2
    Serial3_Init(); // 串口3
    CAN1_Init();
    DrvPower_Init();

    Encoder_Init();

    Encoder_SetDir(ENC_A, +1);
    Encoder_SetDir(ENC_B, -1);
    Encoder_SetDir(ENC_C, -1);
    Encoder_SetDir(ENC_D, +1);

    SC_Init(); // 速度环参数/状态
    // ICM20948_Init();//放在Encoder_Init之前初始化，会导致 D轮抽搐一下
    OLED_Clear();
    IWDG_Init(1000); // 看门狗初始化，1秒
}

/**
 * @description: 启动FreeRTOS
 * @return {*}
 */
void freertos_start(void)
{
    BoardInit(); // 初始化硬件
    /*1.创建一个启动任务*/
    xTaskCreate((TaskFunction_t)start_task,                  // 任务函数的地址
                (char *)"start_task",                        // 任务名称
                (configSTACK_DEPTH_TYPE)START_TASK_STK_SIZE, // 任务栈大小，默认最小128，单位4字节
                (void *)NULL,                                // 传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,                // 任务优先级，数值越大，优先级越高
                (TaskHandle_t *)&start_task_Handle);         // 任务句柄的地址

    /*2.启动调度器：会自动创建空闲任务*/
    vTaskStartScheduler();
}

/**
 * @description: 启动任务:用来创建其他task
 * @param {void*} pvParameters
 * @return {*}
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); // 进入临界区

    gUart2RxQ = xQueueCreate(128, sizeof(uint8_t));
    gUart3RxQ = xQueueCreate(256, sizeof(uint8_t));
    configASSERT(gUart2RxQ && gUart3RxQ);
    gqueueset_handle = xQueueCreateSet(128 + 256);
    xQueueAddToSet(gUart2RxQ, gqueueset_handle);
    xQueueAddToSet(gUart3RxQ, gqueueset_handle);
    vSemaphoreCreateBinary(gsem_OLED_handle); // 会主动释放一次信号量
    configASSERT(gsem_OLED_handle);

    xTaskCreate((TaskFunction_t)Ctrl_Task,
                (char *)"Ctrl_Task",
                (configSTACK_DEPTH_TYPE)CTRL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CTRL_PRIO,
                (TaskHandle_t *)&gCtrlTask_Handle);

    xTaskCreate((TaskFunction_t)Uart_Task,
                (char *)"Uart_Task",
                (configSTACK_DEPTH_TYPE)UART_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)UART_PRIO,
                (TaskHandle_t *)&gUartTask_Handle);

    xTaskCreate((TaskFunction_t)Display_Task,
                (char *)"Display_Task",
                (configSTACK_DEPTH_TYPE)DISPLAY_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)DISPLAY_PRIO,
                (TaskHandle_t *)&gDisplayTask_Handle);

    xTaskCreate((TaskFunction_t)Telemetry_Task,
                (char *)"Telemetry_Task",
                (configSTACK_DEPTH_TYPE)TELEMETRY_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TELEMETRY_PRIO,
                (TaskHandle_t *)&gTelemetryTask_Handle);

    xTaskCreate((TaskFunction_t)IMU_Task,
                (char *)"IMU_Task",
                (configSTACK_DEPTH_TYPE)IMU_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)IMU_PRIO,
                (TaskHandle_t *)&gIMUTask_Handle);

    xTaskCreate((TaskFunction_t)Power_Task,
                (char *)"Power_Task",
                (configSTACK_DEPTH_TYPE)POWER_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)POWER_PRIO,
                (TaskHandle_t *)&gPowerTask_Handle);

    taskEXIT_CRITICAL(); // 退出临界区
    /*启动任务只需要执行一次即可，用完删除*/
    vTaskDelete(NULL); // 删除启动任务自身
}

void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        static TickType_t last_tick = 0;
        TickType_t now = xTaskGetTickCountFromISR();

        // 简单防抖：50ms 内忽略后续中断
        if ((now - last_tick) > pdMS_TO_TICKS(50))
        {
            g_key1_pressed = 1; // 标记“有一次按键”
            last_tick = now;
        }

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/**
 * @brief 栈溢出钩子函数
 *
 * @param xTask
 * @param pcTaskName
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS(); // 全局关闭中断
    Set_Pwm(0, 0, 0, 0);
    for (;;) // 让程序彻底卡死在这里等待复位
    {
    }
}

/**
 * @brief 内存分配失败钩子函数
 *
 */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    Set_Pwm(0, 0, 0, 0);
    for (;;)
    {
    }
}

void vApplicationIdleHook(void)
{
    __DSB();
    __WFI();
    __ISB();
}
