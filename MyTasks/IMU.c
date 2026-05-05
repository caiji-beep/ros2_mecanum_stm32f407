#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "IMU.h"
#include "icm20948.h"
#include "imu_fusion.h"
#include "bsp_tick.h"
#include "bsp_exti.h"
#include "SPL_Delay.h"

#define IMU_GYRO_CALIB_SAMPLES 300U
#define IMU_GYRO_CALIB_WARMUP_SAMPLES 20U
#define IMU_GYRO_CALIB_DELAY_MS 5U
#define IMU_GYRO_DEADBAND_DPS 0.05f
#define DEG2RAD 0.01745329251994f

#define IMU_ERR_NONE 0U
#define IMU_ERR_INIT 1U
#define IMU_ERR_CALIB 2U
#define IMU_ERR_NOTIFY_TO 3U
#define IMU_ERR_INT_STATUS 4U
#define IMU_ERR_NOT_READY 5U
#define IMU_ERR_READ_DATA 6U

static Soft_I2C_Bus icm_i2c_bus = {
    .port = GPIOB,
    .scl_pin = GPIO_Pin_10,
    .sda_pin = GPIO_Pin_11,
};

static ICM20948_Data_t imu_raw;
static IMU_Fusion_t imu_fusion;
static IMU_State_t imu_state;
float dt;

static void IMU_PublishState(const IMU_State_t *state)
{
    taskENTER_CRITICAL();
    imu_state = *state;
    taskEXIT_CRITICAL();
}

uint8_t IMU_GetSnapshot(IMU_State_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = imu_state;
    taskEXIT_CRITICAL();

    return 1U;
}

static void IMU_RecordFailure(IMU_State_t *state, uint8_t err)
{
    state->data_valid = 0U;
    state->fault = 1U;
    state->last_error = err;
    if (state->consecutive_failures < 255U)
    {
        state->consecutive_failures++;
    }
    state->read_fail_count++;
    state->last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    IMU_PublishState(state);
}

static float IMU_ApplyGyroDeadband(float value_dps)
{
    if (value_dps > -IMU_GYRO_DEADBAND_DPS && value_dps < IMU_GYRO_DEADBAND_DPS)
    {
        return 0.0f;
    }

    return value_dps;
}

static uint8_t IMU_CalibrateGyroBias(float *gx_bias,
                                     float *gy_bias,
                                     float *gz_bias)
{
    ICM20948_Data_t sample;
    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;
    uint16_t valid = 0U;
    uint16_t i;

    if (gx_bias == NULL || gy_bias == NULL || gz_bias == NULL)
    {
        return 1U;
    }

    for (i = 0U; i < IMU_GYRO_CALIB_WARMUP_SAMPLES; i++)
    {
        (void)ICM20948_ReadData(&sample);
        SPL_Delay_ms(IMU_GYRO_CALIB_DELAY_MS);
    }

    for (i = 0U; i < IMU_GYRO_CALIB_SAMPLES; i++)
    {
        if (ICM20948_ReadData(&sample) == 0U)
        {
            gx_sum += sample.gyro_x;
            gy_sum += sample.gyro_y;
            gz_sum += sample.gyro_z;
            valid++;
        }

        SPL_Delay_ms(IMU_GYRO_CALIB_DELAY_MS);
    }

    if (valid < (IMU_GYRO_CALIB_SAMPLES / 2U))
    {
        return 1U;
    }

    *gx_bias = gx_sum / (float)valid;
    *gy_bias = gy_sum / (float)valid;
    *gz_bias = gz_sum / (float)valid;

    return 0U;
}

static uint8_t IMU_ReadAndUpdate(IMU_State_t *state, float *last_tick)
{
    uint8_t int_status = 0U;

    if (ICM20948_ReadIntStatus1(&int_status) != 0U)
    {
        IMU_RecordFailure(state, IMU_ERR_INT_STATUS);
        return 1U;
    }

    if ((int_status & 0x01U) == 0U)
    {
        IMU_RecordFailure(state, IMU_ERR_NOT_READY);
        return 1U;
    }

    if (ICM20948_ReadData(&imu_raw) != 0U)
    {
        IMU_RecordFailure(state, IMU_ERR_READ_DATA);
        return 1U;
    }

    {
        dt = BSP_Tick_GetDt(last_tick);

        /*
         * 目标 100Hz 左右。
         * 正常 dt 大约 0.009 ~ 0.011s。
         * 这里放宽一点，避免偶发调度抖动直接进入积分。
         */
        // if (dt < 0.005f || dt > 0.020f)
        // {
        //     return 1U;
        // }

        IMU_Fusion_Update(&imu_fusion,
                          imu_raw.gyro_x,
                          imu_raw.gyro_y,
                          imu_raw.gyro_z,
                          imu_raw.accel_x,
                          imu_raw.accel_y,
                          imu_raw.accel_z,
                          dt);
    }

    state->ax_g = imu_raw.accel_x;
    state->ay_g = imu_raw.accel_y;
    state->az_g = imu_raw.accel_z;
    state->gx_rads = IMU_ApplyGyroDeadband(imu_raw.gyro_x - imu_fusion.gyro_bias_x_dps) * DEG2RAD;
    state->gy_rads = IMU_ApplyGyroDeadband(imu_raw.gyro_y - imu_fusion.gyro_bias_y_dps) * DEG2RAD;
    state->gz_rads = IMU_ApplyGyroDeadband(imu_raw.gyro_z - imu_fusion.gyro_bias_z_dps) * DEG2RAD;

    IMU_Fusion_GetEuler(&imu_fusion,
                        &state->roll_deg,
                        &state->pitch_deg,
                        &state->yaw_deg);

    state->seq++;
    state->calibrated = 1U;
    state->data_valid = 1U;
    state->recovering = 0U;
    state->fault = 0U;
    state->last_error = IMU_ERR_NONE;
    state->consecutive_failures = 0U;
    state->read_ok_count++;
    state->last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    IMU_PublishState(state);

    return 0U;
}

void IMU_Task(void *pvParameters)
{
    IMU_State_t state = {0};
    float last_tick = 0.0f;
    float gx_bias = 0.0f;
    float gy_bias = 0.0f;
    float gz_bias = 0.0f;

    (void)pvParameters;

    BSP_Tick_Init();
    IMU_Fusion_Init(&imu_fusion);

    while (ICM20948_Init(&icm_i2c_bus) != 0U)
    {
        IMU_RecordFailure(&state, IMU_ERR_INIT);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    state.fault = 0U;
    state.last_error = IMU_ERR_NONE;
    state.consecutive_failures = 0U;
    state.last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    IMU_PublishState(&state);

    while (IMU_CalibrateGyroBias(&gx_bias, &gy_bias, &gz_bias) != 0U)
    {
        IMU_RecordFailure(&state, IMU_ERR_CALIB);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    IMU_Fusion_SetGyroBias(&imu_fusion, gx_bias, gy_bias, gz_bias);
    state.calibrated = 1U;
    state.data_valid = 0U;
    state.recovering = 0U;
    state.fault = 0U;
    state.last_error = IMU_ERR_NONE;
    state.consecutive_failures = 0U;
    state.last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    IMU_PublishState(&state);

    ICM20948_EXTI_Init();
    ICM20948_SyncIntAfterExtiInit();
    last_tick = (float)BSP_Tick_GetUs() / 1000000.0f;

    if (ICM20948_DataReadyFlag || ICM20948_EXTI_IsIntPinHigh())
    {
        ICM20948_DataReadyFlag = 0U;
        (void)IMU_ReadAndUpdate(&state, &last_tick);
    }

    for (;;)
    {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        if (notified > 0U)
        {
            (void)IMU_ReadAndUpdate(&state, &last_tick);
        }
        else if (ICM20948_EXTI_IsIntPinHigh())
        {
            (void)IMU_ReadAndUpdate(&state, &last_tick);
        }
        else
        {
            IMU_RecordFailure(&state, IMU_ERR_NOTIFY_TO);
        }
    }
}
