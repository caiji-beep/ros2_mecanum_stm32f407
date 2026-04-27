#ifndef __IMU_ICM20948_H
#define __IMU_ICM20948_H

#include "stm32f4xx.h"

#define IMU_dt 0.01f  // IMU任务周期，单位秒


/*原始数据*/
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} ICM20948_RawData_t;

/* 零偏（静止标定得到） */
typedef struct
{
    float ax_offset;
    float ay_offset;
    float az_offset;
    float gx_offset;
    float gy_offset;
    float gz_offset;
} ICM20948_Offset_t;
/*处理后的数据（物理量+姿态角） */
typedef struct
{
    /*线加速度，单位g */
    float ax_g;
    float ay_g;
    float az_g;
    /*角速度,单位rad/s */
    float gx_rads;
    float gy_rads;
    float gz_rads;
    /*估计姿态角，单位rad */
    float roll;  //滚转
    float pitch;  //俯仰
    float yaw;     //偏航
} ICM20948_ProcessedData_t;

typedef struct
{
    uint8_t seq;
    uint8_t calibrated;
    uint8_t data_valid;
    uint8_t recovering;
    uint8_t fault;
    uint8_t last_error;
    uint8_t consecutive_failures;
    uint32_t read_ok_count;
    uint32_t read_fail_count;
    uint32_t recovery_count;
    uint32_t last_update_ms;
} ICM20948_Status_t;


extern ICM20948_RawData_t      IMU_data;
extern ICM20948_Offset_t       IMU_offset;
extern ICM20948_ProcessedData_t IMU_processed;
extern volatile ICM20948_Status_t IMU_status;

extern int g_i2c_last_error;  // 最近一次错误的步骤编号
extern int g_i2c_error_count; // 错误总次数

void    ICM20948_I2C_HW_Init(void);      // PB10/PB11 I2C2 初始化
uint8_t ICM20948_Init(void);            // 传感器初始化，1=成功
uint8_t ICM20948_ReadData(ICM20948_RawData_t *data);   // 读取6轴原始数据

void ICM20948_Calibrate(ICM20948_Offset_t *off, uint16_t N);
void ICM20948_Process(ICM20948_RawData_t* rdata, ICM20948_Offset_t* off, ICM20948_ProcessedData_t* pdata,float dt);

void ICM20948_MarkCalibrated(uint8_t calibrated);
void ICM20948_StatusOnRead(uint8_t ok, uint32_t tick_ms);
void ICM20948_StatusOnRecovery(uint32_t tick_ms);
ICM20948_Status_t ICM20948_GetStatusSnapshot(void);

void I2C2_BusRecovery(void);

#endif
