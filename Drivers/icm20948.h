#ifndef __ICM20948_H
#define __ICM20948_H

#include "stm32f4xx.h"
#include "bsp_soft_i2c.h"


/* ================= 核心地址与常量 ================= */
#define ICM20948_ADDR           0x68    // AD0引脚接GND时的I2C地址
#define ICM20948_ID             0xEA    // WHO_AM_I 预期返回值

/* ================= 通用寄存器 (ALL BANKS) ========= */
#define REG_BANK_SEL            0x7F    // Bank切换寄存器

/* ================= BANK 0 寄存器 ================== */
#define REG_WHO_AM_I            0x00
#define REG_USER_CTRL           0x03    // 用户控制(I2C主模式、复位等)
#define REG_PWR_MGMT_1          0x06    // 电源管理1 (唤醒、时钟源)
#define REG_PWR_MGMT_2          0x07    // 电源管理2 (开启/关闭传感器)
#define REG_INT_PIN_CFG         0x0F    // 中断引脚配置
#define REG_INT_ENABLE_1        0x11
#define REG_INT_STATUS_1        0x1A
#define REG_INT_STATUS        0x19   
// 数据寄存器起始地址 (一口气读14字节)
#define REG_ACCEL_XOUT_H        0x2D

/* INT_PIN_CFG values used by this driver. */
#define ICM20948_INT1_ACTIVE_HIGH_PP_PULSE 0x00
#define ICM20948_INT1_ACTIVE_HIGH_PP_LATCH 0x20

/* ================= BANK 2 寄存器 ================== */
#define REG_GYRO_CONFIG_1       0x01    // 陀螺仪量程与DLPF
#define REG_ACCEL_CONFIG        0x14    // 加速度计量程与DLPF

#define REG_GYRO_SMPLRT_DIV       0x00    // Bank 2
#define REG_ACCEL_SMPLRT_DIV_1    0x10    // Bank 2
#define REG_ACCEL_SMPLRT_DIV_2    0x11    // Bank 2

#define IMU_SAMPLE_DIV_100HZ      10      // 1125 / (1 + 10) ≈ 102.27Hz

/* ================= 配置参数宏 ================= */
// Bank 选择
#define BANK_0                  0x00
#define BANK_1                  0x10
#define BANK_2                  0x20
#define BANK_3                  0x30

// 时钟与电源
#define CLK_BEST_PLL            0x01    // 选择自动最佳时钟源
#define WAKE_UP                 0x00    // 解除睡眠

// 陀螺仪配置：开启DLPF(bit0=1), 量程±500dps(bit2:1=01), 滤波51.2Hz(bit5:3=011)
// 组合值: 0001 1011 = 0x1B
#define GYRO_CFG_500DPS_50HZ   0x1B    

// 加速度计配置：开启DLPF(bit0=1), 量程±2g(bit2:1=00), 滤波50.4Hz(bit5:3=011)
// 组合值: 0001 1001 = 0x19
#define ACCEL_CFG_2G_50HZ       0x19

typedef struct
{
    float accel_x, accel_y, accel_z; // g
    float gyro_x, gyro_y, gyro_z;    // dps
    float mag_x, mag_y, mag_z;       // uT
    float temp;                      // degC
}ICM20948_Data_t;

extern volatile uint8_t ICM20948_DataReadyFlag;

/* ================= 外部调用的 API ================= */
uint8_t ICM20948_Init(Soft_I2C_Bus *bus);
uint8_t ICM20948_ReadData(ICM20948_Data_t *data);
uint8_t ICM20948_ReadIntStatus1(uint8_t *status);
void ICM20948_SyncIntAfterExtiInit(void);

#endif
