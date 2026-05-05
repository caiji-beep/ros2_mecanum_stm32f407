// /*
//  * @Author: caiji-beep 2978115384@qq.com
//  * @Date: 2025-12-01 19:16:33
//  * @LastEditors: caiji-beep 2978115384@qq.com
//  * @LastEditTime: 2025-12-03 22:44:53
//  * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\HARDWARE\IMU_Icm20948.c
//  */

// #include "stm32f4xx.h"
// #include "IMU_Icm20948.h"
// #include "stm32f4xx_gpio.h"
// #include "stm32f4xx_rcc.h"
// #include "stm32f4xx_i2c.h"
// #include "Usartx.h"
// #include "SPL_Delay.h"
// #include <math.h>

// /* ================== ICM20948 寄存器及地址 ================== */

// #define ICM20948_I2C_ADDR_7BIT 0x68
// #define ICM20948_I2C_ADDR (ICM20948_I2C_ADDR_7BIT << 1)

// #define ICM20948_REG_WHO_AM_I 0x00
// #define ICM20948_REG_PWR_MGMT_1 0x06
// #define ICM20948_REG_PWR_MGMT_2 0x07
// #define ICM20948_REG_BANK_SEL 0x7F

// #define ICM20948_REG_ACCEL_XOUT_H 0x2D
// #define ICM20948_REG_GYRO_XOUT_H 0x33

// #define ICM20948_WHO_AM_I_VALUE 0xEA

// /* ================== 全局数据 ================== */
// ICM20948_RawData_t IMU_data;
// ICM20948_Offset_t IMU_offset;
// ICM20948_ProcessedData_t IMU_processed;
// volatile ICM20948_Status_t IMU_status;
// extern int g_i2c_last_error;

// void ICM20948_MarkCalibrated(uint8_t calibrated)
// {
//     IMU_status.calibrated = calibrated ? 1U : 0U;
// }

// void ICM20948_StatusOnRead(uint8_t ok, uint32_t tick_ms)
// {
//     IMU_status.last_update_ms = tick_ms;
//     IMU_status.last_error = ok ? 0U : (uint8_t)g_i2c_last_error;

//     if (ok)
//     {
//         IMU_status.seq++;
//         IMU_status.data_valid = 1U;
//         IMU_status.recovering = 0U;
//         IMU_status.fault = 0U;
//         IMU_status.consecutive_failures = 0U;
//         IMU_status.read_ok_count++;
//     }
//     else
//     {
//         IMU_status.data_valid = 0U;
//         IMU_status.read_fail_count++;
//         if (IMU_status.consecutive_failures < 255U)
//         {
//             IMU_status.consecutive_failures++;
//         }
//         IMU_status.fault = (IMU_status.consecutive_failures >= 5U) ? 1U : 0U;
//     }
// }

// void ICM20948_StatusOnRecovery(uint32_t tick_ms)
// {
//     IMU_status.recovering = 1U;
//     IMU_status.recovery_count++;
//     IMU_status.last_update_ms = tick_ms;
// }

// ICM20948_Status_t ICM20948_GetStatusSnapshot(void)
// {
//     ICM20948_Status_t s;
//     s.seq = IMU_status.seq;
//     s.calibrated = IMU_status.calibrated;
//     s.data_valid = IMU_status.data_valid;
//     s.recovering = IMU_status.recovering;
//     s.fault = IMU_status.fault;
//     s.last_error = IMU_status.last_error;
//     s.consecutive_failures = IMU_status.consecutive_failures;
//     s.read_ok_count = IMU_status.read_ok_count;
//     s.read_fail_count = IMU_status.read_fail_count;
//     s.recovery_count = IMU_status.recovery_count;
//     s.last_update_ms = IMU_status.last_update_ms;
//     return s;
// }

// int g_i2c_last_error = 0;  // 最近一次错误的步骤编号
// int g_i2c_error_count = 0; // 错误总次数

// /* ================== I2C 超时辅助函数 ================== */

// /* 这个值根据主频和 I2C 速率调，先给一个比较大的 */
// #define I2C2_TIMEOUT ((uint32_t)100000)

// /* 等待 BUSY 清零，带超时，返回 0=OK, -1=超时 */
// static int I2C2_WaitBusyTimeout(void)
// {
//     uint32_t timeout = I2C2_TIMEOUT;
//     while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
//     {
//         if (--timeout == 0)
//         {
//             g_i2c_last_error = 1; // 1 = BUSY 一直清不掉
//             g_i2c_error_count++;
//             return -1;
//         }
//     }
//     return 0;
// }

// /* 等待指定 Event 置位，返回 0=OK, -1=超时 */
// static int I2C2_WaitEvent(uint32_t event)
// {
//     uint32_t timeout = I2C2_TIMEOUT;
//     while (!I2C_CheckEvent(I2C2, event))
//     {
//         if (--timeout == 0)
//         {
//             g_i2c_last_error = 2; // 2 = 某个 Event 等不到
//             g_i2c_error_count++;
//             return -1;
//         }
        
//     }
//     return 0;
// }

// /* 等待接收一个字节（BYTE_RECEIVED），返回 0=OK, -1=超时 */
// static int I2C2_WaitByteReceived(void)
// {
//     uint32_t timeout = I2C2_TIMEOUT;
//     while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
//     {
//         if (--timeout == 0)
//         {
//             g_i2c_last_error = 3;   // 3 = 等字节超时
//             g_i2c_error_count++;
//             return -1;
//         }
            
//     }
//     return 0;
// }

// void I2C2_BusRecovery(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
//     int i;

//     /* 1. 先关掉 I2C2 外设 */
//     I2C_Cmd(I2C2, DISABLE);
//     I2C_DeInit(I2C2);

//     /* 2. 把 PB10(SCL), PB11(SDA) 配成 OD 输出，上拉 */
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//     GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
//     GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);

//     /* 3. 先都拉高，释放总线 */
//     GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);

//     /* 4. 打 9 个 SCL 脉冲，试图把从机的状态机“时钟走完” */
//     for (i = 0; i < 9; i++)
//     {
//         // SCL 低
//         GPIO_ResetBits(GPIOB, GPIO_Pin_10);
//         SPL_Delay_us(5);  // 你已有的微秒延时，比如 SPL_Delay 里

//         // SCL 高
//         GPIO_SetBits(GPIOB, GPIO_Pin_10);
//         SPL_Delay_us(5);
//     }

//     /* 5. 产生一个 STOP 条件：在 SCL 高的时候，让 SDA 从 0 -> 1 */
//     GPIO_ResetBits(GPIOB, GPIO_Pin_11);  // SDA 先拉低
//     SPL_Delay_us(5);
//     GPIO_SetBits(GPIOB, GPIO_Pin_11);    // 再拉高，STOP
//     SPL_Delay_us(5);

//     /* 6. 引脚切回 I2C2 复用，并重新初始化 I2C2 */
//     ICM20948_I2C_HW_Init();  // 你自己的初始化函数：配置 AF + I2C_Init + I2C_Cmd
// }


// /* ================== I2C2 硬件初始化 ================== */

// void ICM20948_I2C_HW_Init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
//     I2C_InitTypeDef I2C_InitStructure;

//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

//     /* PB10 SCL, PB11 SDA 复用为 I2C2 */
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);

//     GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
//     GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

//     I2C_DeInit(I2C2);
//     I2C_InitStructure.I2C_ClockSpeed = 100000; /* 100kHz */
//     I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//     I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//     I2C_InitStructure.I2C_OwnAddress1 = 0x00;
//     I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//     I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//     I2C_Init(I2C2, &I2C_InitStructure);

//     I2C_Cmd(I2C2, ENABLE);
// }

// /* ================== I2C2 基础工具函数（带超时） ================== */

// /* 写一个寄存器：返回 0=成功，-1=失败 */
// static int I2C2_WriteByte(uint8_t reg, uint8_t data)
// {
//     if (I2C2_WaitBusyTimeout() < 0)
//         return -1;

//     /* 1. START */
//     I2C_GenerateSTART(I2C2, ENABLE);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) < 0)
//         goto error;

//     /* 2. 发送地址 + 写 */
//     I2C_Send7bitAddress(I2C2, ICM20948_I2C_ADDR, I2C_Direction_Transmitter);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) < 0)
//         goto error;

//     /* 3. 发送寄存器地址 */
//     I2C_SendData(I2C2, reg);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) < 0)
//         goto error;

//     /* 4. 发送数据 */
//     I2C_SendData(I2C2, data);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) < 0)
//         goto error;

//     /* 5. STOP */
//     I2C_GenerateSTOP(I2C2, ENABLE);
//     return 0;

// error:
//     I2C_GenerateSTOP(I2C2, ENABLE);
//     return -1;
// }

// /* 读一个寄存器：返回 0=成功，-1=失败 */
// static int I2C2_ReadByte(uint8_t reg, uint8_t *pval)
// {
//     if (pval == NULL)
//         return -1;

//     if (I2C2_WaitBusyTimeout() < 0)
//         return -1;

//     /* 1. 先写寄存器地址（写模式） */
//     I2C_GenerateSTART(I2C2, ENABLE);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) < 0)
//         goto error;

//     I2C_Send7bitAddress(I2C2, ICM20948_I2C_ADDR, I2C_Direction_Transmitter);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) < 0)
//         goto error;

//     I2C_SendData(I2C2, reg);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) < 0)
//         goto error;

//     /* 2. 重新 START，切换读模式 */
//     I2C_GenerateSTART(I2C2, ENABLE);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) < 0)
//         goto error;

//     I2C_Send7bitAddress(I2C2, ICM20948_I2C_ADDR, I2C_Direction_Receiver);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) < 0)
//         goto error;

//     /* 3. 只读 1 字节：提前关闭 ACK */
//     I2C_AcknowledgeConfig(I2C2, DISABLE);
//     if (I2C2_WaitByteReceived() < 0)
//         goto error_ack;

//     *pval = I2C_ReceiveData(I2C2);
//     I2C_GenerateSTOP(I2C2, ENABLE);
//     I2C_AcknowledgeConfig(I2C2, ENABLE);
//     return 0;

// error_ack:
//     I2C_GenerateSTOP(I2C2, ENABLE);
//     I2C_AcknowledgeConfig(I2C2, ENABLE);
//     return -1;

// error:
//     I2C_GenerateSTOP(I2C2, ENABLE);
//     return -1;
// }

// /* 连续读取 len 字节：返回 0=成功，-1=失败 */
// static int I2C2_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
// {
//     if (buf == NULL || len == 0)
//         return -1;

//     if (I2C2_WaitBusyTimeout() < 0)
//         return -1;

//     /* 1. 先写寄存器地址（写模式） */
//     I2C_GenerateSTART(I2C2, ENABLE);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) < 0)
//         goto error;

//     I2C_Send7bitAddress(I2C2, ICM20948_I2C_ADDR, I2C_Direction_Transmitter);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) < 0)
//         goto error;

//     I2C_SendData(I2C2, reg);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) < 0)
//         goto error;

//     /* 2. 重新 START，读模式 */
//     I2C_GenerateSTART(I2C2, ENABLE);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) < 0)
//         goto error;

//     I2C_Send7bitAddress(I2C2, ICM20948_I2C_ADDR, I2C_Direction_Receiver);
//     if (I2C2_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) < 0)
//         goto error;

//     /* 3. 循环读 len 个字节 */
//     for (uint8_t i = 0; i < len; ++i)
//     {
//         if (i == (len - 1))
//         {
//             /* 最后一个字节前关闭 ACK，并准备 STOP */
//             I2C_AcknowledgeConfig(I2C2, DISABLE);
//         }

//         if (I2C2_WaitByteReceived() < 0)
//         {
//             I2C_GenerateSTOP(I2C2, ENABLE);
//             I2C_AcknowledgeConfig(I2C2, ENABLE);
//             return -1;
//         }

//         buf[i] = I2C_ReceiveData(I2C2);

//         if (i == (len - 1))
//         {
//             I2C_GenerateSTOP(I2C2, ENABLE);
//             I2C_AcknowledgeConfig(I2C2, ENABLE);
//         }
//     }

//     return 0;

// error:
//     g_i2c_last_error = 4;   // 4 = ReadMulti 内部其它错误
//     g_i2c_error_count++;
//     I2C_GenerateSTOP(I2C2, ENABLE);
//     return -1;
// }

// /* ================== ICM20948 的 Bank 操作封装 ================== */

// static void ICM20948_SetBank(uint8_t bank)
// {
//     /* 这里忽略返回值，失败的话后续读写也会返回错误 */
//     (void)I2C2_WriteByte(ICM20948_REG_BANK_SEL, (bank & 0x03) << 4);
// }

// /* 读某个 bank 下的寄存器（出错时返回 0xFF） */
// static uint8_t ICM20948_ReadReg(uint8_t bank, uint8_t reg)
// {
//     uint8_t val = 0xFF;
//     ICM20948_SetBank(bank);
//     if (I2C2_ReadByte(reg, &val) < 0)
//     {
//         return 0xFF;
//     }
//     return val;
// }

// /* 写某个 bank 下的寄存器（忽略错误） */
// static void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t val)
// {
//     ICM20948_SetBank(bank);
//     (void)I2C2_WriteByte(reg, val);
// }

// /* ================== 对外接口：初始化 & 读原始数据 ================== */

// uint8_t ICM20948_Init(void)
// {
//     ICM20948_I2C_HW_Init();

//     /* 简单上电延时 */
//     for (volatile uint32_t i = 0; i < 1000000; i++)
//         ;

//     /* 读取 WHO_AM_I */
//     uint8_t who_am_i = ICM20948_ReadReg(0, ICM20948_REG_WHO_AM_I);
//     if (who_am_i != ICM20948_WHO_AM_I_VALUE)
//     {
//         /* 可以在这里打印一下实际读到的 who_am_i 调试 */
//         return 0;
//     }

//     /* PWR_MGMT_1: 退出睡眠，选择时钟 */
//     ICM20948_WriteReg(0, ICM20948_REG_PWR_MGMT_1, 0x01);

//     /* PWR_MGMT_2: 使能加速度计和陀螺仪各轴 */
//     ICM20948_WriteReg(0, ICM20948_REG_PWR_MGMT_2, 0x00);

//     return 1;
// }

// /* 读取 6 轴原始数据
//  * 成功返回 1，失败返回 0
//  */
// uint8_t ICM20948_ReadData(ICM20948_RawData_t *data)
// {
//     uint8_t buf[12];

//     if (data == NULL)
//         return 0;

//     ICM20948_SetBank(0);
//     if (I2C2_ReadMulti(ICM20948_REG_ACCEL_XOUT_H, buf, 12) < 0)
//     {
//         /* 本次读取失败 */
//         return 0;
//     }

//     data->ax = (int16_t)((buf[0] << 8) | buf[1]);
//     data->ay = (int16_t)((buf[2] << 8) | buf[3]);
//     data->az = (int16_t)((buf[4] << 8) | buf[5]);
//     data->gx = (int16_t)((buf[6] << 8) | buf[7]);
//     data->gy = (int16_t)((buf[8] << 8) | buf[9]);
//     data->gz = (int16_t)((buf[10] << 8) | buf[11]);

//     return 1;
// }

// /* 简单零偏标定（仍然用阻塞 + 延时，但次数有限） */
// void ICM20948_Calibrate(ICM20948_Offset_t *off, uint16_t N)
// {
//     ICM20948_RawData_t data;
//     int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
//     int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
//     uint16_t valid = 0;

//     if (off == NULL || N == 0)
//         return;

//     for (uint16_t i = 0; i < N; ++i)
//     {
//         if (ICM20948_ReadData(&data))
//         {
//             ax_sum += data.ax;
//             ay_sum += data.ay;
//             az_sum += data.az;
//             gx_sum += data.gx;
//             gy_sum += data.gy;
//             gz_sum += data.gz;
//             valid++;
//         }
//         SPL_Delay_ms(5);
//     }

//     if (valid == 0)
//     {
//         ICM20948_MarkCalibrated(0);
//         return;
//     }

//     off->ax_offset = (float)ax_sum / (float)valid;
//     off->ay_offset = (float)ay_sum / (float)valid;
//     off->az_offset = (float)az_sum / (float)valid;
//     off->gx_offset = (float)gx_sum / (float)valid;
//     off->gy_offset = (float)gy_sum / (float)valid;
//     off->gz_offset = (float)gz_sum / (float)valid;
//     ICM20948_MarkCalibrated(1);
// }

// /* 姿态解算 + 互补滤波 */
// void ICM20948_Process(ICM20948_RawData_t *rdata,
//                       ICM20948_Offset_t *off,
//                       ICM20948_ProcessedData_t *pdata,
//                       float dt)
// {
//     const float ACC_LSB_PER_G = 16384.0f;
//     const float GYRO_LSB_PER_DPS = 131.0f;
//     const float DEG2RAD = 0.01745329251994f;
//     const float ALPHA = 0.98f;

//     if (rdata == NULL || off == NULL || pdata == NULL || dt <= 0.0f)
//         return;

//     /* 1. 去零偏 */
//     float ax_corrected = (float)rdata->ax - off->ax_offset;
//     float ay_corrected = (float)rdata->ay - off->ay_offset;
//     float az_corrected = (float)rdata->az - off->az_offset;
//     float gx_corrected = (float)rdata->gx - off->gx_offset;
//     float gy_corrected = (float)rdata->gy - off->gy_offset;
//     float gz_corrected = (float)rdata->gz - off->gz_offset;

//     /* 2. 转换为物理量 */
//     pdata->ax_g = ax_corrected / ACC_LSB_PER_G;
//     pdata->ay_g = ay_corrected / ACC_LSB_PER_G;
//     pdata->az_g = az_corrected / ACC_LSB_PER_G + 1.0f;
//     pdata->gx_rads = (gx_corrected / GYRO_LSB_PER_DPS) * DEG2RAD;
//     pdata->gy_rads = (gy_corrected / GYRO_LSB_PER_DPS) * DEG2RAD;
//     pdata->gz_rads = (gz_corrected / GYRO_LSB_PER_DPS) * DEG2RAD;

//     /* 3. 利用加速度计估算姿态（rad） */
//     float roll_acc = atan2f(pdata->ay_g, pdata->az_g);
//     float pitch_acc = atan2f(-pdata->ax_g,
//                              sqrtf(pdata->ay_g * pdata->ay_g +
//                                    pdata->az_g * pdata->az_g));

//     /* 4. 互补滤波：先积分陀螺，再融合加速度 */
//     static float roll = 0.0f;
//     static float pitch = 0.0f;
//     static float yaw = 0.0f;
//     static uint8_t first_run = 1;

//     if (first_run)
//     {
//         roll = roll_acc;
//         pitch = pitch_acc;
//         yaw = 0.0f;
//         first_run = 0;
//     }
//     else
//     {
//         roll += pdata->gx_rads * dt;
//         pitch += pdata->gy_rads * dt;
//         yaw += pdata->gz_rads * dt;

//         /* 把 yaw 限制在 -π ~ π */
//         yaw = atan2f(sinf(yaw), cosf(yaw));
//     }

//     roll = ALPHA * roll + (1.0f - ALPHA) * roll_acc;
//     pitch = ALPHA * pitch + (1.0f - ALPHA) * pitch_acc;

//     pdata->roll = roll;
//     pdata->pitch = pitch;
//     pdata->yaw = yaw;
// }
