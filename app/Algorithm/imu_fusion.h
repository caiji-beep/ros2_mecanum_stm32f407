/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-04 21:01:32
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-04 21:22:36
 * @FilePath: \ros2_mecanum\Algorithm\imu_fusion.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __IMU_FUSION_H
#define __IMU_FUSION_H

#include <stdint.h>

typedef struct {
    /* 四元数 */
    float q0, q1, q2, q3;

    /* 欧拉角（弧度） */
    float roll, pitch, yaw;

    /* 积分误差（陀螺仪偏置估计） */
    float ex_int, ey_int, ez_int;
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;
} IMU_Fusion_t;

/**
 * @brief 初始化融合器
 */
void IMU_Fusion_Init(IMU_Fusion_t *fusion);
void IMU_Fusion_SetGyroBias(IMU_Fusion_t *fusion,
                            float gx_bias_dps,
                            float gy_bias_dps,
                            float gz_bias_dps);

/**
 * @brief 更新姿态（每次 IMU 数据就绪时调用）
 * @param gx, gy, gz  陀螺仪数据，单位 dps（度/秒）
 * @param ax, ay, az  加速度计数据，单位 g
 * @param dt          距上次调用的时间间隔，单位秒
 */
void IMU_Fusion_Update(IMU_Fusion_t *fusion,
                       float gx_dps, float gy_dps, float gz_dps,
                       float ax_g,   float ay_g,   float az_g,
                       float dt);

/**
 * @brief 获取欧拉角（度）
 */
void IMU_Fusion_GetEuler(IMU_Fusion_t *fusion,
                         float *roll, float *pitch, float *yaw);

/**
 * @brief 获取四元数
 */
void IMU_Fusion_GetQuaternion(IMU_Fusion_t *fusion,
                              float *q0, float *q1, float *q2, float *q3);

#endif
