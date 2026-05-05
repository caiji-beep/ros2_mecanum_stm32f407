/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-04 21:01:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-05 19:37:05
 * @FilePath: \ros2_mecanum\Algorithm\imu_fusion.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "imu_fusion.h"
#include <math.h>

/* ====== 算法参数 ====== */
#define KP 2.0f                   /* 比例增益：加速度计修正力度 */
#define KI 0.005f                 /* 积分增益：陀螺仪偏置估计 */
#define DEG2RAD 0.01745329251994f /* PI / 180 */
#define RAD2DEG 57.2957795130823f /* 180 / PI */

/* 两个采样周期内加速度计幅值偏差门限，超过则认为在剧烈运动，减小修正 */
#define ACCEL_REJECT_THRESHOLD 0.75f
#define GYRO_DEADBAND_DPS 0.05f
#define STATIONARY_ACCEL_THRESHOLD_G 0.08f  // 加速度计幅值偏差门限
#define STATIONARY_GYRO_THRESHOLD_DPS 0.5f // 陀螺仪角速度门限
#define GYRO_BIAS_ADAPT_ALPHA 0.005f  // EMA 平滑系数

/* ====== 初始化 ====== */

static float IMU_ApplyDeadband(float value, float threshold)
{
    return (fabsf(value) < threshold) ? 0.0f : value;
}

void IMU_Fusion_Init(IMU_Fusion_t *fusion)
{
    fusion->q0 = 1.0f;
    fusion->q1 = 0.0f;
    fusion->q2 = 0.0f;
    fusion->q3 = 0.0f;
    fusion->roll = 0.0f;
    fusion->pitch = 0.0f;
    fusion->yaw = 0.0f;
    fusion->ex_int = 0.0f;
    fusion->ey_int = 0.0f;
    fusion->ez_int = 0.0f;
    fusion->gyro_bias_x_dps = 0.0f;
    fusion->gyro_bias_y_dps = 0.0f;
    fusion->gyro_bias_z_dps = 0.0f;
}

void IMU_Fusion_SetGyroBias(IMU_Fusion_t *fusion,
                            float gx_bias_dps,
                            float gy_bias_dps,
                            float gz_bias_dps)
{
    fusion->gyro_bias_x_dps = gx_bias_dps;
    fusion->gyro_bias_y_dps = gy_bias_dps;
    fusion->gyro_bias_z_dps = gz_bias_dps;
}



void IMU_Fusion_Update(IMU_Fusion_t *fusion,
                       float gx_dps, float gy_dps, float gz_dps,
                       float ax_g,   float ay_g,   float az_g,
                       float dt)
{
    float norm;
    float q0, q1, q2, q3;
    float gx_raw_dps, gy_raw_dps, gz_raw_dps;
    float gx_corr_dps, gy_corr_dps, gz_corr_dps;
    float gyro_magnitude_dps;
    float vx, vy, vz;          /* 重力方向估计值 */
    float ex, ey, ez;          /* 误差 = 加速度计 × 重力估计 */
    float gx, gy, gz;          /* 修正后的角速度 (rad/s) */

    if (dt <= 0.0f) return;

    gx_raw_dps = gx_dps;
    gy_raw_dps = gy_dps;
    gz_raw_dps = gz_dps;

    norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (norm < 1e-6f) return;

    gx_corr_dps = gx_raw_dps - fusion->gyro_bias_x_dps;
    gy_corr_dps = gy_raw_dps - fusion->gyro_bias_y_dps;
    gz_corr_dps = gz_raw_dps - fusion->gyro_bias_z_dps;
    gyro_magnitude_dps = sqrtf(gx_corr_dps * gx_corr_dps +
                               gy_corr_dps * gy_corr_dps +
                               gz_corr_dps * gz_corr_dps);

    // 判断是否静止
    if ((fabsf(norm - 1.0f) < STATIONARY_ACCEL_THRESHOLD_G) &&
        (gyro_magnitude_dps < STATIONARY_GYRO_THRESHOLD_DPS))
    {
        // 静止 → 缓慢更新偏置
        fusion->gyro_bias_x_dps += (gx_raw_dps - fusion->gyro_bias_x_dps) * GYRO_BIAS_ADAPT_ALPHA;
        fusion->gyro_bias_y_dps += (gy_raw_dps - fusion->gyro_bias_y_dps) * GYRO_BIAS_ADAPT_ALPHA;
        fusion->gyro_bias_z_dps += (gz_raw_dps - fusion->gyro_bias_z_dps) * GYRO_BIAS_ADAPT_ALPHA;
    }

    /* ---------- 第一步：单位转换 dps → rad/s ---------- */
    gx_dps = IMU_ApplyDeadband(gx_dps - fusion->gyro_bias_x_dps, GYRO_DEADBAND_DPS);
    gy_dps = IMU_ApplyDeadband(gy_dps - fusion->gyro_bias_y_dps, GYRO_DEADBAND_DPS);
    gz_dps = IMU_ApplyDeadband(gz_dps - fusion->gyro_bias_z_dps, GYRO_DEADBAND_DPS);

    gx = gx_dps * DEG2RAD;
    gy = gy_dps * DEG2RAD;
    gz = gz_dps * DEG2RAD;

    /* ---------- 第二步：加速度计归一化 ---------- */
    norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (norm < 1e-6f) return;          /* 数据无效，跳过 */
    ax_g /= norm;
    ay_g /= norm;
    az_g /= norm;

    /* ---------- 第三步：从当前四元数估计重力方向 ---------- */
    /*
     * 把世界坐标系下的绝对重力 $G = [0, 0, 1]^T$，反向投影到车体坐标系下
     * 重力在机体坐标系中的估计值：
     *   vx = 2*(q1*q3 - q0*q2)
     *   vy = 2*(q0*q1 + q2*q3)
     *   vz = q0² - q1² - q2² + q3²
     */
    vx = 2.0f * (fusion->q1 * fusion->q3 - fusion->q0 * fusion->q2);
    vy = 2.0f * (fusion->q0 * fusion->q1 + fusion->q2 * fusion->q3);
    vz = fusion->q0 * fusion->q0 - fusion->q1 * fusion->q1
       - fusion->q2 * fusion->q2 + fusion->q3 * fusion->q3;

    /* ---------- 第四步：计算误差（叉积） ---------- */
    /*
     * 加速度计测量的重力方向 与 估计的重力方向 做叉积
     * 结果就是 roll/pitch 的误差
     */
    ex = (ay_g * vz - az_g * vy);
    ey = (az_g * vx - ax_g * vz);
    ez = (ax_g * vy - ay_g * vx);


    /* ---------- 第五步：动态调节——剧烈运动时减小修正 ---------- */
    {
        float accel_magnitude = norm;
        float accel_error = fabsf(accel_magnitude - 1.0f);  /* 静止时应为 1g */

        if (accel_error > ACCEL_REJECT_THRESHOLD)
        {
            /* 剧烈加减速，加速度计不可靠，减小修正力度 */
            ex *= 0.1f;
            ey *= 0.1f;
            ez *= 0.1f;
        }
    }

    /* ---------- 第六步：PI 控制器修正陀螺仪 ---------- */
    fusion->ex_int += ex * KI * dt;
    fusion->ey_int += ey * KI * dt;
    //fusion->ez_int += ez * KI * dt;
    fusion->ez_int = 0.0f;  /* 不估计偏航积分误差，避免长期漂移 */
    ez = 0.0f;

     /* 积分限幅，防止 windup */
    #define INT_LIMIT   0.5f
    if (fusion->ex_int >  INT_LIMIT) fusion->ex_int =  INT_LIMIT;
    if (fusion->ex_int < -INT_LIMIT) fusion->ex_int = -INT_LIMIT;
    if (fusion->ey_int >  INT_LIMIT) fusion->ey_int =  INT_LIMIT;
    if (fusion->ey_int < -INT_LIMIT) fusion->ey_int = -INT_LIMIT;
    // if (fusion->ez_int >  INT_LIMIT) fusion->ez_int =  INT_LIMIT;
    // if (fusion->ez_int < -INT_LIMIT) fusion->ez_int = -INT_LIMIT;

    gx += KP * ex + fusion->ex_int;
    gy += KP * ey + fusion->ey_int;
    //gz += KP * ez + fusion->ez_int;

        /* ---------- 第七步：四元数积分更新 ---------- */
    q0 = fusion->q0;
    q1 = fusion->q1;
    q2 = fusion->q2;
    q3 = fusion->q3;

    fusion->q0 += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    fusion->q1 += ( q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    fusion->q2 += ( q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    fusion->q3 += ( q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;

    /* ---------- 第八步：四元数归一化 ---------- */
    norm = sqrtf(fusion->q0 * fusion->q0 + fusion->q1 * fusion->q1
               + fusion->q2 * fusion->q2 + fusion->q3 * fusion->q3);
    fusion->q0 /= norm;
    fusion->q1 /= norm;
    fusion->q2 /= norm;
    fusion->q3 /= norm;

    /* ---------- 第九步：四元数 → 欧拉角（弧度） ---------- */
    fusion->roll  = atan2f(2.0f * (fusion->q0 * fusion->q1 + fusion->q2 * fusion->q3),
                           1.0f - 2.0f * (fusion->q1 * fusion->q1 + fusion->q2 * fusion->q2));

    fusion->pitch = asinf(2.0f * (fusion->q0 * fusion->q2 - fusion->q3 * fusion->q1));

    fusion->yaw   = atan2f(2.0f * (fusion->q0 * fusion->q3 + fusion->q1 * fusion->q2),
                           1.0f - 2.0f * (fusion->q2 * fusion->q2 + fusion->q3 * fusion->q3));

}


/* ====== 获取欧拉角（度） ====== */

void IMU_Fusion_GetEuler(IMU_Fusion_t *fusion,
                         float *roll, float *pitch, float *yaw)
{
    if (roll)  *roll  = fusion->roll  * RAD2DEG;
    if (pitch) *pitch = fusion->pitch * RAD2DEG;
    if (yaw)   *yaw   = fusion->yaw   * RAD2DEG;
}

/* ====== 获取四元数 ====== */

void IMU_Fusion_GetQuaternion(IMU_Fusion_t *fusion,
                              float *q0, float *q1, float *q2, float *q3)
{
    if (q0) *q0 = fusion->q0;
    if (q1) *q1 = fusion->q1;
    if (q2) *q2 = fusion->q2;
    if (q3) *q3 = fusion->q3;
}

