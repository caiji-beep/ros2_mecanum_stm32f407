/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 20:13:36
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-05 16:00:35
 * @FilePath: \ros2_mecanum\MyTasks\IMU.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

typedef struct {
    float ax_g, ay_g, az_g;
    float gx_rads, gy_rads, gz_rads;
    float roll_deg, pitch_deg, yaw_deg;
    uint32_t seq;
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
} IMU_State_t;

void IMU_Task(void *pvParameters);
uint8_t IMU_GetSnapshot(IMU_State_t *out);

#endif
