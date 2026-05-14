/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2025-12-01 20:38:47
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\MyTasks\Telemetry.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __TELEMETRY_H
#define __TELEMETRY_H

/* 遥测用缓存数据，Ctrl_Task 写，Telemetry_Task 读 */
typedef struct
{
    float wA;
    float wB;
    float wC;
    float wD;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_rads;
    float gy_rads;
    float gz_rads;
    float roll;
    float pitch;
    float yaw;
} TelemetryData_t;
extern TelemetryData_t g_telemetry_data;


#endif 
