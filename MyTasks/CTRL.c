#include "stm32f4xx.h"
#include "freertos_demo.h"
#include <string.h>
#include "PWM.h"
#include "Encoder.h"
#include "Uart.h"
#include "CTRL.h"
#include "robot_state.h"
#include "Telemetry.h"
#include <math.h>
#include "watchdog.h"
#include "IMU_Icm20948.h"

// === 低速起步补偿（静摩擦）===
#define SC_DEADZONE_PWM (1200)      // 大死区补偿，必须 ≥ 800
#define SC_VREF_STOP_THR (0.01f)    // “真正停车”的阈值
#define SC_VREF_SLOW_THR (0.10f)    // 低速微调 vs 正常行驶的分界
#define SC_DEADZONE_PWM_SLOW (1000) // 低速微调用的起步补偿，必须 ≥ 800

extern volatile Robot_state g_robot_state;
extern volatile CtrlMode g_mode;
extern TaskHandle_t gTelemetryTask_Handle;

extern TelemetryData_t g_telemetry_data;
extern volatile uint8_t g_nav_cmd_alive;

static SC_One s_mot[4];

static inline float clampf(float x, float lo, float hi)   //限制在lo与hi之间
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}
static inline int clampi(int x, int lo, int hi)   //限制在lo与hi之间
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

// 斜坡限幅：限制 PWM 每周期最大变化量，保护电流/机械
static inline int slew_limit(int now, int target, int step)    //变化量限制在-step~step
{
    int d = target - now;
    if (d > step)
        d = step;
    if (d < -step)
        d = -step;
    return now + d;
}

void SC_Init(void)
{
    // 默认增益（起点值，后面再按轮调）
    const SC_Gains g0 = {.Kp = 4200.0f, .Ki = 180.0f, .Kff = 11111.0f};
    for (int i = 0; i < 4; i++)
    {
        s_mot[i].Kp = g0.Kp;
        s_mot[i].Ki = g0.Ki;
        s_mot[i].Kff = g0.Kff;
        s_mot[i].integ = 0.0f;
        s_mot[i].v_target = 0.0f;
        s_mot[i].v_meas = 0.0f;
        s_mot[i].u_pwm = 0;
    }
}

void SC_SetGains(EncoderId id, const SC_Gains *g)
{
    int i = (int)id;
    s_mot[i].Kp = g->Kp;
    s_mot[i].Ki = g->Ki;
    s_mot[i].Kff = g->Kff;
}

void SC_SetTarget_mps(EncoderId id, float v_mps)
{
    s_mot[(int)id].v_target = v_mps;
}

float SC_GetTarget_mps(EncoderId id)
{
    return s_mot[(int)id].v_target;
}
float SC_GetMeas_mps(EncoderId id)
{
    return s_mot[(int)id].v_meas;
}
int SC_GetPWM(EncoderId id)
{
    return s_mot[(int)id].u_pwm;
}

void SC_Step(void)
{
    // 1) 先读取测速（注意：确保 Encoder_Sample(SC_Ts) 已按 10ms 跑）
    s_mot[ENC_A].v_meas = Encoder_Speed_mps(ENC_A);
    s_mot[ENC_B].v_meas = Encoder_Speed_mps(ENC_B);
    s_mot[ENC_C].v_meas = Encoder_Speed_mps(ENC_C);
    s_mot[ENC_D].v_meas = Encoder_Speed_mps(ENC_D);

    // 2) 每轮 PI + 前馈 + 抗饱和 + 斜坡限幅
    for (int i = 0; i < 4; i++)
    {
        float v_ref = s_mot[i].v_target;
        float v_meas = s_mot[i].v_meas;
        float e = v_ref - v_meas;

        // 前馈：把目标速度按比例直接映射到 PWM 近似值
        float u_ff = s_mot[i].Kff * v_ref;

        // PI（不含抗饱和的“候选”输出）
        float u_pi_noI = s_mot[i].Kp * e;
        float integ = s_mot[i].integ + s_mot[i].Ki * e * SC_Ts;

        // 预限制积分，避免无限增长
        integ = clampf(integ, SC_I_MIN, SC_I_MAX);

        // 候选输出（未限幅）
        float u_unsat = u_ff + u_pi_noI + integ;

        // 输出限幅（抗饱和）
        float u_sat_f = clampf(u_unsat, (float)SC_OUT_MIN, (float)SC_OUT_MAX);

        // 条件积分（anti-windup）：只有在“没饱和”或“误差推动回可行域”时，才接受积分
        int saturated = (u_unsat != u_sat_f);    //不相等说明饱和
        if (saturated)
        {
            // 判断饱和方向与误差方向是否相反（能把输出往可行域推）
            int pushing_back = ((u_unsat > SC_OUT_MAX) && (e < 0)) ||
                               ((u_unsat < SC_OUT_MIN) && (e > 0));
            if (!pushing_back)
            {
                // 丢弃这次积分更新，使用上一次积分
                // （也可用 back-calculation：integ += Kaw*(u_sat - u_unsat)）
                integ = s_mot[i].integ;
            }
        }
        s_mot[i].integ = integ;

        // 最终目标 PWM（float），再斜坡限幅到 int
        int u_target_pwm = (int)(u_sat_f);

        // 斜坡限幅，保护电流/机械冲击
        int u_next = slew_limit(s_mot[i].u_pwm, u_target_pwm, SC_SLEW_PER_TICK);

        // 死区补偿（三段式：停车 / 低速 / 正常）
#if (SC_DEADZONE_PWM > 0)
        if (u_next != 0)
        {
            float v_ref_abs = fabsf(v_ref);

            if (v_ref_abs < SC_VREF_STOP_THR)
            {
                // ① 真正停车区间：把小 PWM 压成 0，防止终点抖动
                if (u_next > -SC_DEADZONE_PWM_SLOW && u_next < SC_DEADZONE_PWM_SLOW)
                {
                    u_next = 0;
                }
            }
            else if (v_ref_abs < SC_VREF_SLOW_THR)
            {
                // ② 低速微调区间：给“够用”的起步补偿
                if (u_next > 0 && u_next < SC_DEADZONE_PWM_SLOW)
                    u_next = SC_DEADZONE_PWM_SLOW;
                else if (u_next < 0 && -u_next < SC_DEADZONE_PWM_SLOW)
                    u_next = -SC_DEADZONE_PWM_SLOW;
            }
            else
            {
                // ③ 正常速度区间：使用原来的大死区补偿
                if (u_next > 0 && u_next < SC_DEADZONE_PWM)
                    u_next = SC_DEADZONE_PWM;
                else if (u_next < 0 && -u_next < SC_DEADZONE_PWM)
                    u_next = -SC_DEADZONE_PWM;
            }
        }
#endif
        // 最终限幅保险
        u_next = clampi(u_next, SC_OUT_MIN, SC_OUT_MAX);

        s_mot[i].u_pwm = u_next;
    }

    // 3) 一次性下发 4 路 PWM
    Set_Pwm(s_mot[ENC_A].u_pwm,
            s_mot[ENC_B].u_pwm,
            s_mot[ENC_C].u_pwm,
            s_mot[ENC_D].u_pwm);
}

// extern ICM20948_RawData_t IMU_data;
// extern ICM20948_Offset_t IMU_offset;
// extern ICM20948_ProcessedData_t IMU_processed;

void Ctrl_Task(void *pvParameters)
{
    // printf("Ctrl_Task started\r\n");
    // uint32_t cnt_10ms = 0; // 10ms 计数器，用来凑 100ms
    while (1)
    {
        //    阻塞等待 TIM6 的一次 10ms“节拍”
        //    pdTRUE: 读取同时清零计数
        //    portMAX_DELAY: 一直等
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待定时器通知
        Encoder_Sample(SC_Ts);                   // 10ms 编码器采样
        // ICM20948_ReadData(&IMU_data);
        // ICM20948_Process(&IMU_data, &IMU_offset, &IMU_processed, IMU_dt);

        if (g_mode == MODE_VEL)
        {
            if (g_robot_state == ROBOT_STATE_NAV && g_nav_cmd_alive == 1)
            {
                TickType_t now = xTaskGetTickCount();
                if ((now - g_nav_last_rx_tick) > pdMS_TO_TICKS(300)) // 比如 300ms
                {
                    // Nav2 失联 / 速度包超时：立刻停车并回到 IDLE
                    Robot_EnterState(ROBOT_STATE_IDLE);
                    g_nav_cmd_alive = 0;
                    continue; // 本周期不再跑 SC_Step
                }
            }
            SC_Step();
        }
        g_telemetry_data.wA = Encoder_Speed_radps(ENC_A);
        g_telemetry_data.wB = Encoder_Speed_radps(ENC_B);
        g_telemetry_data.wC = Encoder_Speed_radps(ENC_C);
        g_telemetry_data.wD = Encoder_Speed_radps(ENC_D);
        // if (++cnt_10ms >= 10)
        // {
        //     cnt_10ms = 0;
        //     // 从编码器模块读当前角速度（rad/s）
        //     g_telemetry_data.wA = Encoder_Speed_radps(ENC_A);
        //     g_telemetry_data.wB = Encoder_Speed_radps(ENC_B);
        //     g_telemetry_data.wC = Encoder_Speed_radps(ENC_C);
        //     g_telemetry_data.wD = Encoder_Speed_radps(ENC_D);
        //     // 通知 Telemetry_Task 有新数据了
        //     if (gTelemetryTask_Handle)
        //     {
        //         xTaskNotifyGive(gTelemetryTask_Handle);
        //     }
        // }
        IWDG_Feed(); // 喂狗    每次必须间隔小于看门狗超时时间
    }
}
