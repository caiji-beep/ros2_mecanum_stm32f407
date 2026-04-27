/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 23:05:35
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-04-27 21:52:07
 * @FilePath: \EIDEe:\STM32_Documents\PROJECT\ros2_mecanum\HARDWARE\Can.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%
 */
#include "stm32f4xx.h"
#include "Can.h"
#include "IMU_Icm20948.h"

CanRxMsg MyCan_RxMsg;
uint8_t MyCan_RxFlag;

void CAN1_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // 使能GPIOD时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // 使能CAN1时钟

	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* PD0 = CAN1_RX, PD1 = CAN1_TX, 复用 AF9 */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); // PD0 -> CAN1_RX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); // PD1 -> CAN1_TX

    // 配置PD1（CAN_TX）：复用推挽输出，高速，上拉
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // F4复用功能模式（替换F1的GPIO_Mode_AF_PP）
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          // 推挽输出（F1合并在Mode里，F4单独配置）
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            // 上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    // 配置PD0（CAN_RX）：复用输入，上拉
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // 复用功能（CAN_RX需要复用输入）
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            // 上拉输入（CAN_RX推荐上拉）
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_Prescaler = 56;//波特率 = 42M / 56 / (1+2+3) = 125k  高速CAN:125k-1M
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;	
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_Init(CAN1,&CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh      = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow       = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh  = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow   = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation  = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);


    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); // 

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t CAN1_Send(CanTxMsg* TxMessage)
{
	uint8_t TransmitMailbox = CAN_Transmit(CAN1,TxMessage);
    if(TransmitMailbox == CAN_TxStatus_NoMailBox)
    {
        return 0; // 发送失败
    }
    return 1; // 发送成功
}

static uint8_t IMU_StatusFlags(const ICM20948_Status_t *s)
{
    uint8_t flags = 0;
    if (s->data_valid)
        flags |= 0x01;
    if (s->calibrated)
        flags |= 0x02;
    if (s->recovering)
        flags |= 0x04;
    if (s->fault)
        flags |= 0x08;
    return flags;
}

// void IMU_CAN_SendAll(const ICM20948_RawData_t *p)
// {
//     uint8_t buffer[8];
//     CanTxMsg TxMessage;
//     uint8_t seq = imu_seq++;
//     //帧0x180:加速度ax,ay,az
//     int16_t ax = (int16_t)(p->ax * 1000); // 转换为mg
//     int16_t ay = (int16_t)(p->ay * 1000); // 转换为mg
//     int16_t az = (int16_t)(p->az * 1000); // 转换为mg
//     buffer[0] = ax & 0xFF;  // 低字节（LSB）
//     buffer[1] = (ax >> 8) & 0xFF;
//     buffer[2] = ay & 0xFF;
//     buffer[3] = (ay >> 8) & 0xFF;
//     buffer[4] = az & 0xFF;
//     buffer[5] = (az >> 8) & 0xFF;
//     buffer[6] = seq;
//     buffer[7] = 0; // 保留字节

//     TxMessage.StdId = 0x180;
//     TxMessage.ExtId = 0x00000000;
//     TxMessage.IDE = CAN_Id_Standard;
//     TxMessage.RTR = CAN_RTR_Data;
//     TxMessage.DLC = 8;
//     for(uint8_t i = 0;i<8;i++)
//     {
//         TxMessage.Data[i] = buffer[i];
//     }
//     CAN1_Send(&TxMessage);
// }


void IMU_CAN_SendAll(const ICM20948_ProcessedData_t *p)
{
    uint8_t buffer[8];
    CanTxMsg TxMessage;
    ICM20948_Status_t status = ICM20948_GetStatusSnapshot();
    uint8_t seq = status.seq;
    uint8_t status_flags = IMU_StatusFlags(&status);
    
    //帧0x180:加速度ax,ay,az
    int16_t ax = (int16_t)(p->ax_g * 1000); // 转换为mg
    int16_t ay = (int16_t)(p->ay_g * 1000); // 转换为mg
    int16_t az = (int16_t)(p->az_g * 1000); // 转换为mg
    buffer[0] = ax & 0xFF;  // 低字节（LSB）使用小端序，且与上位机保持一致// 低字节在前
    buffer[1] = (ax >> 8) & 0xFF;
    buffer[2] = ay & 0xFF;
    buffer[3] = (ay >> 8) & 0xFF;
    buffer[4] = az & 0xFF;
    buffer[5] = (az >> 8) & 0xFF;
    buffer[6] = seq;
    buffer[7] = status_flags;

    TxMessage.StdId = 0x180;
    TxMessage.ExtId = 0x00000000;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8;
    for(uint8_t i = 0;i<8;i++)
    {
        TxMessage.Data[i] = buffer[i];
    }
    CAN1_Send(&TxMessage);

    //帧0x181:角速度gx,gy,gz
    int16_t gx = (int16_t)(p->gx_rads * 1000); // 转换为mrad/s
    int16_t gy = (int16_t)(p->gy_rads * 1000); // 转换为mrad/s
    int16_t gz = (int16_t)(p->gz_rads * 1000); // 转换为mrad/s
    buffer[0] = gx & 0xFF;  // 低字节（LSB）
    buffer[1] = (gx >> 8) & 0xFF;
    buffer[2] = gy & 0xFF;
    buffer[3] = (gy >> 8) & 0xFF;
    buffer[4] = gz & 0xFF;
    buffer[5] = (gz >> 8) & 0xFF;
    buffer[6] = seq;
    buffer[7] = status_flags;


    TxMessage.StdId = 0x181;
    TxMessage.ExtId = 0x00000000;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8;
    for(uint8_t i = 0;i<8;i++)
    {
        TxMessage.Data[i] = buffer[i];
    }
    CAN1_Send(&TxMessage);
    //帧0x182:姿态角roll,pitch,yaw
    int16_t roll = (int16_t)(p->roll * 1000); // 转换为mrad
    int16_t pitch = (int16_t)(p->pitch * 1000); // 转换为mrad
    int16_t yaw = (int16_t)(p->yaw * 1000); // 转换为mrad
    buffer[0] = roll & 0xFF;  // 低字节（LSB）
    buffer[1] = (roll >> 8) & 0xFF;
    buffer[2] = pitch & 0xFF;
    buffer[3] = (pitch >> 8) & 0xFF;
    buffer[4] = yaw & 0xFF;
    buffer[5] = (yaw >> 8) & 0xFF;
    buffer[6] = seq;
    buffer[7] = 0; // 保留字节  

    TxMessage.StdId = 0x182;
    TxMessage.ExtId = 0x00000000;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8;
    for(uint8_t i = 0;i<8;i++)
    {
        TxMessage.Data[i] = buffer[i];
    }
    CAN1_Send(&TxMessage);

    // 0x184: IMU health/status. Use gyro_z on ROS side for heading fusion.
    buffer[0] = seq;
    buffer[1] = status_flags;
    buffer[2] = status.last_error;
    buffer[3] = status.consecutive_failures;
    buffer[4] = (uint8_t)(status.read_fail_count & 0xFF);
    buffer[5] = (uint8_t)(status.recovery_count & 0xFF);
    buffer[6] = (uint8_t)(status.last_update_ms & 0xFF);
    buffer[7] = (uint8_t)((status.last_update_ms >> 8) & 0xFF);

    TxMessage.StdId = 0x184;
    TxMessage.ExtId = 0x00000000;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8;
    for(uint8_t i = 0;i<8;i++)
    {
        TxMessage.Data[i] = buffer[i];
    }
    CAN1_Send(&TxMessage);
}



void CAN1_RX0_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
    {
        CAN_Receive(CAN1,CAN_FIFO0,&MyCan_RxMsg);
        // 在这里处理接收到的消息 RxMessage
        CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
        MyCan_RxFlag = 1;
    }
}


