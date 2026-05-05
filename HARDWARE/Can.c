#include <stddef.h>
#include "stm32f4xx.h"
#include "Can.h"
#include "IMU.h"

#define CAN_DEG_TO_MRAD 17.45329251994f
#define IMU_CAN_FLAG_DATA_VALID 0x01U
#define IMU_CAN_FLAG_CALIBRATED 0x02U
#define IMU_CAN_FLAG_RECOVERING 0x04U
#define IMU_CAN_FLAG_FAULT 0x08U

CanRxMsg MyCan_RxMsg;
uint8_t MyCan_RxFlag;

static void CAN_ClearBuffer(uint8_t *buffer)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        buffer[i] = 0U;
    }
}

static void CAN_PutI16LE(uint8_t *buffer, uint8_t offset, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    buffer[offset] = (uint8_t)(raw & 0xFFU);
    buffer[offset + 1U] = (uint8_t)((raw >> 8) & 0xFFU);
}

static void CAN_PutU16LE(uint8_t *buffer, uint8_t offset, uint16_t value)
{
    buffer[offset] = (uint8_t)(value & 0xFFU);
    buffer[offset + 1U] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CAN_SendStd8(uint16_t std_id, const uint8_t *buffer)
{
    CanTxMsg TxMessage;
    uint8_t i;

    TxMessage.StdId = std_id;
    TxMessage.ExtId = 0x00000000;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8;

    for (i = 0U; i < 8U; i++)
    {
        TxMessage.Data[i] = buffer[i];
    }

    (void)CAN1_Send(&TxMessage);
}

static uint8_t IMU_CAN_StatusFlags(const IMU_State_t *p)
{
    uint8_t flags = 0U;

    if (p->data_valid != 0U)
    {
        flags |= IMU_CAN_FLAG_DATA_VALID;
    }
    if (p->calibrated != 0U)
    {
        flags |= IMU_CAN_FLAG_CALIBRATED;
    }
    if (p->recovering != 0U)
    {
        flags |= IMU_CAN_FLAG_RECOVERING;
    }
    if (p->fault != 0U)
    {
        flags |= IMU_CAN_FLAG_FAULT;
    }

    return flags;
}

static void IMU_CAN_SendStatus(const IMU_State_t *p)
{
    uint8_t buffer[8];

    CAN_ClearBuffer(buffer);
    buffer[0] = (uint8_t)(p->seq & 0xFFU);
    buffer[1] = IMU_CAN_StatusFlags(p);
    buffer[2] = p->last_error;
    buffer[3] = p->consecutive_failures;
    buffer[4] = (uint8_t)(p->read_fail_count & 0xFFU);
    buffer[5] = (uint8_t)(p->recovery_count & 0xFFU);
    CAN_PutU16LE(buffer, 6U, (uint16_t)(p->last_update_ms & 0xFFFFU));
    CAN_SendStd8(0x184U, buffer);
}

void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
    CAN_InitStructure.CAN_Prescaler = 56;
    CAN_Init(CAN1, &CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t CAN1_Send(CanTxMsg *TxMessage)
{
    uint8_t TransmitMailbox;

    if (TxMessage == NULL)
    {
        return 0U;
    }

    TransmitMailbox = CAN_Transmit(CAN1, TxMessage);
    if (TransmitMailbox == CAN_TxStatus_NoMailBox)
    {
        return 0U;
    }

    return 1U;
}

void IMU_CAN_SendAll(const IMU_State_t *p)
{
    uint8_t buffer[8];
    uint8_t status_flags;

    if (p == NULL)
    {
        return;
    }

    status_flags = IMU_CAN_StatusFlags(p);

    if (p->data_valid == 0U || p->fault != 0U)
    {
        IMU_CAN_SendStatus(p);
        return;
    }

    CAN_ClearBuffer(buffer);
    CAN_PutI16LE(buffer, 0U, (int16_t)(p->ax_g * 1000.0f));
    CAN_PutI16LE(buffer, 2U, (int16_t)(p->ay_g * 1000.0f));
    CAN_PutI16LE(buffer, 4U, (int16_t)(p->az_g * 1000.0f));
    buffer[6] = (uint8_t)(p->seq & 0xFFU);
    buffer[7] = status_flags;
    CAN_SendStd8(0x180U, buffer);

    CAN_ClearBuffer(buffer);
    CAN_PutI16LE(buffer, 0U, (int16_t)(p->gx_rads * 1000.0f));
    CAN_PutI16LE(buffer, 2U, (int16_t)(p->gy_rads * 1000.0f));
    CAN_PutI16LE(buffer, 4U, (int16_t)(p->gz_rads * 1000.0f));
    buffer[6] = (uint8_t)(p->seq & 0xFFU);
    buffer[7] = status_flags;
    CAN_SendStd8(0x181U, buffer);

    CAN_ClearBuffer(buffer);
    CAN_PutI16LE(buffer, 0U, (int16_t)(p->roll_deg * CAN_DEG_TO_MRAD));
    CAN_PutI16LE(buffer, 2U, (int16_t)(p->pitch_deg * CAN_DEG_TO_MRAD));
    CAN_PutI16LE(buffer, 4U, (int16_t)(p->yaw_deg * CAN_DEG_TO_MRAD));
    buffer[6] = (uint8_t)(p->seq & 0xFFU);
    buffer[7] = status_flags;
    CAN_SendStd8(0x182U, buffer);

    IMU_CAN_SendStatus(p);
}

void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &MyCan_RxMsg);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        MyCan_RxFlag = 1U;
    }
}
