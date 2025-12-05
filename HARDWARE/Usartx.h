#ifndef __USARTX_H
#define __USARTX_H

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>


typedef enum
{
    STATE_HDR0,
    STATE_HDR1,
    STATE_ID,
    STATE_LEN,
    STATE_DATA,
    STATE_CRC0,
    STATE_CRC1
} ParserState;

extern float vA_mps, vB_mps, vC_mps, vD_mps;
extern uint8_t Serial2_RXData;
extern volatile uint8_t Serial2_RXFlag;

extern volatile uint16_t Serial3_RXData;
extern volatile uint8_t Serial3_RXFlag;

void Serial2_Init(void);
void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array,uint16_t Length);
void Serial2_SendString(char *String);
uint32_t Serial2_Pow(uint32_t X,uint32_t Y);
void Serial2_SendNum(uint32_t Num,uint8_t Length);
void Serial2_Printf(char *format, ...);
uint8_t Serial2_GetRXFlag(void);
uint8_t Serial2_GetRXData(void);
void HandleCommand(uint8_t cmd);


uint16_t Serial3_CRC16(const uint8_t* data, size_t len);
void Serial3_SendMeasPacket(float w1, float w2, float w3, float w4);
void Serial3_ParsePacket(uint8_t data);
void Serial3_Init(void);
void Serial3_SendByte(uint8_t Byte);
void Serial3_SendArray(uint8_t *Array, uint16_t Length);
void Serial3_SendFloat(float value);
uint8_t Serial3_GetRXFlag(void);

#endif 
