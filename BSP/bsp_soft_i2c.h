#ifndef __BSP_SOFT_I2C_H
#define __BSP_SOFT_I2C_H

#include "stm32f4xx.h"
#include "SPL_Delay.h"

/* ====== 总线结构体 ====== */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      scl_pin;
    uint16_t      sda_pin;
} Soft_I2C_Bus;


/* ====== 底层时序原语 ====== */
void    Soft_I2C_Init(Soft_I2C_Bus *bus);
void    Soft_I2C_Start(Soft_I2C_Bus *bus);
void    Soft_I2C_Stop(Soft_I2C_Bus *bus);
uint8_t Soft_I2C_WaitAck(Soft_I2C_Bus *bus);
void    Soft_I2C_SendByte(Soft_I2C_Bus *bus, uint8_t byte);
uint8_t Soft_I2C_ReadByte(Soft_I2C_Bus *bus, uint8_t ack);

/* ====== 中间层：通用裸数据 ====== */
uint8_t Soft_I2C_WriteBytes(Soft_I2C_Bus *bus, uint8_t dev_addr, const uint8_t *data, uint16_t len);
uint8_t Soft_I2C_ReadBytes(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t *buf, uint16_t len);

/* ====== 上层：寄存器式设备 ====== */
uint8_t Soft_I2C_WriteReg(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t Soft_I2C_ReadReg(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
uint8_t Soft_I2C_ReadRegs(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len);

#endif
