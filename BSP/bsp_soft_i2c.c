/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2026-05-03 01:42:31
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-04 03:39:35
 * @FilePath: \ros2_mecanum\BSP\soft_i2c.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_soft_i2c.h"

static void I2C_Delay(void)
{
    SPL_Delay_us(2);
}


/* ====== 底层：所有 GPIO 操作通过 bus->port / bus->scl_pin / bus->sda_pin ====== */

static void SCL_Write(Soft_I2C_Bus *bus, uint8_t x)
{
    GPIO_WriteBit(bus->port, bus->scl_pin, (BitAction)x);
}

static void SDA_Write(Soft_I2C_Bus *bus, uint8_t x)
{
    GPIO_WriteBit(bus->port, bus->sda_pin, (BitAction)x);
}

static uint8_t SDA_Read(Soft_I2C_Bus *bus)
{
    return GPIO_ReadInputDataBit(bus->port, bus->sda_pin);
}

static uint8_t SCL_Read(Soft_I2C_Bus *bus)
{
    return GPIO_ReadInputDataBit(bus->port, bus->scl_pin);
}

/* ====== 初始化 ====== */
void Soft_I2C_Init(Soft_I2C_Bus *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure;

     // 根据端口自动使能时钟
    if (bus->port == GPIOA) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    else if (bus->port == GPIOB) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    else if (bus->port == GPIOC) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    else if (bus->port == GPIOD) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    else if (bus->port == GPIOE) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    else if (bus->port == GPIOF) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    else if (bus->port == GPIOG) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    else if (bus->port == GPIOH) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    else if (bus->port == GPIOI) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

    GPIO_InitStructure.GPIO_Pin = bus->scl_pin | bus->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  /* 普通输出 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // 开漏输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   // 内部上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(bus->port, &GPIO_InitStructure);

    /* 总线空闲电平（高） */
    SCL_Write(bus, 1);
    SDA_Write(bus, 1);

}

/* ====== 时序原语 ====== */

void Soft_I2C_Start(Soft_I2C_Bus *bus)
{
    SDA_Write(bus, 1);
    I2C_Delay();
    SCL_Write(bus, 1);
    I2C_Delay();
    SDA_Write(bus, 0);
    I2C_Delay();
    SCL_Write(bus, 0);
    I2C_Delay();
}

void Soft_I2C_Stop(Soft_I2C_Bus *bus)
{
    SDA_Write(bus, 0);
    I2C_Delay();
    SCL_Write(bus, 1);
    I2C_Delay();
    SDA_Write(bus, 1);
    I2C_Delay();
}

uint8_t Soft_I2C_WaitAck(Soft_I2C_Bus *bus)
{
    uint8_t timeout = 0;
    uint8_t ack_status;

    SDA_Write(bus, 1);
    I2C_Delay();

    /* Clock Stretching：等从机释放 SCL */
    SCL_Write(bus, 1);
    while (SCL_Read(bus) == 0) {    // 从机还拉着 SCL
        if (++timeout > 250) {
            Soft_I2C_Stop(bus);
            return 2;               // 超时，总线异常
        }
        I2C_Delay();
    }

    /* 采样 ACK/NACK：看一眼 SDA */
    if (SDA_Read(bus)) {
        ack_status = 1;             // NACK
        Soft_I2C_Stop(bus);
    } else {
        ack_status = 0;             // ACK
    }

    SCL_Write(bus, 0);
    I2C_Delay();

    return ack_status;
}

// 发送一个字节
void Soft_I2C_SendByte(Soft_I2C_Bus *bus, uint8_t byte)
{
    uint8_t i;
    SCL_Write(bus, 0);
    for (i = 0; i < 8; i++)
    {
        if ((byte & 0x80) >> 7)
        {
            SDA_Write(bus, 1);
        }
        else
        {
            SDA_Write(bus, 0);
        }
        byte <<= 1;
        I2C_Delay();
        SCL_Write(bus, 1);
        I2C_Delay();
        SCL_Write(bus, 0);
    }
}

uint8_t Soft_I2C_ReadByte(Soft_I2C_Bus *bus, uint8_t ack)
{
    uint8_t i, byte = 0;
    SDA_Write(bus, 1); // 释放 SDA 线
    for (i = 0; i < 8; i++)
    {
        SCL_Write(bus, 0);
        I2C_Delay();
        SCL_Write(bus, 1);
        I2C_Delay();
        byte <<= 1;
        if (SDA_Read(bus))
        {
            byte |= 0x01;//byte++;
        }
        I2C_Delay();
    }
    SCL_Write(bus, 0); // 时钟拉低，准备发 ACK
    if (ack)
    {
        SDA_Write(bus, 0); // ACK：拉低 SDA 表示"收到了"
    }
    else
    {
        SDA_Write(bus, 1); // NACK：释放 SDA 表示"不要再发了"
    }
    I2C_Delay();
    SCL_Write(bus, 1); // 时钟拉高，从机采样 ACK 信号
    I2C_Delay();
    SCL_Write(bus, 0); // 时钟拉低，结束 ACK 时序
    return byte;
}


uint8_t Soft_I2C_WriteBytes(Soft_I2C_Bus *bus, uint8_t dev_addr, const uint8_t *data, uint16_t len)
{
    Soft_I2C_Start(bus);
    Soft_I2C_SendByte(bus, (dev_addr << 1) | 0x00);// 发送设备地址 + 写标志
    if (Soft_I2C_WaitAck(bus)) goto error;

    for (uint16_t i = 0; i < len; i++) {
        Soft_I2C_SendByte(bus, data[i]);
        if (Soft_I2C_WaitAck(bus)) goto error;
    }
    Soft_I2C_Stop(bus);
    return 0;

error:
    Soft_I2C_Stop(bus);
    return 1;
}

uint8_t Soft_I2C_ReadBytes(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t *buf, uint16_t len)
{
    Soft_I2C_Start(bus);
    Soft_I2C_SendByte(bus, (dev_addr << 1) | 0x01);
    if (Soft_I2C_WaitAck(bus)) goto error;

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = Soft_I2C_ReadByte(bus, i < len - 1 ? 1 : 0);
    }
    Soft_I2C_Stop(bus);
    return 0;

error:
    Soft_I2C_Stop(bus);
    return 1;
}

/* ====== 上层 ====== */

uint8_t Soft_I2C_WriteReg(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = { reg_addr, data };
    return Soft_I2C_WriteBytes(bus, dev_addr, buf, 2);
}

uint8_t Soft_I2C_ReadReg(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    if (Soft_I2C_WriteBytes(bus, dev_addr, &reg_addr, 1)) return 1;
    return Soft_I2C_ReadBytes(bus, dev_addr, data, 1);
}

uint8_t Soft_I2C_ReadRegs(Soft_I2C_Bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    if (Soft_I2C_WriteBytes(bus, dev_addr, &reg_addr, 1)) return 1;
    return Soft_I2C_ReadBytes(bus, dev_addr, buf, len);
}

