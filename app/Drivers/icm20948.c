#include <stddef.h>
#include "stm32f4xx.h"
#include "icm20948.h"
#include "bsp_soft_i2c.h"
#include "bsp_exti.h"
#include "SPL_Delay.h"
#include "stm32f4xx_exti.h"

static uint8_t current_bank = 0xFFU;
static Soft_I2C_Bus *icm_bus = NULL;

volatile uint8_t ICM20948_DataReadyFlag = 0U;

static uint8_t ICM_SelectBank(uint8_t bank)
{
    if (icm_bus == NULL)
    {
        return 1U;
    }

    if (bank != current_bank)
    {
        if (Soft_I2C_WriteReg(icm_bus, ICM20948_ADDR, REG_BANK_SEL, bank) != 0U)
        {
            return 1U;
        }
        current_bank = bank;
    }

    return 0U;
}

static uint8_t ICM_WriteReg(uint8_t bank, uint8_t reg, uint8_t data)
{
    if (ICM_SelectBank(bank) != 0U)
    {
        return 1U;
    }

    return Soft_I2C_WriteReg(icm_bus, ICM20948_ADDR, reg, data);
}

static uint8_t ICM_ReadReg(uint8_t bank, uint8_t reg, uint8_t *data)
{
    if (ICM_SelectBank(bank) != 0U)
    {
        return 1U;
    }

    return Soft_I2C_ReadReg(icm_bus, ICM20948_ADDR, reg, data);
}

static uint8_t ICM_ReadRegs(uint8_t bank, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (ICM_SelectBank(bank) != 0U)
    {
        return 1U;
    }

    return Soft_I2C_ReadRegs(icm_bus, ICM20948_ADDR, reg, buf, len);
}

uint8_t ICM20948_ReadIntStatus1(uint8_t *status)
{
    uint8_t dummy = 0U;

    if (status == NULL || icm_bus == NULL)
    {
        return 1U;
    }

    if (ICM_ReadReg(BANK_0, REG_INT_STATUS_1, status) != 0U)
    {
        return 1U;
    }

    if (ICM_ReadReg(BANK_0, REG_INT_STATUS, &dummy) != 0U)
    {
        return 1U;
    }

    return 0U;
}

uint8_t ICM20948_Init(Soft_I2C_Bus *bus)
{
    uint8_t who_am_i = 0U;
    uint8_t dummy = 0U;

    if (bus == NULL)
    {
        return 1U;
    }

    icm_bus = bus;
    current_bank = 0xFFU;

    Soft_I2C_Init(icm_bus);
    SPL_Delay_ms(10);

    if (ICM_ReadReg(BANK_0, REG_WHO_AM_I, &who_am_i) != 0U)
    {
        return 1U;
    }

    if (who_am_i != ICM20948_ID)
    {
        return 1U;
    }

    if (ICM_WriteReg(BANK_0, REG_PWR_MGMT_1, 0x80U) != 0U)
    {
        return 1U;
    }
    SPL_Delay_ms(50);
    current_bank = 0xFFU;

    if (ICM_WriteReg(BANK_0, REG_PWR_MGMT_1, CLK_BEST_PLL | WAKE_UP) != 0U)
    {
        return 1U;
    }
    SPL_Delay_ms(10);

    if (ICM_WriteReg(BANK_0, REG_PWR_MGMT_2, 0x00U) != 0U)
    {
        return 1U;
    }

    if (ICM_WriteReg(BANK_0, REG_INT_PIN_CFG, ICM20948_INT1_ACTIVE_HIGH_PP_LATCH) != 0U)
    {
        return 1U;
    }

    if (ICM_WriteReg(BANK_0, REG_INT_ENABLE_1, 0x01U) != 0U)
    {
        return 1U;
    }

    (void)ICM_ReadReg(BANK_0, REG_INT_STATUS_1, &dummy);
    (void)ICM_ReadReg(BANK_0, REG_INT_STATUS, &dummy);

    if (ICM_WriteReg(BANK_2, REG_GYRO_CONFIG_1, GYRO_CFG_500DPS_50HZ) != 0U)
    {
        return 1U;
    }

    if (ICM_WriteReg(BANK_2, REG_ACCEL_CONFIG, ACCEL_CFG_2G_50HZ) != 0U)
    {
        return 1U;
    }

    /* Gyro ODR ≈ 102.27Hz */
    ICM_WriteReg(BANK_2, REG_GYRO_SMPLRT_DIV, 10);

    /* Accel ODR ≈ 102.27Hz */
    ICM_WriteReg(BANK_2, REG_ACCEL_SMPLRT_DIV_1, 0x00);
    ICM_WriteReg(BANK_2, REG_ACCEL_SMPLRT_DIV_2, 10);

    return ICM_SelectBank(BANK_0);
}

void ICM20948_SyncIntAfterExtiInit(void)
{
    uint8_t status = 0U;

    ICM20948_DataReadyFlag = 0U;
    EXTI_ClearITPendingBit(EXTI_Line12);
    (void)ICM20948_ReadIntStatus1(&status);
    EXTI_ClearITPendingBit(EXTI_Line12);

    if (ICM20948_EXTI_IsIntPinHigh())
    {
        ICM20948_DataReadyFlag = 1U;
    }
}

uint8_t ICM20948_ReadData(ICM20948_Data_t *data)
{
    uint8_t buf[14];
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    int16_t temp_raw;

    if (icm_bus == NULL || data == NULL)
    {
        return 1U;
    }

    if (ICM_ReadRegs(BANK_0, REG_ACCEL_XOUT_H, buf, 14U) != 0U)
    {
        return 1U;
    }

    ax_raw = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    ay_raw = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    az_raw = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
    gx_raw = (int16_t)((uint16_t)buf[6] << 8 | buf[7]);
    gy_raw = (int16_t)((uint16_t)buf[8] << 8 | buf[9]);
    gz_raw = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
    temp_raw = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);

    data->accel_x = (float)ax_raw / 16384.0f;
    data->accel_y = (float)ay_raw / 16384.0f;
    data->accel_z = (float)az_raw / 16384.0f;
    data->gyro_x = (float)gx_raw / 65.5f;
    data->gyro_y = (float)gy_raw / 65.5f;
    data->gyro_z = (float)gz_raw / 65.5f;
    data->temp = (float)temp_raw / 333.87f + 21.0f;

    return 0U;
}
