/*
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:17
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-05-04 04:17:21
 * @FilePath: \ros2_mecanum\BSP\OLED.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __OLED_H
#define __OLED_H
#include "bsp_soft_i2c.h"

extern Soft_I2C_Bus i2c_oled;

void OLED_Init(Soft_I2C_Bus *bus);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_WriteDataBuffer(const uint8_t *data, uint16_t len);
void OLED_UpdateBuffer(const uint8_t *buffer);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
