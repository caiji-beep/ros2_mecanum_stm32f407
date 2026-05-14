#ifndef __KEY_H
#define __KEY_H


typedef struct
{
    uint8_t Key1_State;
    uint8_t Key2_State;
} Key_Data;

void Key_Init(void);
uint8_t Key_GetNum(void);
#endif
