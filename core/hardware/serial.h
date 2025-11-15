#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "stm32f10x.h"

void Serial_Init(void);

void Serial_SendByte(uint8_t);
void Serial_SendBytes(uint8_t *bytes, uint16_t length);
void Serial_SendString(char *str);
void Serial_SendNumberByte(uint32_t number, uint8_t length);
void Serial_SendNumber(uint32_t number);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

void Serial_Printf(char *fmt, ...);
void Serial_Sprintf(char *fmt, ...);

#endif