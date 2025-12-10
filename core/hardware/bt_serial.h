#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include "stm32f10x.h"
#include <stdio.h>

#define BTSERIAL_USART                USART2
#define BTSERIAL_USART_IRQ_HANDLER    USART2_IRQHandler

#define BTSERIAL_BAUDRATE             9600

#define BTSERIAL_TX_PORT              GPIOA
#define BTSERIAL_TX_PIN               GPIO_Pin_2

#define BTSERIAL_RX_PORT              GPIOA
#define BTSERIAL_RX_PIN               GPIO_Pin_3

void BTSerial_Init(void);
void BTSerial_SendByte(uint8_t Byte);
void BTSerial_SendArray(uint8_t *Array, uint16_t Length);
void BTSerial_SendString(char *String);
void BTSerial_SendNumber(uint32_t Number, uint8_t Length);
void BTSerial_Printf(char *format, ...);

uint8_t BTSerial_GetRxFlag(void);
char* BTSerial_GetRxData(void);

#endif
