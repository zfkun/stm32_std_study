#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "stm32f10x.h"                  // Device header

#define SPI_HW 1	    // 使用硬件SPI

void MySPI_Init(void);
void MySPI_Start(void);
void MySPI_Stop(void);
uint8_t MySPI_SwapByte(uint8_t ByteSend);

#endif
