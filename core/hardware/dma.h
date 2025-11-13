#ifndef __DMA_H__
#define __DMA_H__

#include "stm32f10x.h"

void MyDMA_Init(uint32_t fromAddr, uint32_t toAddr, uint16_t size);
void MyDMA_Transfer(void);

#endif