#ifndef __BSP_SPI_HW_H__
#define __BSP_SPI_HW_H__

void MySPI_Init(void);
void MySPI_Start(void);
void MySPI_Stop(void);
uint8_t MySPI_SwapByte(uint8_t ByteSend);

#endif