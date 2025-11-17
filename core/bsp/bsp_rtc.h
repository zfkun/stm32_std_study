#ifndef __BSP_RTC_H
#define __BSP_RTC_H

#include "stm32f10x.h"

typedef struct
{
    uint16_t Year;
    uint8_t Month;
    uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
} MyRTC_TimeTypeDef;

extern MyRTC_TimeTypeDef MyRTC_Time;

void MyRTC_Init(void);
void MyRTC_SetTime(void);
void MyRTC_ReadTime(void);

#endif
