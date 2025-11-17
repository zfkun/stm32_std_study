#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f10x.h"

void Timer_Init(void);
uint16_t Timer_GetTIMCount(void);
uint16_t Timer_GetCount(void);
uint16_t Timer_GetCountWithReset(void);

#endif
