#ifndef __COUNTER_SENSOR_H__
#define __COUNTER_SENSOR_H__

#include "stm32f10x.h"

void CounterSensor_Init(void);
void CounterSensor_Reset(void);
uint32_t CounterSensor_GetCount(void);
uint32_t CounterSensor_GetCountWithReset(void);

#endif