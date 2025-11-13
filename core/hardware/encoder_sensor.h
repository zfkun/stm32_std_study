#ifndef __ENCODER_SENSOR_H__
#define __ENCODER_SENSOR_H__

#include "stm32f10x.h"

void EncoderSensor_Init(void);
void EncoderSensor_Reset(void);
uint16_t EncoderSensor_GetCount(void);
uint16_t EncoderSensor_GetCountWithReset(void);

#endif