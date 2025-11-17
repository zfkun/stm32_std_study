#ifndef __AD_H__
#define __AD_H__

#include "stm32f10x.h"

void AD_Init(void);
uint16_t AD_GetValue(void);
uint16_t  AD_GetValueWithChannel(uint8_t ADC_Channel);

#endif
