#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f10x.h"

void PWM_Init(void);
uint16_t PWM_GetTIMCount(void);
void PWM_SetCompare(uint16_t);
uint16_t PWM_GetCompare(void);
void PWM_SetPrescaler(uint16_t prescaler);

#endif
