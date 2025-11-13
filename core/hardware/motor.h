#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f10x.h"

void Motor_Init(void);
void Motor_SetSpeed(int8_t speed);

#endif