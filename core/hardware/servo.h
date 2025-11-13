#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f10x.h"

void Servo_Init(void);
void Servo_SetAngle(float angle);
float Servo_getAngle(void);

#endif