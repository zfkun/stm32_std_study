// SG90 舵机

#include "bsp_math.h"
#include "pwm.h"
#include "servo.h"

void Servo_Init(void)
{
    PWM_Init();
}


/**
 * 设置舵机角度
 * 
 * 角度范围 0 ~ 180
 * CCR范围 500 ~ 2500
 * 
 * @param angle 角度
 */
void Servo_SetAngle(float angle)
{
    PWM_SetCompare(map(angle, 0, 180, 500, 2500));
}

float Servo_getAngle(void)
{
    return map(PWM_GetCompare(), 500, 2500, 0, 180);
}
