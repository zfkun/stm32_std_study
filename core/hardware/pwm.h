#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f10x.h"

#define PWM_TIM    TIM2

typedef struct {
    uint16_t ARR;               // 自动重装器
    uint16_t PSC;               // 预分频器
    uint16_t CCR;               // 输出比较
    GPIO_TypeDef* GPIO_port;    // GPIO端口
    uint16_t GPIO_pin;          // GPIO引脚
} PWM_Config;

void PWM_Init(void);

uint16_t PWM_GetTIMCount(void);

void PWM_SetCompare1(uint16_t);
void PWM_SetCompare2(uint16_t);

uint16_t PWM_GetCompare1(void);
uint16_t PWM_GetCompare2(void);

void PWM_SetPrescaler(uint16_t prescaler);

#endif
