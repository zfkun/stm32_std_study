#ifndef __IC_H__
#define __IC_H__

#include "stm32f10x.h"

void IC_Init(void); // 输入捕获初始化

uint32_t IC_GetFreq(void); // 获取当前频率 (Hz)

uint32_t IC_GetDuty(void); // 获取当前占空比 (整数: 0 ~ 100)
float IC_GetDutyFloat(void); // 获取当前占空比 (浮点: 0 ~ 1)

#endif