#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f10x.h"

#define BSP_TIMER_TIM				TIM1					// 定时器
#define BSP_TIMER_RCC_PERIPH		RCC_APB2Periph_TIM1		// 使能时钟
#define BSP_TIMER_NVIC_IRQ_CHANNEL	TIM1_UP_IRQn			// 中断通道 (更新中断)

// // 1kHz 配置 (即 1s 时钟)
// #define BSP_TIMER_ARR				10000 - 1				// 自动重装器的值
// #define BSP_TIMER_PSC				7200 - 1				// 预分频器的值

// 1Hz 配置 (即 1ms 时钟)
#define BSP_TIMER_ARR				1000 - 1				// 自动重装器的值
#define BSP_TIMER_PSC				72 - 1					// 预分频器的值

void Timer_Init(void);

uint16_t Timer_GetTIMCount(void);
uint16_t Timer_GetCount(void);
uint16_t Timer_GetCountWithReset(void);

void Timer_SetOnTimerTick(void (*callback)(void));
void Timer_SetAfterTimerTick(void (*callback)(void));

#endif
