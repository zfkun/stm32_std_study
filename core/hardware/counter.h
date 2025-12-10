#ifndef __COUNTER_H__
#define __COUNTER_H__

#include "stm32f10x.h"

#define COUNTER_RCC_PERIPH      (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO) // EXTI外部中断线需要开启AFIO时钟

//////////////////////////// 左轮 ////////////////////////////

#define COUNTER_PORT_1            GPIOA
#define COUNTER_PIN_1             GPIO_Pin_7

#define COUNTER_PORT_SOURCE_1     GPIO_PortSourceGPIOA
#define COUNTER_PIN_SOURCE_1      GPIO_PinSource7

#define COUNTER_EXTI_LINE_1       EXTI_Line7            // 注意不要和 右轮 的 EXTI_LINE 一样

// STM32F103C8T6 只能选
// - EXTI9_5_IRQn    : EXTI[9:5]中断
// - EXTI15_10_IRQn  : EXTI[15:10]中断
// 前面 EXIT_LINE 分别设置了 7, 所以这里必须对应选 EXTI9_5_IRQn
#define COUNTER_NVIC_IRQ_1            EXTI9_5_IRQn
#define COUNTER_NVIC_IRQ_HANDLER_1    EXTI15_10_IRQHandler

//////////////////////////// 右轮 ////////////////////////////

#define COUNTER_PORT_2            GPIOB
#define COUNTER_PIN_2             GPIO_Pin_6

#define COUNTER_PORT_SOURCE_2     GPIO_PortSourceGPIOB
#define COUNTER_PIN_SOURCE_2      GPIO_PinSource6

#define COUNTER_EXTI_LINE_2       EXTI_Line6            // 注意不要和 左轮 的 EXTI_LINE 一样

// STM32F103C8T6 只能选
// - EXTI9_5_IRQn    : EXTI[9:5]中断
// - EXTI15_10_IRQn  : EXTI[15:10]中断
// 前面 EXIT_LINE 分别设置了 6, 所以这里必须对应选 EXTI9_5_IRQn
#define COUNTER_NVIC_IRQ_2            EXTI9_5_IRQn
#define COUNTER_NVIC_IRQ_HANDLER_2    EXTI15_10_IRQHandler


void Counter_Init(void);
void Counter_Reset(void);

uint32_t Counter_GetCount1(void);
uint32_t Counter_GetCount2(void);
uint32_t Counter_GetCount1WithReset(void);
uint32_t Counter_GetCount2WithReset(void);

void Counter_SetOnCounter1Tick(void (*callback)(void));
void Counter_SetOnCounter2Tick(void (*callback)(void));
void Counter_SetAfterCounter1Tick(void (*callback)(void));
void Counter_SetAfterCounter2Tick(void (*callback)(void));

#endif
