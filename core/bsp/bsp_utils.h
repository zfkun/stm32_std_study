#ifndef __BSP_UTILS_H
#define __BSP_UTILS_H

#include "stm32f10x.h"                  // Device header

void Utils_RCC_PeriphClock_Enable(uint32_t RCC_Periph);
void Utils_RCC_PeriphClock_Disable(uint32_t RCC_Periph);

void Utils_GPIO_CLOCK_Enable(GPIO_TypeDef* GPIOx);

void Utils_USART_CLOCK_Enable(USART_TypeDef *USARTx);
IRQn_Type Utils_USART_GetIRQn(USART_TypeDef *USARTx);

#endif
