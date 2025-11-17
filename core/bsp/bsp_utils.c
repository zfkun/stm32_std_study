
#include "bsp_utils.h"

static inline void _rccPeriphClockCmd(uint32_t RCC_Periph, FunctionalState NewState) {
	if (IS_RCC_AHB_PERIPH(RCC_Periph)) {
		RCC_AHBPeriphClockCmd(RCC_Periph, NewState);
	}
	else if (IS_RCC_APB1_PERIPH(RCC_Periph)) {
		RCC_APB1PeriphClockCmd(RCC_Periph, NewState);
	}
	else if (IS_RCC_APB2_PERIPH(RCC_Periph)) {
		RCC_APB2PeriphClockCmd(RCC_Periph, NewState);
	}
}

void Utils_RCC_PeriphClock_Enable(uint32_t RCC_Periph) {
	_rccPeriphClockCmd(RCC_Periph, ENABLE);
}

void Utils_RCC_PeriphClock_Disable(uint32_t RCC_Periph) {
	_rccPeriphClockCmd(RCC_Periph, DISABLE);
}

void Utils_GPIO_CLOCK_Enable(GPIO_TypeDef* GPIOx) {
	if (GPIOx == GPIOA) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }
    else if (GPIOx == GPIOC) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    }
    else if (GPIOx == GPIOD) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    }
    else if (GPIOx == GPIOE) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    }
    else if (GPIOx == GPIOF) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    }
    else if (GPIOx == GPIOG) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    }
}

void Utils_USART_CLOCK_Enable(USART_TypeDef *USARTx) {
	if (USARTx == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	}
	else if (USARTx == USART2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	}
	else if (USARTx == USART3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	}
	else if (USARTx == UART4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	}
	else if (USARTx == UART5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	}
}

IRQn_Type Utils_USART_GetIRQn(USART_TypeDef *USARTx) {
	if (USARTx == USART1) return USART1_IRQn;
	else if (USARTx == USART2) return USART2_IRQn;
#ifdef USART3_IRQn
	else if (UARTx == USART3) return USART3_IRQn;
#endif
	else return USART1_IRQn;
}