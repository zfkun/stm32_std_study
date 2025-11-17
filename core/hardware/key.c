// 按键检测

#include "bsp_delay.h"
#include "bsp_utils.h"
#include "key.h"

#define KEY_1_PORT				GPIOB
#define KEY_1_PIN				GPIO_Pin_14

#define KEY_2_PORT				GPIOB
#define KEY_2_PIN				GPIO_Pin_15

#define KEY_SWITCH_TICK_COUNT	20

static volatile uint8_t _keyNum = 0;


/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	// 1. 配置时钟
	Utils_GPIO_CLOCK_Enable(KEY_1_PORT);
	Utils_GPIO_CLOCK_Enable(KEY_2_PORT);
	
	// 2. 配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = KEY_1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = KEY_2_PIN;
	GPIO_Init(KEY_2_PORT, &GPIO_InitStructure);
}


uint8_t Key_GetNum(void)
{
	uint8_t k = 0;

	if (_keyNum) {
		k = _keyNum;
		_keyNum = 0;
	}

	return k;
}

uint8_t Key_GetState(void) {
	if (GPIO_ReadInputDataBit(KEY_1_PORT, KEY_1_PIN) == 0) return 1;
	if (GPIO_ReadInputDataBit(KEY_2_PORT, KEY_2_PIN) == 0) return 2;
	return 0;
}

void Key_Tick(void) {
	static uint8_t count = 0;
	static uint8_t currentState, prevState;

	count++;
	if (count >= KEY_SWITCH_TICK_COUNT) {
		count = 0;

		prevState = currentState;
		currentState = Key_GetState();

		if (currentState == 0 && prevState != 0) {
			_keyNum = prevState;
		}
	}
}