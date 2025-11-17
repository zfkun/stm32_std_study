// 板载LED

#include "bsp_utils.h"
#include "led.h"

#define LED_PORT	GPIOC
#define LED_PIN 	GPIO_Pin_13		// 板载LED

/**
  * 函    数：LED初始化
  * 参    数：无
  * 返 回 值：无
  */
void LED_Init(void)
{
	Utils_GPIO_CLOCK_Enable(LED_PORT);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);

	GPIO_SetBits(LED_PORT, LED_PIN);	// 默认熄灭
}

/**
  * 函    数：LED开启
  * 参    数：无
  * 返 回 值：无
  */
void LED_ON(void)
{
	GPIO_ResetBits(LED_PORT, LED_PIN);		//低电平点亮
}

/**
  * 函    数：LED关闭
  * 参    数：无
  * 返 回 值：无
  */
void LED_OFF(void)
{
	GPIO_SetBits(LED_PORT, LED_PIN);		//高电熄灭
}

/**
  * 函    数：LED状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED_Turn(void)
{
	if (GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
	{
		GPIO_SetBits(LED_PORT, LED_PIN);					//则设置高电平
	}
	else													//否则，即当前引脚输出高电平
	{
		GPIO_ResetBits(LED_PORT, LED_PIN);					//则设置低电平
	}
}
