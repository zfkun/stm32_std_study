// I2C (硬件)

#include "stm32f10x.h"                  // Device header

#include "bsp_delay.h"
#include "bsp_i2c_hw.h"

#ifndef HW_I2C_CHANNEL
#define HW_I2C_CHANNEL I2C2
#endif

#ifndef HW_I2C_SCL_PORT
#define HW_I2C_SCL_PORT GPIOB
#endif

#ifndef HW_I2C_SDA_PORT
#define HW_I2C_SDA_PORT GPIOB
#endif

#ifndef HW_I2C_SCL_PIN
#define HW_I2C_SCL_PIN GPIO_Pin_10
#endif

#ifndef HW_I2C_SDA_PIN
#define HW_I2C_SDA_PIN GPIO_Pin_11
#endif


static void _clock_i2c_init(I2C_TypeDef* I2Cx) {
    if (I2Cx == I2C1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    }
    else if (I2Cx == I2C2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    }
}

static void _clock_gpio_init(GPIO_TypeDef* GPIOx) {
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

static void _waitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
      // TODO: 超时处理
      // ...
			break;
		}
	}
}


void HwI2C_Init(void)
{
	// 1. 开启时钟 (I2C2, GPIO)
  _clock_i2c_init(HW_I2C_CHANNEL);
  _clock_gpio_init(HW_I2C_SCL_PORT);
  if (HW_I2C_SDA_PORT != HW_I2C_SCL_PORT) _clock_gpio_init(HW_I2C_SDA_PORT);

	// 2. 配置 GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // 必须配置为**复用**开漏输出 (开漏输出也支持输入, 控制权交给硬件外设)
	GPIO_InitStructure.GPIO_Pin = HW_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HW_I2C_SCL_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // 必须配置为**复用**开漏输出 (开漏输出也支持输入, 控制权交给硬件外设)
	GPIO_InitStructure.GPIO_Pin = HW_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HW_I2C_SDA_PORT, &GPIO_InitStructure);

  // 3. 配置 I2C
  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                                // I2C模式
  I2C_InitStructure.I2C_ClockSpeed = 50000;                                 // 时钟速度，设置为 50KHz (标准: 最大100kHz, 快速: 最大400kHz)
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                        // 时钟占空比，选择 Tlow/Thigh = 2 (时钟速度不超过标准时, 固定为 1:1)
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                               // 启用应答
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 应答地址，选择7位，从机模式下才有效
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;                                 // 自身地址，从机模式下才有效
  I2C_Init(HW_I2C_CHANNEL, &I2C_InitStructure);

  // 4. 启动 I2C
  I2C_Cmd(HW_I2C_CHANNEL, ENABLE);
}


void HwI2C_Start(void)
{
    I2C_GenerateSTART(HW_I2C_CHANNEL, ENABLE);
    _waitEvent(HW_I2C_CHANNEL, I2C_EVENT_MASTER_MODE_SELECT);
}

void HwI2C_Stop(void)
{
	I2C_GenerateSTOP(HW_I2C_CHANNEL, ENABLE);
}

void HwI2C_SendAddress(uint8_t addr, uint8_t dir, uint32_t event)
{
  I2C_Send7bitAddress(HW_I2C_CHANNEL, addr, dir);
  _waitEvent(HW_I2C_CHANNEL, event);
}

void HwI2C_SendData(uint8_t data, uint32_t event)
{
	I2C_SendData(HW_I2C_CHANNEL, data);
  _waitEvent(HW_I2C_CHANNEL, event);
}

uint8_t HwI2C_ReceiveData(void)
{
  uint8_t data;

	_waitEvent(HW_I2C_CHANNEL, I2C_EVENT_MASTER_BYTE_RECEIVED);   //等待EV7 
	data = I2C_ReceiveData(HW_I2C_CHANNEL);

  return data;
}

void HwI2C_Ack(FunctionalState state)
{
	I2C_AcknowledgeConfig(HW_I2C_CHANNEL, state);
}
