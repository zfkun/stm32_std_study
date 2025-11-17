// SPI (硬件)

#include "bsp_delay.h"
#include "bsp_spi_hw.h"

#ifndef MySPI_CHANNEL
#define MySPI_CHANNEL SPI1
#endif

#ifndef MySPI_SS_PORT
#define MySPI_SS_PORT GPIOA
#endif
#ifndef MySPI_SS_PIN
#define MySPI_SS_PIN GPIO_Pin_4
#endif

#ifndef MySPI_SCK_PORT
#define MySPI_SCK_PORT GPIOA
#endif
#ifndef MySPI_SCK_PIN
#define MySPI_SCK_PIN GPIO_Pin_5
#endif

#ifndef MySPI_MOSI_PORT
#define MySPI_MOSI_PORT GPIOA
#endif
#ifndef MySPI_MOSI_PIN
#define MySPI_MOSI_PIN GPIO_Pin_7
#endif

#ifndef MySPI_MISO_PORT
#define MySPI_MISO_PORT GPIOA
#endif
#ifndef MySPI_MISO_PIN
#define MySPI_MISO_PIN GPIO_Pin_6
#endif


void MySPI_W_SS(uint8_t BitValue)
{
	GPIO_WriteBit(MySPI_SS_PORT, MySPI_SS_PIN, (BitAction)BitValue);		//根据BitValue，设置SS引脚的电平
}

static void _clock_init(GPIO_TypeDef* GPIOx) {
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

static void _waitFlag(uint16_t flag, FlagStatus status)
{
	uint32_t Timeout;
	Timeout = 100000;
	while (SPI_I2S_GetFlagStatus(MySPI_CHANNEL, flag) != status)
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

void MySPI_Init(void)
{
  // 1. 配置时钟
	_clock_init(MySPI_SS_PORT);
	if (MySPI_SCK_PORT != MySPI_SS_PORT) _clock_init(MySPI_SCK_PORT);
	if (MySPI_MOSI_PORT != MySPI_SS_PORT) _clock_init(MySPI_MOSI_PORT);
	if (MySPI_MISO_PORT != MySPI_SS_PORT) _clock_init(MySPI_MISO_PORT);
	
	// 2. 配置 GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出 (片选)
	GPIO_InitStructure.GPIO_Pin = MySPI_SS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MySPI_SS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;    	// 复用输出 (时钟), 交由片上硬件控制
	GPIO_InitStructure.GPIO_Pin = MySPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MySPI_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;    	// 复用输出 (主机输出, 从机输如 口), 交由片上硬件控制
	GPIO_InitStructure.GPIO_Pin = MySPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MySPI_MOSI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 必须是上拉输入 (主机输入, 从机输出 口)
	GPIO_InitStructure.GPIO_Pin = MySPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MySPI_MISO_PORT, &GPIO_InitStructure);
	
	// 3. 配置 SPI
	SPI_Cmd(MySPI_CHANNEL, ENABLE);						//使能SPI，开始运行
	
	// 4. 设置默认状态
	MySPI_W_SS(1);										//SS默认高电平
}

void MySPI_Start(void)
{
	MySPI_W_SS(0);				//拉低SS，开始时序
}
void MySPI_Stop(void)
{
	MySPI_W_SS(1);				//拉高SS，终止时序
}

uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	_waitFlag(SPI_I2S_FLAG_TXE, SET);               //等待发送数据寄存器空
	
  SPI_I2S_SendData(SPI1, ByteSend);					//写入数据到发送数据寄存器，开始产生时序

  _waitFlag(SPI_I2S_FLAG_RXNE, SET);				//等待接收数据寄存器非空
  
  return SPI_I2S_ReceiveData(SPI1);					//读取接收到的数据并返回
}
