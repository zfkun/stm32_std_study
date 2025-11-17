// SPI (软件)

#include "bsp_delay.h"
#include "bsp_spi.h"

#ifndef SPI_SS_PORT
#define SPI_SS_PORT GPIOA
#endif
#ifndef SPI_SS_PIN
#define SPI_SS_PIN GPIO_Pin_4
#endif

#ifndef SPI_SCK_PORT
#define SPI_SCK_PORT GPIOA
#endif
#ifndef SPI_SCK_PIN
#define SPI_SCK_PIN GPIO_Pin_5
#endif

#ifndef SPI_MOSI_PORT
#define SPI_MOSI_PORT GPIOA
#endif
#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN GPIO_Pin_7
#endif

#ifndef SPI_MISO_PORT
#define SPI_MISO_PORT GPIOA
#endif
#ifndef SPI_MISO_PIN
#define SPI_MISO_PIN GPIO_Pin_6
#endif

// 0: SPI模式0, 1: SPI模式1, 2: SPI模式2, 3: SPI模式3
#ifndef SPI_MODE
#define SPI_MODE 0
#endif


// 一些基本概念
// - SS: 片选信号, 也有缩写为 CS
// - SCK: 时钟信号, 也有缩写为 CLK
// - MOSI: 主机输出, 也有缩写为 DO (主机侧), DI (从机侧)
// - MISO: 主机输入, 也有缩写为 DI (主机侧), DO (从机侧)
//


/*引脚配置层*/

/**
  * 函    数：SPI写SS引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SS的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SS为低电平，当BitValue为1时，需要置SS为高电平
  */
void MySPI_W_SS(uint8_t BitValue)
{
	GPIO_WriteBit(SPI_SS_PORT, SPI_SS_PIN, (BitAction)BitValue);		//根据BitValue，设置SS引脚的电平
}

/**
  * 函    数：SPI写SCK引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SCK的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SCK为低电平，当BitValue为1时，需要置SCK为高电平
  */
void MySPI_W_SCK(uint8_t BitValue)
{
	GPIO_WriteBit(SPI_SCK_PORT, SPI_SCK_PIN, (BitAction)BitValue);		//根据BitValue，设置SCK引脚的电平
}

/**
  * 函    数：SPI写MOSI引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入MOSI的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置MOSI为低电平，当BitValue为1时，需要置MOSI为高电平
  */
void MySPI_W_MOSI(uint8_t BitValue)
{
	GPIO_WriteBit(SPI_MOSI_PORT, SPI_MOSI_PIN, (BitAction)BitValue);		//根据BitValue，设置MOSI引脚的电平，BitValue要实现非0即1的特性
}

/**
  * 函    数：I2C读MISO引脚电平
  * 参    数：无
  * 返 回 值：协议层需要得到的当前MISO的电平，范围0~1
  * 注意事项：此函数需要用户实现内容，当前MISO为低电平时，返回0，当前MISO为高电平时，返回1
  */
uint8_t MySPI_R_MISO(void)
{
	return GPIO_ReadInputDataBit(SPI_MISO_PORT, SPI_MISO_PIN);			//读取MISO电平并返回
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

/**
  * 函    数：SPI初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，实现SS、SCK、MOSI和MISO引脚的初始化
  */
void MySPI_Init(void)
{
  // 1. 配置时钟
	_clock_init(SPI_SS_PORT);
	if (SPI_SCK_PORT != SPI_SS_PORT) _clock_init(SPI_SCK_PORT);
	if (SPI_MOSI_PORT != SPI_SS_PORT) _clock_init(SPI_MOSI_PORT);
	if (SPI_MISO_PORT != SPI_SS_PORT) _clock_init(SPI_MISO_PORT);
	
	// 2. 配置 GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出 (片选)
	GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_SS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出 (时钟)
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出 (主机输出, 从机输如 口)
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 必须是上拉输入 (主机输入, 从机输出 口)
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_MISO_PORT, &GPIO_InitStructure);
	
	// 3. 设置默认状态
	MySPI_W_SS(1);											//SS默认高电平 (默认 不选中从机)
	MySPI_W_SCK(0);											//SCK默认低电平 (希望 SPI 模式0, 所以这里默认 SCK 线为低电平 )
}

/*协议层*/

/**
  * 函    数：SPI起始
  * 参    数：无
  * 返 回 值：无
  */
void MySPI_Start(void)
{
	MySPI_W_SS(0);				//拉低SS，开始时序
}

/**
  * 函    数：SPI终止
  * 参    数：无
  * 返 回 值：无
  */
void MySPI_Stop(void)
{
	MySPI_W_SS(1);				//拉高SS，终止时序
}

/**
  * 函    数：SPI交换传输一个字节，使用SPI模式0
  * 参    数：ByteSend 要发送的一个字节
  * 返 回 值：接收的一个字节
  */
uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	uint8_t i, ByteReceive = 0x00;					    //定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到
	
  //逐位交换，依次交换每一位数据
	for (i = 0; i < 8; i ++)
	{
#if SPI_MODE == 0
		/*两个!可以对数据进行两次逻辑取反，作用是把非0值统一转换为1，即：!!(0) = 0，!!(非0) = 1*/
		MySPI_W_MOSI(!!(ByteSend & (0x80 >> i))); //使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
		MySPI_W_SCK(1);								            //拉高SCK，上升沿移出数据
		if (MySPI_R_MISO()) {                     //读取MISO数据，并存储到Byte变量
      ByteReceive |= (0x80 >> i);
    }
															                //当MISO为1时，置变量指定位为1，当MISO为0时，不做处理，指定位为默认的初值0
		MySPI_W_SCK(0);								            //拉低SCK，下降沿移入数据
#elif SPI_MODE == 1
    MySPI_W_SCK(1);								            //拉高SCK，上升沿移出数据
    MySPI_W_MOSI(!!(ByteSend & (0x80 >> i))); //使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
    MySPI_W_SCK(0);								            //拉低SCK，下降沿移入数据
		if (MySPI_R_MISO()) {                     //读取MISO数据，并存储到Byte变量
      ByteReceive |= (0x80 >> i);
    }
#elif SPI_MODE == 2
    MySPI_W_MOSI(!!(ByteSend & (0x80 >> i))); //使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
		MySPI_W_SCK(0);								            //拉高SCK，上升沿移出数据
		if (MySPI_R_MISO()) {                     //读取MISO数据，并存储到Byte变量
      ByteReceive |= (0x80 >> i);
    }
		MySPI_W_SCK(1);								            //拉低SCK，下降沿移入数据
#elif SPI_MODE == 3
    MySPI_W_SCK(0);								            //拉高SCK，上升沿移出数据
    MySPI_W_MOSI(!!(ByteSend & (0x80 >> i))); //使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
    MySPI_W_SCK(1);								            //拉低SCK，下降沿移入数据
		if (MySPI_R_MISO()) {                     //读取MISO数据，并存储到Byte变量
      ByteReceive |= (0x80 >> i);
    }
#endif
	}
	
	return ByteReceive;								          //返回接收到的一个字节数据
}
