// HC-04 蓝牙串口模块

#include <stdarg.h>
#include "bsp_utils.h"
#include "bt_serial.h"

static volatile char _rxData[100];	//接收数据
static volatile uint8_t _rxFlag;	//接收标志位

void BTSerial_Init(void)
{
	// 1. 配置 时钟 (USART, GPIO)
	Utils_USART_CLOCK_Enable(BTSERIAL_USART);
    Utils_GPIO_CLOCK_Enable(BTSERIAL_TX_PORT);
    Utils_GPIO_CLOCK_Enable(BTSERIAL_RX_PORT);
	
	// 2. 配置 GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = BTSERIAL_TX_PIN;        // TX 引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // 复用推挽输出
    GPIO_Init(BTSERIAL_TX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BTSERIAL_RX_PIN;        // RX 引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 上拉输入
	GPIO_Init(BTSERIAL_RX_PORT, &GPIO_InitStructure);
	
	// 3. 配置 USART
	USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = BTSERIAL_BAUDRATE; // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式 和 接收模式 都开启
    USART_Init(BTSERIAL_USART, &USART_InitStructure);
	
	// 4. 配置 接收中断
	USART_ITConfig(BTSERIAL_USART, USART_IT_RXNE, ENABLE);						//开启串口接收数据的中断
	
	// 5. 配置 NVIC
    // // 5.1 配置 NVIC 分组 (全局只需配置一次)
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	// 5.2 配置 NVIC
	NVIC_InitTypeDef NVIC_InitStructure;					                	//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = Utils_USART_GetIRQn(BTSERIAL_USART);	//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                	//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;					//指定NVIC线路的抢占优先级为3 (较低优先级, 避免抢占更重要的定时器中断)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		       				//指定NVIC线路的响应优先级为0
	NVIC_Init(&NVIC_InitStructure);							        			//将结构体变量交给NVIC_Init，配置NVIC外设
	
	USART_Cmd(BTSERIAL_USART, ENABLE);
}

void BTSerial_SendByte(uint8_t Byte)
{
	USART_SendData(BTSERIAL_USART, Byte);
	while (USART_GetFlagStatus(BTSERIAL_USART, USART_FLAG_TXE) == RESET);
}

void BTSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BTSerial_SendByte(Array[i]);
	}
}

void BTSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BTSerial_SendByte(String[i]);
	}
}

uint32_t BTSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BTSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BTSerial_SendByte(Number / BTSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

uint8_t BTSerial_GetRxFlag(void) {
	if (_rxFlag == 1)			//如果标志位为1
	{
		_rxFlag = 0;
		return 1;				//则返回1，并自动清零标志位
	}
	return 0;					//如果标志位为0，则返回0
}

char* BTSerial_GetRxData(void) {
	return (char *)_rxData;
}

void BTSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BTSerial_SendString(String);
}

#ifdef BTSERIAL_USART_IRQ_HANDLER
void BTSERIAL_USART_IRQ_HANDLER(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(BTSERIAL_USART, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(BTSERIAL_USART);
		
		if (RxState == 0)
		{
			if (RxData == '[' && _rxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == ']')
			{
				RxState = 0;
				_rxData[pRxPacket] = '\0';
				_rxFlag = 1;
			}
			else
			{
				_rxData[pRxPacket] = RxData;
				pRxPacket ++;
			}
		}
		
		USART_ClearITPendingBit(BTSERIAL_USART, USART_IT_RXNE);
	}
}
#endif
