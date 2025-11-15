// USART

#include <stdio.h>
#include <stdarg.h>
#include "serial.h"

// 一些基本概念
// - 传输数据长度: uint8_t (即一次1个字节)
// - 传输数据顺序: 低位 -> 高位
//

static uint8_t _rxData;		//接收数据
static uint8_t _rxFlag;		//接收标志位

void Serial_Init(void)
{
    // 目标:
    // - 使用 USART1
    // - 发送 和 接收 数据

    // 1. 配置 时钟 (USART1, GPIO)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置 GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           // TX 引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          // RX 引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 USART1
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200; // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式 和 接收模式 都开启
    USART_Init(USART1, &USART_InitStructure);

    // 4. 配置 接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);          //开启串口接收数据的中断

    // 5. 配置 NVIC
    // // 5.1 配置 NVIC 分组 (全局只需配置一次)
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    // 5.2 配置 NVIC
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设

    // 5. 使能 USART1
    USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t byte)
{
    // 1. 等待发送缓冲区空闲
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

    // 2. 发送数据
    USART_SendData(USART1, byte);

    // 3. 等待发送完成
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void Serial_SendBytes(uint8_t *bytes, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        Serial_SendByte(bytes[i]);
    }
}

void Serial_SendString(char *str)
{
    while (*str != '\0')
    {
        Serial_SendByte(*str);
        str++;
    }
}

void Serial_SendNumber(uint32_t number)
{
    char str[10];
    sprintf(str, "%d", number);
    Serial_SendString(str);
}

// 辅助计算 10 的 n 次方（整数版本, 避免引入浮点运算）
static uint32_t _power10(uint8_t n)
{
    uint32_t result = 1;
    while (n--)
    {
        result *= 10;
    }
    return result;
}

void Serial_SendNumberByte(uint32_t number, uint8_t length)
{
    // 数字拆分每一位 (按十进制), 按字节逐一发送
    // 数字 0 ~ 9 对应的 ASCII 码: 0x30 ~ 0x39 (即 每个数字要加上 30 偏移值)
    // 举例:
    // - 目标数字: 12345
    // - 拆分万位: 12345 / 10000 => 得到 1      => 1 % 10       => 得到 1
    // - 拆分千位: 12345 / 1000  => 得到 12     => 12 % 10      => 得到 2
    // - 拆分百位: 12345 / 100   => 得到 123    => 123 % 10     => 得到 3
    // - 拆分十位: 12345 / 10    => 得到 1234   => 1234 % 10    => 得到 4
    // - 拆分个位: 12345 / 1     => 得到 12345  => 12345 % 10   => 得到 5
    for (uint8_t i = 0; i < length; i++)
    {
        Serial_SendByte(number / _power10(length - i - 1) % 10 + 0x30);
    }
}


/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (_rxFlag == 1)			//如果标志位为1
	{
		_rxFlag = 0;
		return 1;				//则返回1，并自动清零标志位
	}
	return 0;					//如果标志位为0，则返回0
}

/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的数据，范围：0~255
  */
uint8_t Serial_GetRxData(void)
{
	return _rxData;			//返回接收的数据变量
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		_rxData = USART_ReceiveData(USART1);				    //读取数据寄存器，存放在接收的数据变量
		_rxFlag = 1;										    //置接收标志位变量为1

        // 读取数据寄存器会自动清除此标志位
		// 如果已经读取了数据寄存器，也可以不执行此代码
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);			//清除USART1的RXNE标志位
	}
}


/**
 * @brief 串口重定向 (printf 重定向到 USART1)
 *
 * @param str
 * @return int
 */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

// 无缓冲串口重定向
void Serial_Printf(char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

// 带缓冲串口重定向
void Serial_Sprintf(char *fmt, ...)
{
    char buf[256]; // 缓冲区
    va_list args;
    va_start(args, fmt);
    vsprintf(buf, fmt, args); // 写入缓冲区
    // vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial_SendString(buf); // 发送缓冲区
}