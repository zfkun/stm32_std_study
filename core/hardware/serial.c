// USART

#include <stdio.h>
#include <stdarg.h>
#include "serial.h"

// 一些基本概念
// - 传输数据长度: uint8_t (即一次1个字节)
// - 传输数据顺序: 低位 -> 高位
//

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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX 引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 USART1
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200; // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Tx; // 只发送
    USART_Init(USART1, &USART_InitStructure);

    // 4. 使能 USART1
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