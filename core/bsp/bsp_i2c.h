#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "stm32f10x.h"

#define I2C_DELAY_DISABLE 1     // 关闭 SCL / SDA 收发时的延迟等待

#ifndef I2C_SCL_PORT
#define I2C_SCL_PORT GPIOB
#endif

#ifndef I2C_SDA_PORT
#define I2C_SDA_PORT GPIOB
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN GPIO_Pin_10
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN GPIO_Pin_11
#endif

#ifdef _I2C_C_
    #define _I2C_EXT_
#else
    #define _I2C_EXT_ extern
#endif

_I2C_EXT_ void MyI2C_Init(void);                //初始化I2C
_I2C_EXT_ void MyI2C_Start(void);               //发送I2C开始信号
_I2C_EXT_ void MyI2C_Stop(void);                //发送I2C停止信号
_I2C_EXT_ void MyI2C_SendByte(uint8_t Byte);    //I2C发送一个字节
_I2C_EXT_ uint8_t MyI2C_ReceiveByte(void);      //I2C读取一个字节
_I2C_EXT_ void MyI2C_Ack(void);                 //I2C发送ACK信号
_I2C_EXT_ void MyI2C_NAck(void);				//I2C不发送ACK信号
_I2C_EXT_ uint8_t MyI2C_WaitAck(void);          //I2C等待ACK信号

#endif
