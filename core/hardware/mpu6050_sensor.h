#ifndef __MPU6050_SENSOR_H__
#define __MPU6050_SENSOR_H__

#include "stm32f10x.h"

// #define MPU6050_I2C_HW 1	// 使用硬件I2C

typedef struct
{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
    int16_t Temp;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
} MPU6050_Data_t;

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(MPU6050_Data_t *Data);

#endif