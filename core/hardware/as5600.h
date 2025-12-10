#ifndef __AS5600_H__
#define __AS5600_H__

#include "stm32f10x.h"

// #define AS5600_I2C_HW 1	// 使用硬件I2C

// 原始角度转换系数 (因AS5600的 RawAngle 范围是 0 ~ 4095, 转换为角度 0 ~ 360 范围的角度值, 则 1° = 0.0872665 ≈ 0.87)
#define AS5600_DEGREE_PER_RAWANGLE 0.087	// 1° = 0.0872665 ≈ 0.87

typedef struct
{
	int16_t RawAngle;	// 原始角度: 0 ~ 4095
	int16_t Angle;		// 缩放后的原始角度: 0 ~ 4095
	int16_t ClacAngle;	// 转换后的角度: 0 ~ 360
    int16_t Status;		// 状态
	int16_t Agc;		// 自动增益
	int16_t Magnitude;	// 磁强度
} AS5600_Data_t;

void AS5600_Init(void);
uint16_t AS5600_GetRawAngle(void);
uint16_t AS5600_GetAngle(void);
uint16_t AS5600_GetClacAngle(void);
uint16_t AS5600_GetStatus(void);
uint16_t AS5600_GetMagnitude(void);
void AS5600_GetData(AS5600_Data_t *Data);

#endif
