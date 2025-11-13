#include "stm32f10x.h"                  // Device header
#include <stdio.h>

// 板级功能
#include "bsp_delay.h"
#include "bsp_timer.h"
// #include "bsp_i2c.h"

// 外设模块
#include "key.h"
// #include "led.h"
#include "oled.h"
#include "serial.h"
#include "counter_sensor.h"
#include "encoder_sensor.h"
#include "mpu6050_sensor.h"
// #include "pwm.h"
// #include "servo.h"
// #include "motor.h"
// #include "ic.h"
#include "ad.h"
#include "dma.h"

// uint8_t keyNum = 0;
// uint16_t keyClickTotal = 0;
// int8_t speed = 0;
// int8_t speed_inc = 10; 
// uint16_t advalue;

// uint8_t dataA[] = {0x01, 0x02, 0x03, 0x04};
// uint8_t dataB[] = {0, 0, 0, 0};



int main(void)
{
	// 配置 NVIC 优先级分组 (全局只需配置一次)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Key_Init();
	Timer_Init();
	// LED_Init();
	OLED_Init();
	CounterSensor_Init();
	// EncoderSensor_Init();
	// PWM_Init();
	// Servo_Init();
	// Motor_Init();
	// IC_Init();
	// AD_Init();
	Serial_Init();
	// MyI2C_Init();
	MPU6050_Init();

	// MyI2C_Start();
	// MyI2C_SendByte(0xD1); // MPU6050 的从机地址: 0xD0 = 1101 0000 = 0110 1000 << 1 = 0x68 << 1
	// uint8_t ack = MyI2C_ReceiveAck();
	// MyI2C_Stop();
	// printf("MPU6050 ACK: %d\r\n", ack);
	// OLED_ShowNum(3, 1, ack, 3);
	uint8_t mpuID = MPU6050_GetID();
	OLED_ShowString(1, 1, "MPU6050:");
	OLED_ShowHexNum(1, 9, mpuID, 2);

	// Servo_SetAngle(90)
	
	// // PA0 输出 频率 1KHz, 占空比 70% 的 PWM信号
	// PWM_SetPrescaler(720 - 1); // Freq = 72MHz / (PSC + 1) = 72MHz / 720 = 100KHz
	// PWM_SetCompare(70); // Duty = CCR / (ARR + 1) = CCR / 100 = 70 / 100 = 70%

	uint32_t counter = 0;

	// OLED_ShowHexNum(3, 1, dataA[0], 2);
	// OLED_ShowHexNum(3, 4, dataA[1], 2);
	// OLED_ShowHexNum(3, 7, dataA[2], 2);
	// OLED_ShowHexNum(3, 10, dataA[3], 2);

	// OLED_ShowHexNum(3, 1, dataB[0], 2);
	// OLED_ShowHexNum(3, 4, dataB[1], 2);
	// OLED_ShowHexNum(3, 7, dataB[2], 2);
	// OLED_ShowHexNum(3, 10, dataB[3], 2);
	// MyDMA_Init((uint32_t)dataA, (uint32_t)dataB, 16);

	MPU6050_Data_t mpu = {0};

	while(1)
	{
		// keyNum = Key_GetNum();
		// if (keyNum == 1)
		// {
		// // 	LED1_Turn();

		// 	keyClickTotal++;
		// // 	OLED_ShowNum(1, 1, keyClickTotal, 4);
		// // 	OLED_ShowSignedNum(2, 1, keyClickTotal, 4);
		// // 	OLED_ShowBinNum(3, 1, keyClickTotal, 4);
		// // 	OLED_ShowHexNum(4, 1, 0xABCD1234, 8);
		// // 	OLED_ShowChar(4, 10, 'A');
		
		// 	if (speed > 60) {
		// 		speed_inc = -10;
		// 	} else if (speed < -60) {
		// 		speed_inc = 10;
		// 	}
			
		// 	speed += speed_inc;

		// 	Motor_SetSpeed(speed);
		// }

		// counter = CounterSensor_GetCount();
		// OLED_ShowString(1, 1, "Counter:");
		// OLED_ShowNum(1, 9, counter, 6);

		// if (counter % 6 == 1) {
		// 	Servo_SetAngle(0);
		// } else if (counter % 6 == 2) {
		// 	Servo_SetAngle(45);
		// } else if (counter % 6 == 3) {
		// 	Servo_SetAngle(90);
		// } else if (counter % 6 == 4) {
		// 	Servo_SetAngle(135);
		// } else if (counter % 6 == 5) {
		// 	Servo_SetAngle(180);
		// }

		// OLED_ShowString(2, 1, "Encoder:");
		// OLED_ShowNum(2, 9, EncoderSensor_GetCountWithReset(), 6);

		// OLED_ShowString(2, 1, "Timer:");
		// OLED_ShowNum(2, 7, Timer_GetCount(), 6);

		if (mpuID > 0) {
			MPU6050_GetData(&mpu);
			// printf("AccX: %d\r\n", mpu.AccX);
			// printf("AccY: %d\r\n", mpu.AccY);
			// printf("AccZ: %d\r\n", mpu.AccZ);
			// printf("GyroX: %d\r\n", mpu.GyroX);
			// printf("GyroY: %d\r\n", mpu.GyroY);
			// printf("GyroZ: %d\r\n", mpu.GyroZ);
			// printf("Temp: %d\r\n", mpu.Temp);
			// Delay_ms(1000);
			OLED_ShowSignedNum(2, 1, mpu.AccX, 5);
			OLED_ShowSignedNum(3, 1, mpu.AccY, 5);
			OLED_ShowSignedNum(4, 1, mpu.AccZ, 5);
			OLED_ShowSignedNum(2, 8, mpu.GyroX, 5);
			OLED_ShowSignedNum(3, 8, mpu.GyroY, 5);
			OLED_ShowSignedNum(4, 8, mpu.GyroZ, 5);
		}



		// printf("12345678901234567890.456: %zu bytes\r\n", sizeof((float)12345678901234567890.456));
		// printf("12345678901234567890.456: %zu bytes\r\n", sizeof(12345678901234567890.456));

		// OLED_ShowHexNum(4, 1, dataB[0], 2);
		// OLED_ShowHexNum(4, 4, dataB[1], 2);
		// OLED_ShowHexNum(4, 7, dataB[2], 2);
		// OLED_ShowHexNum(4, 10, dataB[3], 2);
		// Delay_ms(1000);
		// MyDMA_Transfer();

		// dataA[0]++;
		// dataA[1]++;
		// dataA[2]++;
		// dataA[3]++;

		// OLED_ShowString(1, 1, "ADC0:");
		// advalue = AD_GetValue();
		// OLED_ShowNum(1, 6, advalue, 6);
		// OLED_ShowNum(1, 6, AD_GetValueWithChannel(ADC_Channel_0), 4);
		// OLED_ShowString(2, 1, "ADC1:");
		// OLED_ShowNum(2, 6, AD_GetValueWithChannel(ADC_Channel_1), 4);
		// OLED_ShowString(3, 1, "ADC2:");
		// OLED_ShowNum(3, 6, AD_GetValueWithChannel(ADC_Channel_2), 4);
		// OLED_ShowString(4, 1, "ADC3:");
		// OLED_ShowNum(4, 6, AD_GetValueWithChannel(ADC_Channel_3), 4);

		// OLED_ShowString(4, 1, "Volate:    V");
		// char volateString[6];
		// sprintf(volateString, "%.2f", (float)advalue / 4095 * 3.3);
		// OLED_ShowString(4, 8, volateString);

		// OLED_ShowString(3, 1, "Freq:00000Hz");
		// OLED_ShowNum(3, 6, IC_GetFreq(), 5);

		// OLED_ShowString(4, 1, "Duty:00%");
		// OLED_ShowNum(4, 6, IC_GetDuty(), 2);
		// // float 转 字符串
		// float duty = IC_GetDutyFloat();
		// char dutyString[6];
		// sprintf(dutyString, ",%.2f", duty);
		// OLED_ShowString(4, 9, dutyString);

		// OLED_ShowString(3, 1, "Servo:");
		// OLED_ShowNum(3, 7, Servo_getAngle(), 3);

		// OLED_ShowString(3, 1, "MOTOR:");
		// OLED_ShowSignedNum(3, 7, speed, 2);
		// OLED_ShowString(3, 10, "->");
		// OLED_ShowSignedNum(3, 12, speed_inc, 2);

		// for (i = 0; i < 100; i++) {
		// 	TIM_SetCompare1(TIM2, i); // 设置 CCR 的值
		// 	Delay_ms(10);
		// }

		// for (i = 0; i < 100; i++) {
		// 	TIM_SetCompare1(TIM2, 100 - i); // 设置 CCR 的值
		// 	Delay_ms(10);
		// }
	}
}
