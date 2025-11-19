#include "stm32f10x.h"                  // Device header
#include <stdio.h>

// 板级功能
#include "bsp_delay.h"
#include "bsp_timer.h"
// #include "bsp_rtc.h"
// #include "bsp_i2c.h"

// 外设模块
#include "key.h"
// #include "led.h"
#include "oled.h"
#include "serial.h"
// #include "counter_sensor.h"
// #include "encoder_sensor.h"
#include "mpu6050_sensor.h"
#include "as5600_sensor.h"
// #include "pwm.h"
// #include "servo.h"
// #include "motor.h"
// #include "ic.h"
// #include "ad.h"
// #include "dma.h"

uint8_t keyNum = 0;
uint16_t keyClickTotal = 0;
// int8_t speed = 0;
// int8_t speed_inc = 10; 
// uint16_t advalue;

// uint8_t dataA[] = {0x01, 0x02, 0x03, 0x04};
// uint8_t dataB[] = {0, 0, 0, 0};


uint16_t timer_count = 0;

MPU6050_Data_t mpu = {0};

void onTimerUpdate(void) {
	Key_Tick();
	MPU6050_GetData(&mpu);
}

void afterTimerUpdate(void) {
	timer_count = Timer_GetTIMCount(); // 调试查看中断耗时
}


int main(void)
{
	// 配置 NVIC 优先级分组 (全局只需配置一次)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Key_Init();

	Timer_SetOnTimerTick(onTimerUpdate);
	Timer_SetAfterTimerTick(afterTimerUpdate);
	Timer_Init();

	// LED_Init();
	OLED_Init();
	// MyRTC_Init();
	// CounterSensor_Init();
	// EncoderSensor_Init();
	// PWM_Init();
	// Servo_Init();
	// Motor_Init();
	// IC_Init();
	// AD_Init();
	Serial_Init();
	// MyI2C_Init();
	MPU6050_Init();
	// AS5600_Init();

	// printf("BKP:%x\r\n", BKP_ReadBackupRegister(BKP_DR1));
	// OLED_ShowHexNum(1, 1, BKP_ReadBackupRegister(BKP_DR1), 4);

	// MyI2C_Start();
	// MyI2C_SendByte(0xD1); // MPU6050 的从机地址: 0xD0 = 1101 0000 = 0110 1000 << 1 = 0x68 << 1
	// uint8_t ack = MyI2C_ReceiveAck();
	// MyI2C_Stop();
	// printf("MPU6050 ACK: %d\r\n", ack);
	// OLED_ShowNum(3, 1, ack, 3);
	// uint8_t mpuID = MPU6050_GetID();
	// float mpuTempture = 0.0f;
	// OLED_Printf(1, 1, OLED_6X8, "MPU6050: %d = %d", mpuID, MPU6050_SLAVE_ADDR >> 1);
	// if (mpuID == MPU6050_SLAVE_ADDR >> 1) {
	// 	mpu_ready = 1;
	// }
	// OLED_ShowHexNum(1, 9, mpuID, 2, OLED_6X8);

	// Servo_SetAngle(90)
	
	// // PA0 输出 频率 1KHz, 占空比 70% 的 PWM信号
	// PWM_SetPrescaler(720 - 1); // Freq = 72MHz / (PSC + 1) = 72MHz / 720 = 100KHz
	// PWM_SetCompare(70); // Duty = CCR / (ARR + 1) = CCR / 100 = 70 / 100 = 70%

	// uint32_t counter = 0;

	// OLED_ShowHexNum(3, 1, dataA[0], 2);
	// OLED_ShowHexNum(3, 4, dataA[1], 2);
	// OLED_ShowHexNum(3, 7, dataA[2], 2);
	// OLED_ShowHexNum(3, 10, dataA[3], 2);

	// OLED_ShowHexNum(3, 1, dataB[0], 2);
	// OLED_ShowHexNum(3, 4, dataB[1], 2);
	// OLED_ShowHexNum(3, 7, dataB[2], 2);
	// OLED_ShowHexNum(3, 10, dataB[3], 2);
	// MyDMA_Init((uint32_t)dataA, (uint32_t)dataB, 16);

	// now = 1672588795;
	// OLED_ShowString(2, 1, "Time:");
	// // now_tm = gmtime(&now);
	// now_tm = localtime(&now);
	// // OLED_ShowString(2, 6, ctime(&now));
	// // printf("Time: %s", asctime(now_tm));
	// printf("Year: %d\r\n", now_tm->tm_year + 1900);
	// printf("Month: %d\r\n", now_tm->tm_mon + 1);
	// printf("Day: %d\r\n", now_tm->tm_mday);
	// printf("Hour: %d\r\n", now_tm->tm_hour);
	// printf("Minute: %d\r\n", now_tm->tm_min);
	// printf("Second: %d\r\n", now_tm->tm_sec);

	// now = mktime(&now_tm);
	// printf("Now: %d\r\n", now);

	// OLED_ShowString(1, 1, "Date:xxxx-xx-xx");
	// OLED_ShowString(2, 1, "Time:xx:xx:xx");
	// OLED_ShowString(3, 1, "CNT:");
	// OLED_ShowString(4, 1, "DIV:");

	// OLED_ShowString(1, 1, "SYSCLK:");
	// OLED_ShowNum(1, 8, SystemCoreClock, 8);

	// // 停止 systick 定时器
	// SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

	// //开启PWR的时钟 (停止模式 和 待机模式 一定要记得开启)
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	// OLED_ShowString(1, 1, "CNT:");
	// OLED_ShowString(2, 1, "ALRM:");
	// OLED_ShowString(3, 1, "ALRF:");

	// PWR_WakeUpPinCmd(ENABLE); //使能位于PA0的WKUP引脚，WKUP引脚上升沿唤醒待机模式

	// // 设置闹钟
	// uint32_t alarmVal = RTC_GetCounter() + 10;
	// RTC_SetAlarm(alarmVal);
	// OLED_ShowNum(2, 6, alarmVal, 10);

	// printf("alarmVal: %d\r\n", alarmVal);

	// uint8_t rxData;
	
	// AS5600_Data_t angle;

	// OLED_Printf(0, 0, OLED_6X8, "Accel");
	// OLED_Printf((4 + 5) * OLED_6X8, 0, OLED_6X8, "Gyro");

	while(1)
	{
		// AS5600_GetData(&angle);
		// OLED_Printf(1 * OLED_6X8, 0, OLED_6X8, "RawAngle: %d", angle.RawAngle);
		// OLED_Printf(4 * OLED_6X8, 8, OLED_6X8, "Angle: %d", angle.Angle);
		// OLED_Printf(0, 16, OLED_6X8, "ClacAngle: %d", angle.ClacAngle);
		// OLED_Printf(3 * OLED_6X8, 24, OLED_6X8, "Status: %x", angle.Status);
		// OLED_Printf(6 * OLED_6X8, 32, OLED_6X8, "AGC: %d", angle.Agc);
		// OLED_Printf(0, 40, OLED_6X8, "Magnitude: %06d", angle.Magnitude);
		// OLED_Update();

		// // Blink
		// LED_ON();
		// Delay_ms(500);
		// LED_OFF();
		// Delay_ms(500);
		// LED_Turn();
		// Delay_ms(500);
		// LED_Turn();
		// Delay_ms(500);

		// printf("原始角度: %d\r\n", angle.RawAngle);
		// printf("缩放原始角度: %d\r\n", angle.Angle);
		// printf("转换角度: %d\r\n", angle.ClacAngle);
		// printf("状态: 0x%X\r\n", angle.Status);
		// printf("  - 检测到磁铁: %d\r\n", angle.Status & 0x20 ? 1 : 0);
		// printf("  - 磁铁太强: %d\r\n", angle.Status & 0x08 ? 1 : 0);
		// printf("  - 磁铁太弱: %d\r\n", angle.Status & 0x10 ? 1 : 0);
		// printf("自动增益: %d\r\n", angle.Agc);
		// printf("磁场强度: %d\r\n", angle.Magnitude);

		// if (Serial_GetRxFlag() == 1) {
		// 	rxData = Serial_GetRxData();
		// 	Serial_SendByte(rxData);
		// 	OLED_ShowHexNum(0, 8, rxData, 2, OLED_6X8);
		// 	OLED_ShowChar(0, 16, rxData, OLED_6X8);
		// }
		
		// OLED_ShowString(4, 1, "Runing");
		// Delay_ms(100);
		// OLED_ShowString(4, 1, "      ");
		// Delay_ms(100);

		// __WFI();	//执行WFI指令，CPU睡眠，并等待中断唤醒

		// OLED_ShowNum(1, 6, RTC_GetCounter(), 10);
		// OLED_ShowNum(3, 6, RTC_GetFlagStatus(RTC_FLAG_ALR), 1);

		// OLED_ShowString(4, 1, "Runing");
		// Delay_ms(100);
		// OLED_ShowString(4, 1, "      ");
		// Delay_ms(100);

		// OLED_ShowString(4, 9, "Strandby");
		// Delay_ms(1000);
		// OLED_ShowString(4, 9, "        ");
		// Delay_ms(100);

		// OLED_Clear();

		
		// PWR_EnterSTANDBYMode(); // 进入 STANDBY 模式

		// __WFI(); // 进入 WFI 模式

		// MyRTC_ReadTime();

		// OLED_ShowNum(1, 6, MyRTC_Time.Year, 4);  // 年
		// OLED_ShowNum(1, 11, MyRTC_Time.Month, 2); // 月
		// OLED_ShowNum(1, 14, MyRTC_Time.Day, 2); // 日
		// OLED_ShowNum(2, 6, MyRTC_Time.Hour, 2); // 时
		// OLED_ShowNum(2, 9, MyRTC_Time.Minute, 2); // 分
		// OLED_ShowNum(2, 12, MyRTC_Time.Second, 2); // 秒

		// OLED_ShowNum(3, 6, RTC_GetCounter(), 10);
		// OLED_ShowNum(4, 6, RTC_GetDivider(), 10);
		// // OLED_ShowNum(4, 6, (32767 - RTC_GetDivider()) / 32767.0 * 999, 10); // 重映射到 0 ~ 999

		// printf("RTC: %d-%d-%d %d:%d:%d\r\n", MyRTC_Time.Year, MyRTC_Time.Month, MyRTC_Time.Day,
		// 	MyRTC_Time.Hour, MyRTC_Time.Minute, MyRTC_Time.Second);

		// keyNum = Key_GetNum();
		// if (keyNum == 1)
		// {
		// // 	LED1_Turn();

		
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

		// OLED_Printf(0, 48, OLED_6X8, "Timer: %d", Timer_GetCount());
		// OLED_Update();

		// keyNum = Key_GetNum();
		// if (keyNum == 1) {
		// 	keyClickTotal++;
		// } else if (keyNum == 2) {
		// 	keyClickTotal += 10;
		// }
		// OLED_Printf(0, 56, OLED_6X8, "Key: %d", keyClickTotal);
		// OLED_Update();

		// if (mpu_ready > 0) {
			// if (mpu_ready) {
			// 	mpu_ready = 0;
			// 	MPU6050_GetData(&mpu);
			// 	mpuTempture = MPU6050_GetTempture();
			// }

			OLED_Printf(0, 0, OLED_6X8, "A: %+d %+d %+d", mpu.raw.ax, mpu.raw.ay, mpu.raw.az);
			OLED_Printf(0, 8, OLED_6X8, "G: %+d %+d %+d", mpu.raw.gx, mpu.raw.gy, mpu.raw.gz);
			// OLED_Printf(0, 32, OLED_6X8, "Temp: %+d . %+.2f", mpu.raw.temp, mpuTempture);
			OLED_Printf(0, 24, OLED_6X8, "  yaw: %+.3f", mpu.euler.yaw);
			OLED_Printf(0, 32, OLED_6X8, " roll: %+.3f", mpu.euler.roll);
			OLED_Printf(0, 40, OLED_6X8, "pitch: %+.3f", mpu.euler.pitch);
		// }

		// printf("%+.3f, %+.3f, %+.3f\r\n", mpu.euler.yaw, mpu.euler.roll, mpu.euler.pitch);
		// printf("Timer: %d\r\n", timer_count);
		OLED_Printf(0, 56, OLED_6X8, "Timer TTL: %dus", timer_count);
		OLED_Update();



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
