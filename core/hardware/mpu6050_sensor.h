#ifndef __MPU6050_SENSOR_H__
#define __MPU6050_SENSOR_H__

#include "stm32f10x.h"                  // Device header

// #define MPU6050_I2C_HW 1	// 使用硬件I2C


// MPU6050 外部帧同步(FSYNC)引脚采样
typedef enum
{
	MPU6050_ExtSync_Disable = 0x00,	// 禁止外部同步
	MPU6050_ExtSync_Temp = 0x10,	// 温度传感器外部帧同步
	MPU6050_ExtSync_GyroX = 0x08,	// 陀螺仪X轴外部帧同步
	MPU6050_ExtSync_GyroY = 0x18,	// 陀螺仪Y轴外部帧同步
	MPU6050_ExtSync_GyroZ = 0x20,	// 陀螺仪Z轴外部帧同步
	MPU6050_ExtSync_AccX = 0x28,	// 加速度计X轴外部帧同步
	MPU6050_ExtSync_AccY = 0x30,	// 加速度计Y轴外部帧同步
	MPU6050_ExtSync_AccZ = 0x38	    // 加速度计Z轴外部帧同步
} MPU6050_ExtSync_TypeDef;
#define IS_MPU6050_EXTSYNC(EXTSYNC) (((EXTSYNC) == MPU6050_ExtSync_Disable) || \
	((EXTSYNC) == MPU6050_ExtSync_GyroX) || ((EXTSYNC) == MPU6050_ExtSync_GyroY) || ((EXTSYNC) == MPU6050_ExtSync_GyroZ) || \
	((EXTSYNC) == MPU6050_ExtSync_AccX) || ((EXTSYNC) == MPU6050_ExtSync_AccY) || ((EXTSYNC) == MPU6050_ExtSync_AccZ))

// MPU6050 数字低通滤波(DLPF)
typedef enum
{
	MPU6050_Filter_260Hz = 0x00,	// 加速度计(Band: 260Hz, Delay: 0ms, Fs: 1kHz) + 陀螺仪(Band: 256Hz, Delay: 0.98ms, Fs: 8kHz)
	MPU6050_Filter_184Hz = 0x01,  	// 加速度计(Band: 184Hz, Delay: 2.05ms, Fs: 1kHz) + 陀螺仪(Band: 188Hz, Delay: 1.9ms, Fs: 1kHz)
	MPU6050_Filter_94Hz = 0x02,   	// 加速度计(Band: 94Hz, Delay: 4.9ms, Fs: 1kHz) + 陀螺仪(Band: 92Hz, Delay: 3.9ms, Fs: 1kHz)
	MPU6050_Filter_44Hz = 0x03,   	// 加速度计(Band: 44Hz, Delay: 8.5ms, Fs: 1kHz) + 陀螺仪(Band: 42Hz, Delay: 7.9ms, Fs: 1kHz)
	MPU6050_Filter_21Hz = 0x04,   	// 加速度计(Band: 21Hz, Delay: 17ms, Fs: 1kHz) + 陀螺仪(Band: 20Hz, Delay: 16.6ms, Fs: 1kHz)
	MPU6050_Filter_10Hz = 0x05,   	// 加速度计(Band: 10Hz, Delay: 34ms, Fs: 1kHz) + 陀螺仪(Band: 10Hz, Delay: 32.8ms, Fs: 1kHz)
	MPU6050_Filter_5Hz = 0x06     	// 加速度计(Band: 5Hz, Delay: 68ms, Fs: 1kHz) + 陀螺仪(Band: 5Hz, Delay: 66.6ms, Fs: 1kHz)
} MPU6050_Filter_TypeDef;
#define IS_MPU6050_FILTER(FILTER) (((FILTER) == MPU6050_Filter_260Hz) || ((FILTER) == MPU6050_Filter_184Hz) || \
	((FILTER) == MPU6050_Filter_94Hz) || ((FILTER) == MPU6050_Filter_44Hz) || ((FILTER) == MPU6050_Filter_21Hz) || \
	((FILTER) == MPU6050_Filter_10Hz) || ((FILTER) == MPU6050_Filter_5Hz))


// MPU6050 陀螺仪 - 满刻度范围
typedef enum
{
	MPU6050_GYRO_FullScale_250dps = 0x00,		// 陀螺满仪刻度范围 ±250°/s
	MPU6050_GYRO_FullScale_500dps = 0x08,		// 陀螺满仪刻度范围 ±500°/s
	MPU6050_GYRO_FullScale_1000dps = 0x10,		// 陀螺满仪刻度范围 ±1000°/s
	MPU6050_GYRO_FullScale_2000dps = 0x18,		// 陀螺满仪刻度范围 ±2000°/s
} MPU6050_GYRO_FullScale_TypeDef;
#define IS_MPU6050_GYRO_FULLSCALE(FULLSCALE) (((FULLSCALE) == MPU6050_GYRO_FullScale_250dps) || \
	((FULLSCALE) == MPU6050_GYRO_FullScale_500dps) || ((FULLSCALE) == MPU6050_GYRO_FullScale_1000dps) || \
	((FULLSCALE) == MPU6050_GYRO_FullScale_2000dps))

// MPU6050 陀螺仪 - 自检 (可并行)
#define MPU6050_GYRO_SELFTEST_X 		0x80	// 启用X轴陀螺仪自检
#define MPU6050_GYRO_SELFTEST_Y 		0x40	// 启用Y轴陀螺仪自检
#define MPU6050_GYRO_SELFTEST_Z 		0x20	// 启用Z轴陀螺仪自检
#define IS_MPU6050_GYRO_SELFTEST(GYRO_SELFTEST) (((GYRO_SELFTEST) & ~(MPU6050_GYRO_SELFTEST_X | \
    MPU6050_GYRO_SELFTEST_Y | MPU6050_GYRO_SELFTEST_Z)) == 0)


// MPU6050 加速度计 - 满刻度范围
typedef enum
{
	MPU6050_ACCEL_FullScale_2g = 0x00,			// 加速度计满刻度范围 ±2g
	MPU6050_ACCEL_FullScale_4g = 0x08,			// 加速度计满刻度范围 ±4g
	MPU6050_ACCEL_FullScale_8g = 0x10,			// 加速度计满刻度范围 ±8g
	MPU6050_ACCEL_FullScale_16g = 0x18,			// 加速度计满刻度范围 ±16g
} MPU6050_ACCEL_FULLSCALE_TypeDef;
#define IS_MPU6050_ACCEL_FULLSCALE(FULLSCALE) ((((FULLSCALE) == MPU6050_ACCEL_FullScale_2g) || \
	((FULLSCALE) == MPU6050_ACCEL_FullScale_4g) || ((FULLSCALE) == MPU6050_ACCEL_FullScale_8g) || \
	((FULLSCALE) == MPU6050_ACCEL_FullScale_16g))

// MPU6050 加速度计 - 自检 (可并行)
#define MPU6050_ACCEL_SELFTEST_Disable	0x00	// 禁用加速度计自检
#define MPU6050_ACCEL_SELFTEST_X		0x80	// 启用X轴加速度计自检
#define MPU6050_ACCEL_SELFTEST_Y		0x40	// 启用Y轴加速度计自检
#define MPU6050_ACCEL_SELFTEST_Z		0x20	// 启用Z轴加速度计自检
#define IS_MPU6050_ACCEL_SELFTEST(ACCEL_SELFTEST) (((ACCEL_SELFTEST) & ~(MPU6050_ACCEL_SELFTEST_X | \
    MPU6050_ACCEL_SELFTEST_Y | MPU6050_ACCEL_SELFTEST_Z | MPU6050_ACCEL_SELFTEST_Disable)) == 0)



// MPU6050 FIFO使能 (可并行)
#define MPU6050_FIFO_SLV0		0x01	// 从设备0 EXT_SENS_DATA 数据写入FIFO缓冲器
#define MPU6050_FIFO_SLV1		0x02	// 从设备1 EXT_SENS_DATA 数据写入FIFO缓冲器
#define MPU6050_FIFO_SLV2		0x04	// 从设备2 EXT_SENS_DATA 数据写入FIFO缓冲器
#define MPU6050_FIFO_ACCEL		0x08	// 加速度计XYZ轴 (H + L) 数据写入FIFO缓冲器
#define MPU6050_FIFO_ZG			0x10	// 陀螺仪Z轴 (H + L) 数据写入FIFO缓冲器
#define MPU6050_FIFO_YG			0x20	// 陀螺仪Y轴 (H + L) 数据写入FIFO缓冲器
#define MPU6050_FIFO_XG			0x40	// 陀螺仪X轴 (H + L) 数据写入FIFO缓冲器
#define MPU6050_FIFO_TEMP		0x80	// 温度数据 (H + L) 数据写入FIFO缓冲器
#define IS_MPU6050_FIFO(FIFO) (((FIFO) & ~(MPU6050_FIFO_SLV0 | \
    MPU6050_FIFO_SLV1 | MPU6050_FIFO_SLV2 | MPU6050_FIFO_ACCEL | \
    MPU6050_FIFO_ZG | MPU6050_FIFO_YG | MPU6050_FIFO_XG | MPU6050_FIFO_TEMP)) == 0)

// MPU6050 电源管理1 - 时钟源
// - 选择内部 8MHz 振荡器或外部时钟源时，可以在禁用陀螺仪的情况下以低功耗模式运行
// - 强烈建议将设备配置为使用其中一个陀螺仪（或外部时钟源）作为时钟基准，以提高稳定性
typedef enum
{
	MPU6050_PWR_MGMT1_Clock_IN_8MHz = 0x00,		// 内部 8MHz 振荡器 时钟源
	MPU6050_PWR_MGMT1_Clock_GX = 0x01,			// 陀螺仪X轴 时钟源
	MPU6050_PWR_MGMT1_Clock_GY = 0x02,			// 陀螺仪Y轴 时钟源
	MPU6050_PWR_MGMT1_Clock_GZ = 0x03,			// 陀螺仪Z轴 时钟源
	MPU6050_PWR_MGMT1_Clock_EXT_32kHz = 0x04,	// 外部 32.768kHz 振荡器 时钟源
	MPU6050_PWR_MGMT1_Clock_EXT_19MHz = 0x05,	// 外部 19.2MHz 振荡器 时钟源
	MPU6050_PWR_MGMT1_Clock_STOPPED = 0x07,		// 停止时钟, 使定时发生器处于复位状态
} MPU6050_PWR_MGMT1_Clock_TypeDef;
#define IS_MPU6050_PWR_MGMT1_CLOCK(CLOCK) (((CLOCK) == MPU6050_PWR_MGMT1_Clock_IN_8MHz) || \
	((CLOCK) == MPU6050_PWR_MGMT1_Clock_GX) || ((CLOCK) == MPU6050_PWR_MGMT1_Clock_GY) || \
	((CLOCK) == MPU6050_PWR_MGMT1_Clock_GZ) || ((CLOCK) == MPU6050_PWR_MGMT1_Clock_EXT_32kHz) || \
	((CLOCK) == MPU6050_PWR_MGMT1_Clock_EXT_19MHz))

// MPU6050 电源管理1 - 模式 (可并行)
// - 在禁用 SLEEP 的同时将 CYCLE 设置为1 时，将进入 睡眠/唤醒 的循环模式, 采集频率为 LP_Wake 的设定值
// - 启用复位会把所有内部寄存器重置为默认值。复位完成后，该位自动清零
#define MPU6050_PWR_MGMT1_MODE_TEMP_DISABLE		0x80,		// 禁用温度传感器
#define MPU6050_PWR_MGMT1_MODE_CYCLE_ENABLE		0x20,		// 启用循环模式 (禁用休眠才有效)
#define MPU6050_PWR_MGMT1_MODE_SLEEP_ENABLE		0x40,		// 启用休眠模式
#define MPU6050_PWR_MGMT1_MODE_DEVICE_RESET		0x80,		// 启用设备复位
#define IS_MPU6050_PWR_MGMT1_MODE(MODE) (((MODE) & ~(MPU6050_PWR_MGMT1_TEMP_DISABLE | \
    MPU6050_PWR_MGMT1_CYCLE_ENABLE | MPU6050_PWR_MGMT1_SLEEP_ENABLE | MPU6050_PWR_MGMT1_DEVICE_RESET)) == 0)

// MPU6050 电源管理2 - 模式 (可并行)
#define MPU6050_PWR_MGMT2_MODE_STBY_ZG			0x01		// 禁用Z轴陀螺仪
#define MPU6050_PWR_MGMT2_MODE_STBY_YG			0x02		// 禁用Y轴陀螺仪
#define MPU6050_PWR_MGMT2_MODE_STBY_XG			0x04		// 禁用X轴陀螺仪
#define MPU6050_PWR_MGMT2_MODE_STBY_ZA			0x08		// 禁用Z轴加速度计
#define MPU6050_PWR_MGMT2_MODE_STBY_YA			0x10		// 禁用Y轴加速度计
#define MPU6050_PWR_MGMT2_MODE_STBY_XA			0x20		// 禁用Z轴加速度计
#define IS_MPU6050_PWR_MGMT2_MODE(MODE) (((MODE) & ~(MPU6050_PWR_MGMT2_STBY_ZG | \
    MPU6050_PWR_MGMT2_STBY_YG | MPU6050_PWR_MGMT2_STBY_XG | MPU6050_PWR_MGMT2_STBY_ZA | \
    MPU6050_PWR_MGMT2_STBY_YA | MPU6050_PWR_MGMT2_STBY_XA))) == 0)

// MPU6050 电源管理2 - 唤醒频率 
// - 仅 加速计低功耗模式 下的唤醒频率
typedef enum
{
	MPU6050_PWR_MGMT2_LP_WAKE_1_25Hz = 0x00,	// 唤醒频率 1.25Hz (仅加速计低功耗模式下)
	MPU6050_PWR_MGMT2_LP_WAKE_5Hz = 0x40,		// 唤醒频率 5Hz (仅加速计低功耗模式下)
	MPU6050_PWR_MGMT2_LP_WAKE_20Hz = 0x80,		// 唤醒频率 20Hz (仅加速计低功耗模式下)
	MPU6050_PWR_MGMT2_LP_WAKE_40Hz = 0xC0,		// 唤醒频率 40Hz (仅加速计低功耗模式下)
} MPU6050_PWR_MGMT2_LP_Wake_TypeDef;
#define IS_MPU6050_PWR_MGMT2_LP_WAKE(LP_WAKE) (((LP_WAKE) == MPU6050_PWR_MGMT2_LP_WAKE_1_25Hz) || \
	((LP_WAKE) == MPU6050_PWR_MGMT2_LP_WAKE_5Hz) || ((LP_WAKE) == MPU6050_PWR_MGMT2_LP_WAKE_20Hz) || \
	(LP_WAKE) == MPU6050_PWR_MGMT2_LP_WAKE_40Hz)

//  MPU6050 初始化配置
typedef struct
{
	uint8_t SMPRT_DIV; 									// 采样率分频 (范围: 0 ~ 254)
	MPU6050_ExtSync_TypeDef ExtSync;					// 外部帧同步
	MPU6050_Filter_TypeDef Filter;						// 数字低通滤波

	MPU6050_GYRO_FullScale_TypeDef GYRO_FullScale;		// 陀螺仪满量程范围
	uint8_t GYRO_SelfTest;								// 陀螺仪自检

	MPU6050_ACCEL_FULLSCALE_TypeDef ACCEL_FullScale;	// 加速度计
	uint8_t ACCEL_SelfTest;

	uint8_t FIFO;										// FIFO

	uint8_t PWR1_Mode;
	MPU6050_PWR_MGMT1_Clock_TypeDef PWR1_Clock;			// 电源管理1 时钟源

	uint8_t PWR2_Mode;									// 电源管理2 模式
	MPU6050_PWR_MGMT2_LP_Wake_TypeDef PWR2_LP_Wake;		// 唤醒频率 (仅加速度计低功耗模式下)
} MPU6050_InitTypeDef;


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
