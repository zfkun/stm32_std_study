// MPU6050 传感器模块 (GY-521)
// 三轴加速计, 三轴电子陀螺仪, 6DOF角度姿态传感器
//
// 模块基本参数
// - 内置16位ADC采集传感器: 模拟信号量化范围 -32768 ~ 32767
// - 加速度计满量程选择: ±2, ±4, ±8, ±16 (g)
// - 陀螺仪满量程选择: ±250, ±500, ±1000, ±2000 (°/s)
// - 可配置的数字低通滤波器:
// - 可配置的时钟源
// - 可配置的采样分频
// - I2C从机地址:
//    - 0110 1000 (AD0=0)
//    - 0110 1001 (AD0=1)
//   当 AD0 = 0 时, 即 0x68
//   也有把 0xD0 当作从机地址, 是因为下面描述的规则情况下, 直接把 0xD0 当作了从机地址而的得来的:
//      I2C 通讯协议里规定的:
//      寻址开始信号发送的 一个字节数据 (8bit) 里, 前7位(7bit ~ 1bit) 是从机地址, 最低位 (0bit) 是 读写位 (1: 读, 0: 写)
//      那么, 当主机发送开始信号查询 MPU6050 从机设备的时候, 开始信号的数据字节就应该是
//         (0x68 << 1 (左移1位))   |  0x0 (写)      // (最低位 与 读写位, 按 位或 运算)
//         =            1101 0000 | 0000 0000
//         =               0xD0   |   0x0
//         =                     0xD0
//   无论从机地址按哪个值的说法, 都是一样的, 只是 0x68 是按没有位移之前的说的, 0xD0 是按位移之后说的
//   同理, 如果 AD0 = 1 时, 从机地址可以说是  0x69 也可以说是 0xD1
//   
// - SCL 和 SDA 引脚: 内置 4.7KΩ 的上拉电阻
// - XCL / XDA 引脚: 可用于外接 磁力计, 气压计 用于扩展集成
// - AD0: 从机地址最低位
// - INT: 中断信号输出
//
// 需要关注的寄存器:     
// - SMPLRT_DIV: 采样分频, 地址 0x19 (十六进制), 25 (十进制)
//   分频越小, 采样速率越大, 精度越高
//   采样率 = 陀螺仪输出时钟频率 / (1 + SMPLRT_DIV)
//   注意: 陀螺仪输出时钟频率 = 8kHz (当 低通滤波器 DLPF 禁用时, 即 DLPF_CFG = 0 或 7)
//         陀螺仪输出时钟频率 = 1kHz (当 低通滤波器 DLPF 启用时)
// - CONFIG: 配置寄存器, 地址 0x1A (十六进制), 26 (十进制)
// - GYRO_CONFIG: 陀螺仪配置寄存器, 地址 0x1B (十六进制), 27 (十进制)
//   - AFS_SEL: 满量程选择
// - ACCEL_CONFIG: 加速度计配置寄存器, 地址 0x1C (十六进制), 28 (十进制)
//   - AFS_SEL: 满量程选择
// - ACCEL_XOUT_H: 加速度计X轴输出寄存器, 地址 0x3B (十六进制), 59 (十进制)
// - ACCEL_XOUT_L: 加速度计X轴输出寄存器, 地址 0x3C (十六进制), 60 (十进制)
// - ACCEL_YOUT_H: 加速度计Y轴输出寄存器, 地址 0x3D (十六进制), 61 (十进制)
// - ACCEL_YOUT_L: 加速度计Y轴输出寄存器, 地址 0x3E (十六进制), 62 (十进制)
// - ACCEL_ZOUT_H: 加速度计Z轴输出寄存器, 地址 0x3F (十六进制), 63 (十进制)
// - ACCEL_ZOUT_L: 加速度计Z轴输出寄存器, 地址 0x40 (十六进制), 64 (十进制)
// - TEMP_OUT_H: 温度输出寄存器, 地址 0x41 (十六进制), 65 (十进制)
// - TEMP_OUT_L: 温度输出寄存器, 地址 0x42 (十六进制), 66 (十进制)
// - GYRO_XOUT_H: 陀螺仪X轴输出寄存器, 地址 0x43 (十六进制), 67 (十进制)
// - GYRO_XOUT_L: 陀螺仪X轴输出寄存器, 地址 0x44 (十六进制), 68 (十进制)
// - GYRO_YOUT_H: 陀螺仪Y轴输出寄存器, 地址 0x45 (十六进制), 69 (十进制)
// - GYRO_YOUT_L: 陀螺仪Y轴输出寄存器, 地址 0x46 (十六进制), 70 (十进制)
// - GYRO_ZOUT_H: 陀螺仪Z轴输出寄存器, 地址 0x47 (十六进制), 71 (十进制)
// - GYRO_ZOUT_L: 陀螺仪Z轴输出寄存器, 地址 0x48 (十六进制), 72 (十进制)
// - PWR_MGMT_1: 电源管理寄存器, 地址 0x6B (十六进制), 107 (十进制)
//   - DEVICE_RESET: 复位设备
//   - SLEEP: 睡眠模式
//   - CYCLE: 循环模式 (间隔一定周期 LP_WAKE_CTRL 采集一次数据)
//   - TEMP_DIS: 禁用温度传感器
//   - CLKSEL: 晶振选择, 建议选择 陀螺仪晶振
// - PWR_MGMT_2: 电源管理寄存器, 地址 0x6C (十六进制), 108 (十进制)
//   - STBY_XA: 禁用X轴加速度计
//   - STBY_YA: 禁用Y轴加速度计
//   - STBY_ZA: 禁用Z轴加速度计
//   - STBY_XG: 禁用X轴陀螺仪
//   - STBY_YG: 禁用Y轴陀螺仪
//   - STBY_ZG: 禁用Z轴陀螺仪
//   - STBY_XYZA: 禁用所有轴加速度计
//   - STBY_XYZG: 禁用所有轴陀螺仪
//   - LP_WAKE_CTRL: 循环模式下, 采集一次数据间隔时间
// - WHO_AM_I: 读写设备ID寄存器, 地址 0x75 (十六进制), 117 (十进制)
//     --------------------------------------------------------------------------
//     bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0
//       -  |   -  |   -  |   -  |   -  |   -  |   -  |  -
//       0      1      1      0      1      0      0     0
//   即设备ID, 固定为: 0x68
//
// 特别注意:
// 上电后, 2个寄存器的默认值:
//   - PWR_MGMT_1: 0x40 = 0100 0000 (即 默认睡眠模式)
//     --------------------------------------------------------------------------
//           bit7   |  bit6  |  bit5   |  bit4 |   bit3     | bit2 | bit1 | bit0
//     DEVICE_RESET |  SLEEP |  CYCLE  |   -   |  TEMP_DIS  |    CLKSEL[2:0]
//            0         1        0         0         0         0      0     0
//   - WHO_AM_I: 0x68
//     AD0 的状态, 并不会影响 bit0
//

#include <math.h>
#include "bsp_math.h"
#include "mpu6050.h"
#include "mpu6050_reg.h"

#ifdef MPU6050_I2C_HW
#include "bsp_i2c_hw.h"
#else
#include "bsp_i2c.h"
#endif

/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
static void _writeReg(uint8_t RegAddress, uint8_t Data)
{
#ifdef MPU6050_I2C_HW
    HwI2C_Start();                                                                                                  //硬件I2C生成起始条件 并 等待EV5

    HwI2C_SendAddress(MPU6050_SLAVE_ADDR, I2C_Direction_Transmitter, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);   //硬件I2C发送从机地址，方向为发送 并 等待EV6
    HwI2C_SendData(RegAddress, I2C_EVENT_MASTER_BYTE_TRANSMITTING);                                                 //硬件I2C发送寄存器地址 并 等待EV8
    HwI2C_SendData(Data, I2C_EVENT_MASTER_BYTE_TRANSMITTED);                                                        //硬件I2C发送数据 并 等待EV8_2

    HwI2C_Stop();                                                                                                   //硬件I2C生成终止条件
#else
	MyI2C_Start();						//I2C起始
	
    MyI2C_SendByte(MPU6050_SLAVE_ADDR); //发送从机地址，读写位为0，表示即将写入
	MyI2C_WaitAck();					//接收应答
	
    MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_WaitAck();					//接收应答

	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_WaitAck();					//接收应答
	
    MyI2C_Stop();						//I2C终止
#endif
}

// /**
//   * 函    数：MPU6050批量写寄存器
//   * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
//   * 参    数：Data 要写入寄存器的数组数据
//   * 参    数：length 要写入寄存器的数据长度
//   * 返 回 值：无
//   */
// static void _writeArrayReg(uint8_t RegAddress, uint8_t *Data, uint16_t length)
// {
// #ifdef MPU6050_I2C_HW
// #else
// 	MyI2C_Start();						//I2C起始
	
//     MyI2C_SendByte(MPU6050_SLAVE_ADDR); //发送从机地址，读写位为0，表示即将写入
// 	MyI2C_WaitAck();					//接收应答
	
//     MyI2C_SendByte(RegAddress);			//发送寄存器地址
// 	MyI2C_WaitAck();					//接收应答

//     for(uint16_t i = 0; i < length; i++)
//     {
//         MyI2C_SendByte(Data[i]);	    //发送要写入寄存器的数据
// 		MyI2C_WaitAck();				//接收应答
//     }
	
//     MyI2C_Stop();						//I2C终止
// #endif
// }

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
static uint8_t _readReg(uint8_t RegAddress)
{
	uint8_t Data;

#ifdef MPU6050_I2C_HW
    HwI2C_Start();                                                                                                  //硬件I2C生成起始条件 并 等待EV5
    HwI2C_SendAddress(MPU6050_SLAVE_ADDR, I2C_Direction_Transmitter, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);   //硬件I2C发送从机地址，方向为发送 并 等待EV6
    HwI2C_SendData(RegAddress, I2C_EVENT_MASTER_BYTE_TRANSMITTED);                                                  //硬件I2C发送寄存器地址 并 等待EV8_2

    HwI2C_Start();                                                                                                  //硬件I2C生成重复起始条件 并 等待EV5
    HwI2C_SendAddress(MPU6050_SLAVE_ADDR, I2C_Direction_Receiver, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);         //硬件I2C发送从机地址，方向为接收 并 等待EV6

    HwI2C_Ack(DISABLE);									                                                            //在接收最后一个字节之前提前将应答失能
	HwI2C_Stop();										                                                            //在接收最后一个字节之前提前申请停止条件
    Data = HwI2C_ReceiveData();							                                                            //接收数据寄存器
    HwI2C_Ack(ENABLE);									                                                            //将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
#else
	MyI2C_Start();						        //I2C起始
    MyI2C_SendByte(MPU6050_SLAVE_ADDR);         //发送从机地址，读写位为0，表示即将写入
	MyI2C_WaitAck();					        //接收应答
    MyI2C_SendByte(RegAddress);			        //发送寄存器地址
	MyI2C_WaitAck();					        //接收应答
	
	MyI2C_Start();						        //I2C重复起始
    MyI2C_SendByte(MPU6050_SLAVE_ADDR | 0x01);  //发送从机地址，读写位为1，表示即将读取
	MyI2C_WaitAck();					        //接收应答

    Data = MyI2C_ReceiveByte();			        //接收指定寄存器的数据
	MyI2C_NAck();					            //发送应答，给从机非应答，终止从机的数据输出

	MyI2C_Stop();						        //I2C终止
#endif

	return Data;
}

static void _readArrayReg(uint8_t StartRegAddress, uint8_t *Data, uint16_t length)
{
#ifdef MPU6050_I2C_HW
    HwI2C_Start();                                                                                                  //硬件I2C生成起始条件 并 等待EV5
    HwI2C_SendAddress(MPU6050_SLAVE_ADDR, I2C_Direction_Transmitter, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);   //硬件I2C发送从机地址，方向为发送 并 等待EV6
    HwI2C_SendData(StartRegAddress, I2C_EVENT_MASTER_BYTE_TRANSMITTED);                                             //硬件I2C发送寄存器地址 并 等待EV8_2

    HwI2C_Start();                                                                                                  //硬件I2C生成重复起始条件 并 等待EV5
    HwI2C_SendAddress(MPU6050_SLAVE_ADDR, I2C_Direction_Receiver, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);         //硬件I2C发送从机地址，方向为接收 并 等待EV6
    
    for(uint16_t i = 0; i < length; i++)
    {
        if (i == length - 1) {
            HwI2C_Ack(DISABLE);									                                                    //在接收最后一个字节之前提前将应答失能
	        HwI2C_Stop();										                                                    //在接收最后一个字节之前提前申请停止条件
        }

        Data[i] = HwI2C_ReceiveData();		                                                                        //接收一个字节数据
    }

    HwI2C_Ack(ENABLE);									                                                            //将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
#else
	MyI2C_Start();						        //I2C起始

    MyI2C_SendByte(MPU6050_SLAVE_ADDR);         //发送从机地址，读写位为0，表示即将写入
	MyI2C_WaitAck();					        //接收应答

    MyI2C_SendByte(StartRegAddress);	        //发送要读取的**起始**寄存器的地址
	MyI2C_WaitAck();						    //接收应答

    MyI2C_Start();						        //I2C重复起始
    
    MyI2C_SendByte(MPU6050_SLAVE_ADDR | 0x01);  //发送从机地址，读写位为1，表示即将读取
    MyI2C_WaitAck();					        //接收应答

    for (uint16_t i = 0; i < length; i++)
    {
        Data[i] = MyI2C_ReceiveByte();		    //接收指定寄存器的数据

        if (i == length - 1) {
            MyI2C_NAck();                       //发送应答，给从机非应答，终止从机的数据输出
        }
        else {
            MyI2C_Ack();                        //发送应答，给从机应答，继续等待从机的数据输出
        }
    }
	
    MyI2C_Stop();						         //I2C终止
#endif
}

/**
 * 配置 MPU6050
 */
static void _initConfig(MPU6050_InitTypeDef *InitStruct) {
    assert_param(IS_MPU6050_PWR_MGMT1_CLOCK(InitStruct->PWR1_Clock));
    assert_param(IS_MPU6050_PWR_MGMT1_MODE(InitStruct->PWR1_Mode));
    assert_param(IS_MPU6050_PWR_MGMT2_LP_WAKE(InitStruct.PWR2_LP_Wake));
    assert_param(IS_MPU6050_PWR_MGMT2_MODE(InitStruct->PWR2_Mode));
    assert_param(InitStruct->SMPRT_DIV > 0 && InitStruct->SMPRT_DIV < 256);
    assert_param(IS_MPU6050_EXTSYNC(InitStruct->ExtSync));
    assert_param(IS_MPU6050_FILTER(InitStruct->Filter));
    assert_param(IS_MPU6050_GYRO_FULLSCALE(InitStruct->GYRO_FullScale));
    assert_param(IS_MPU6050_ACCEL_FULLSCALE(InitStruct->ACCEL_FullScale));
    assert_param(IS_MPU6050_GYRO_SELFTEST(InitStruct->GYRO_SelfTest));
    assert_param(IS_MPU6050_ACCEL_SELFTEST(InitStruct->ACCEL_SelfTest));
    assert_param(IS_MPU6050_FIFO(InitStruct->FIFO));


    float _scale;
    
    // 加速度计量程比例因子计算
    //   读取的原始值 * 比例因子 = 实际的加速度值
    //
    // - 由于 寄存器 ACCEL_XOUT_H, ACCEL_XOUT_L, ... ACCEL_ZOUT_L 是 16位的 (即取值范围 -32768 ~ 32767)
    //   所以计算 在满量程 (±16g) 下的  x, y, z 的加速度值, 只需要 {读取的寄存器原始值} / 32768 * 16 即可
    //   即
    //     - 满量程(±16g):
    //       X加速度 = raw.ax / 32768 * 16
    //               = raw.ax * (16 / 32768)
    //               = raw.ax * (1 / (32768 / 16))
    //               = raw.ax * (1 / 2048)
    //       Y加速度 = raw.ay * (1 / 2048)
    //       Z加速度 = raw.az * (1 / 2048)
    //     - 量程(±8g):
    //       X加速度 = raw.ax / 32768 * 8
    //               = raw.ax * (8 / 32768)
    //               = raw.ax * (1 / (32768 / 8))
    //               = raw.ax * (1 / 4096)
    //       Y加速度 = raw.ay * (1 / 4096)
    //       Z加速度 = raw.az * (1 / 4096)
    //     - 量程(±4g):
    //       X加速度 = raw.ax / 32768 * 4
    //               = raw.ax * (4 / 32768)
    //               = raw.ax * (1 / (32768 / 4))
    //               = raw.ax * (1 / 8192)
    //       Y加速度 = raw.ay * (1 / 8192)
    //       Z加速度 = raw.az * (1 / 8192)
    //     - 量程(±2g):
    //       X加速度 = raw.ax / 32768 * 2
    //               = raw.ax * (2 / 32768)
    //               = raw.ax * (1 / (32768 / 2))
    //               = raw.ax * (1 / 16384)
    //       Y加速度 = raw.ay * (1 / 16384)
    //       Z加速度 = raw.az * (1 / 16384)
    //
    // - 所以, 加速度计的不同量程设置对应的比例因子固定为:
    //   - 2g: 1 / 16384
    //   - 4g: 1 / 8192
    //   - 8g: 1 / 4096
    //   - 16g: 1 / 2048
    //
    if (InitStruct->ACCEL_FullScale == MPU6050_ACCEL_FullScale_2g) _scale = 16384;
    else if (InitStruct->ACCEL_FullScale == MPU6050_ACCEL_FullScale_4g) _scale = 8192;
    else if (InitStruct->ACCEL_FullScale == MPU6050_ACCEL_FullScale_8g) _scale = 4096;
    else if (InitStruct->ACCEL_FullScale == MPU6050_ACCEL_FullScale_16g) _scale = 2048;
    accelFactor = 1.0f / _scale;

    // 角速度量程比例因子计算
    //   读取的原始值 * 比例因子 = 实际的角速度值
    //
    // - 由于 寄存器 GYRO_XOUT_H, GYRO_XOUT_L, ... GYRO_ZOUT_L 是 16位的 (即取值范围 -32768 ~ 32767)
    //   所以计算 在满量程 (±2000°/s) 下的  x, y, z 的角速度值, 只需要 {读取的寄存器原始值} / 32768 * 2000 即可
    //   即
    //     - 满量程(±2000°/s):
    //       X角速度 = raw.gx / 32768 * 2000
    //              = raw.gx * (2000 / 32768)
    //              = raw.gx * (1 / (32768 / 2000))
    //              = raw.gx * (1 / 16.384)
    //       Y角速度 = raw.gy * (1 / 16.384)
    //       Z角速度 = raw.gz * (1 / 16.384)
    //
    // - 所以, 角速度计的不同量程设置对应的比例因子固定为:
    //   - 250dps: 1 / 131.072
    //   - 500dps: 1 / 65.536
    //   - 1000dps: 1 / 32.768
    //   - 2000dps: 1 / 16.384
    //
    if (InitStruct->GYRO_FullScale == MPU6050_GYRO_FullScale_250dps) _scale = 131.072f;
    else if (InitStruct->GYRO_FullScale == MPU6050_GYRO_FullScale_500dps) _scale = 65.536f;
    else if (InitStruct->GYRO_FullScale == MPU6050_GYRO_FullScale_1000dps) _scale = 32.768f;
    else if (InitStruct->GYRO_FullScale == MPU6050_GYRO_FullScale_2000dps) _scale = 16.384f;
    gyroFactor = _1DEG / _scale;

    _writeReg(MPU6050_PWR_MGMT_1, InitStruct->PWR1_Mode | InitStruct->PWR1_Clock);
	_writeReg(MPU6050_PWR_MGMT_2, InitStruct->PWR2_LP_Wake | InitStruct->PWR2_Mode);
	_writeReg(MPU6050_SMPLRT_DIV, InitStruct->SMPRT_DIV);
	_writeReg(MPU6050_CONFIG, InitStruct->ExtSync | InitStruct->Filter);
	_writeReg(MPU6050_GYRO_CONFIG, InitStruct->GYRO_SelfTest | InitStruct->GYRO_FullScale);
	_writeReg(MPU6050_ACCEL_CONFIG, InitStruct->ACCEL_SelfTest | InitStruct->ACCEL_FullScale);
    if (InitStruct->FIFO > 0) _writeReg(MPU6050_FIFO_EN, InitStruct->FIFO);
}

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
#ifdef MPU6050_I2C_HW
    HwI2C_Init();                                   //先初始化底层的I2C
#else
	MyI2C_Init();									//先初始化底层的I2C
#endif

    // 初始化配置
    MPU6050_InitTypeDef MPU6050_InitStruct;
    MPU6050_InitStruct.PWR1_Clock = MPU6050_PWR_MGMT1_Clock_GX;             // 时钟源为X轴陀螺仪
    MPU6050_InitStruct.PWR1_Mode = 0;                                       // 不休眠, 不循环, 不禁用温度传感器, 不复位
    MPU6050_InitStruct.PWR2_LP_Wake = MPU6050_PWR_MGMT2_LP_WAKE_1_25Hz;     // 唤醒频率 1.25Hz (仅加速度计低功耗模式下有效)
    MPU6050_InitStruct.PWR2_Mode = 0;                                       // 禁用低功耗模式

    // 这里设置 8 分频的原因:
    // 为了让 陀螺仪采样率达到 1KHz (即 1ms 刷新一次, 比较通用)
    // **但必要前提**: Filter 要设置为 `MPU6050_Filter_260Hz`, 即 陀螺仪频率是 8kHz (可以查看 `mpu6050.h` )
    //                这样根据 公式 就正好:  采样率 = 8kHz / (8 - 1) = 1kHz
    MPU6050_InitStruct.SMPRT_DIV = 8 - 1;                                   // 8分频采样
    MPU6050_InitStruct.ExtSync = MPU6050_ExtSync_Disable;                   // 禁用外部同步
    MPU6050_InitStruct.Filter = MPU6050_Filter_260Hz;                       // 不启用自带滤波 (延迟最小, 噪声最大, 否则怎么调 PID 也达不到理想效果)

    MPU6050_InitStruct.FIFO = 0;                                            // 关闭FIFO
    MPU6050_InitStruct.GYRO_SelfTest = 0;                                   // 禁用陀螺仪自测
    MPU6050_InitStruct.GYRO_FullScale = MPU6050_GYRO_FullScale_2000dps;     // 陀螺仪满量程
    MPU6050_InitStruct.ACCEL_SelfTest = 0;                                  // 禁用加速度计自测
    MPU6050_InitStruct.ACCEL_FullScale = MPU6050_ACCEL_FullScale_16g;       // 加速度计满量程

    _initConfig(&MPU6050_InitStruct);
}

/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return _readReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}


// 获取原始数据
void MPU6050_GetRaw(MPU6050_Data_t *data)
{
    uint8_t datas[14];
    _readArrayReg(MPU6050_ACCEL_XOUT_H, datas, 14);

    data->raw.ax = (((int16_t)datas[0]) << 8) | datas[1];
    data->raw.ay = (((int16_t)datas[2]) << 8) | datas[3];
    data->raw.az = (((int16_t)datas[4]) << 8) | datas[5];
    data->raw.temp = (((int16_t)datas[6]) << 8) | datas[7];
    data->raw.gx = (((int16_t)datas[8]) << 8) | datas[9];
    data->raw.gy = (((int16_t)datas[10]) << 8) | datas[11];
    data->raw.gz = (((int16_t)datas[12]) << 8) | datas[13];

    data->raw.gx += YAW_GX_OFFSET;  // 零飘补偿
    data->raw.gy += YAW_GY_OFFSET;  // 零飘补偿
}

// 获取欧拉角数据
// α(Alpha)
// β(Beta)
// θ(Theta)
// φ(Phi)
//
// - 俯仰角(pitch) 测量方案  
//   因为平衡车的物理结构 (面朝 X轴正方向, 俯仰变化对应 Y轴旋转, 偏航变化对应 Z轴旋转) 特性,
//   只需要考虑 XOZ平面的角度 (Y轴不参与计算) 即可
//
//   - 测量方案
//     - 仅 加速度计 (加速度):
//       - 公式
//         θacc = atan(ax / az) * (180 / PI)      (atan 返回范围是 -90 ~ 90, 会丢失精度, 90度边界会产生突变, 一般不使用)
//              = atan2(ax, az) * (180 / PI)      (atan2 函数返回范围是 -180 ~ 180, 一般使用这个)
//           - θacc: 加速度计的绝对角度
//           - ax: 加速度计的X轴分量
//           - az: 加速度计的Z轴分量
//         注意, 公式中的 ax, az 并没用使用 缩放比例因子 转换单位 到标准单位(°), 
//              是因为公式本身是计算的 ax 与 az 的比例大小关系(除法), 有没有缩放不影响结果, 所以没必要多一步缩放转换
//       - 优点:
//         - 静态稳定性 好: 重力分量测量准确, 角度误差小, 长时间不漂移
//         - 绝对和相对 好: 参考重力方向, 可以测量绝对角度
//       - 缺点   
//         - 动态稳定性 差: 受 运动加速度影响, 计算的角度会有偏移 (受到震动, 颠簸 等冲击力时, 偏移会更大更剧烈)
//     - 仅 陀螺仪 (角速度):
//       对角速度做 积分(累加和) 得到 角度 (变化量, 相对于启动时的角度(视作0点)的相对变化量)
//       - 公式
//         在 XOZ平面, 小车前后倾斜运动时, 产生绕Y轴旋转的角速度,
//         在离散系统中, 每隔 dt 时间采样一次角速度 gy(k) 的值,
//         初始俯仰角 为 θ(0), 则 当前时刻的俯仰角 θ(k) 满足:
//         θ(k) = θ(0) + ∑{gy(k) * dt}
//         θ(k) = θ(k - 1) + gy(k) * dt
//
//         最终
//         θgyro = θgyro + (gy * factor) * dt
//           - θgyro: 陀螺仪的角速度积分(累加和)
//           - gy: 陀螺仪的Y轴分量
//           - factor: 陀螺仪的Y轴分量的缩放因子 (即 角速度量程比例因子), 用于转换 gy 的值到标准单位(°)
//           - dt: 采样间隔时间(即 定时器 的时间, 单位秒, 一般是 1ms = 0.001s)
//         注意, 公式中的 gy 要先使用 缩放比例因子 转换单位到标准单位(°), 再进行后续计算
//
//       - 优点
//         - 动态稳定性 好: 不依赖重力分量, 运动加速度不影响测量, 受冲击力影响小
//       - 缺点
//         - 静态稳定性 差: 由于陀螺仪有零漂和误差, 长时间累加导致角度逐渐漂移增大
//         - 绝对和相对 差: 只能测量相对角度变化, 需要明确初始角度值, 才能测量绝对角度
//   
//     - 最终方案
//       互补滤波: 陀螺仪 (主) + 加速度计 (辅)
//         通过参考加速度计的角度, 修正陀螺仪角度的漂移, 同时为陀螺仪提供绝对参考
//         加速度计角度 和 陀螺仪角度 取加权平均值, 得到互补滤波后的陀螺仪角度:
//         θgyro_f = α * θacc + (1 - α) * θgyro
//           - α: 互补滤波权重, 取值范围 [0, 1], 值越大, 加速度计权重越大, 陀螺仪权重越小,  vice versa
//           - θacc: 加速度计绝对角度
//           - θgyro: 陀螺仪角速度积分(累加和)
//           - θgyro_f: 互补滤波后的陀螺仪角速度
//         注意, 滤波后, 下次的 θgyro 积分(累加), 要在 θgyro_f 的基础上计算(而不是用 θgyro)
//
//         公式也可以变换为: (更直观些)
//         θgyro_f = θgyro + α * (θacc - θgyro)
//
//           可以看出, 互补滤波会给 θgyro 添加一个偏移值, 方向朝向 θacc, 大小正比于 θacc 和 θgyro 的距离, 比例系数为 α
//         
void MPU6050_GetEuler(MPU6050_Data_t *data)
{
    float ax, ay, az;   // 实际加速度
    float gx, gy, gz;   // 实际角速度

    ax = data->raw.ax * accelFactor;
    ay = data->raw.ay * accelFactor;
    az = data->raw.az * accelFactor;

    gx = data->raw.gx * gyroFactor * data->gyro_dt;
    gy = data->raw.gy * gyroFactor * data->gyro_dt;
    gz = data->raw.gz * gyroFactor * data->gyro_dt;

    static float pitchGyro = 0.0f;      // 陀螺仪 Y轴角速度积分(累加和)
    static float rollGyro = 0.0f;       // 陀螺仪 X轴角速度积分(累加和)
    float alpha = FILTER_WEIGHT;        // 互补滤波权重 (0 ~ 1, 取值尽可能小)

#ifdef MPU6050_COMPLEMENTARY_FILTER_DYNAMIC_WEIGHT
    // 加速度的绝对值
    float absAcc = sqrt(ax * ax + ay * ay + az * az);

    // 动态调整权重
    if (absAcc > 1.2) {
        alpha = 0.2f;  // 快速运动或剧烈振动状态，减小加速度计权重
    } else {
        alpha = FILTER_WEIGHT;
    }
#endif

    // 加速度计 测量
    data->accel.pitch = -atan2(ax, az) * _1RAD;
    data->accel.pitch += EULER_AY_OFFSET;        // 零点校正补偿
    data->accel.roll = atan2(ay, az) * _1RAD;
    data->accel.roll += EULER_AX_OFFSET;

    // 陀螺仪 测量
    // pitchGyro += gy;
    // rollGyro += gx;
    // 互补滤波情况下, 角速度积分(累加和) 基于滤波后的值累加
    pitchGyro = data->euler.pitch + gy;
    rollGyro = data->euler.roll + gx;

    data->euler.pitch = alpha * data->accel.pitch + (1 - alpha) * pitchGyro; // 套公式
    data->euler.roll = alpha * data->accel.roll + (1 - alpha) * rollGyro;    // 套公式
    data->euler.yaw += gz * YAW_GZ_FACTOR + YAW_GZ_OFFSET;
}

void MPU6050_GetData(MPU6050_Data_t *data)
{
    MPU6050_GetRaw(data);
    MPU6050_GetEuler(data);
}

// 获取温度 (摄氏度)
int16_t MPU6050_GetTempture(void)
{ 
    uint8_t datas[2];
    _readArrayReg(MPU6050_TEMP_OUT_H, datas, 2);
    return (int16_t)((((int16_t)datas[0]) << 8) | datas[1]) * TEMP_MUL_FACTOR + 36.53;
}
