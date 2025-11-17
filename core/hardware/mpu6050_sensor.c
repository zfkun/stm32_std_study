// MPU6050 传感器模块 (GY-521)
// 三轴加速计, 三轴电子陀螺仪, 6DOF角度姿态传感器

#include "mpu6050_sensor.h"
#include "mpu6050_sensor_reg.h"

#ifdef MPU6050_I2C_HW
#include "bsp_i2c_hw.h"
#else
#include "bsp_i2c.h"
#endif

// MPU6050 模块基本参数
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

/**
  * 函    数：MPU6050批量写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数组数据
  * 参    数：length 要写入寄存器的数据长度
  * 返 回 值：无
  */
static void _writeArrayReg(uint8_t RegAddress, uint8_t *Data, uint16_t length)
{
#ifdef MPU6050_I2C_HW
#else
	MyI2C_Start();						//I2C起始
	
    MyI2C_SendByte(MPU6050_SLAVE_ADDR); //发送从机地址，读写位为0，表示即将写入
	MyI2C_WaitAck();					//接收应答
	
    MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_WaitAck();					//接收应答

    for(uint16_t i = 0; i < length; i++)
    {
        MyI2C_SendByte(Data[i]);	    //发送要写入寄存器的数据
		MyI2C_WaitAck();				//接收应答
    }
	
    MyI2C_Stop();						//I2C终止
#endif
}

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
	MyI2C_NAck();					        //发送应答，给从机非应答，终止从机的数据输出

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

	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	_writeReg(MPU6050_PWR_MGMT_1, 0x01);    //电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	_writeReg(MPU6050_PWR_MGMT_2, 0x00);    //电源管理寄存器2，保持默认值0，所有轴均不待机
	_writeReg(MPU6050_SMPLRT_DIV, 0x09);    //采样率分频寄存器，配置采样率 (10分频)
	_writeReg(MPU6050_CONFIG, 0x06);        //配置寄存器，配置DLPF (不启用外部同步00000, 滤波最平滑110)
	_writeReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s (000, 量程满11, 000)
	_writeReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g (自测000, 量程满11, 滤波无000)
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

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(MPU6050_Data_t *Data)
{
    // // 1. 逐个读取方式
	// uint8_t DataH, DataL;                        //定义数据高8位和低8位的变量
	
	// DataH = _readReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	// DataL = _readReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	// Data->AccX = (DataH << 8) | DataL;           //数据拼接，通过输出参数返回
	
	// DataH = _readReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	// DataL = _readReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	// Data->AccY = (DataH << 8) | DataL;           //数据拼接，通过输出参数返回
	
	// DataH = _readReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	// DataL = _readReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	// Data->AccZ = (DataH << 8) | DataL;           //数据拼接，通过输出参数返回

    // DataH = _readReg(MPU6050_TEMP_OUT_H);        //读取温度传感器的高8位数据
    // DataL = _readReg(MPU6050_TEMP_OUT_L);        //读取温度传感器的低8位数据
    // Data->Temp = (DataH << 8) | DataL;           //数据拼接，通过输出参数返回
	
	// DataH = _readReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	// DataL = _readReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	// Data->GyroX = (DataH << 8) | DataL;          //数据拼接，通过输出参数返回
	
	// DataH = _readReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	// DataL = _readReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	// Data->GyroY = (DataH << 8) | DataL;          //数据拼接，通过输出参数返回
	
	// DataH = _readReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	// DataL = _readReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	// Data->GyroZ = (DataH << 8) | DataL;          //数据拼接，通过输出参数返回

    // 2. 批量读取方式 (更效率)
    uint8_t datas[14];
    _readArrayReg(MPU6050_ACCEL_XOUT_H, datas, 14);

    Data->AccX = (datas[0] << 8) | datas[1];
    Data->AccY = (datas[2] << 8) | datas[3];
    Data->AccZ = (datas[4] << 8) | datas[5];
    Data->Temp = (datas[6] << 8) | datas[7];
    Data->GyroX = (datas[8] << 8) | datas[9];
    Data->GyroY = (datas[10] << 8) | datas[11];
    Data->GyroZ = (datas[12] << 8) | datas[13];
}
