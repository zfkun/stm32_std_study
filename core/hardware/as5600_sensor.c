// AS5600 传感器模块
// 12位可编程,非接触式,磁感应角度测量传感器
//
// 注意:
//    - 由于AS5600是12位精度的霍尔传感器，一个8位寄存器无法存储全部数据，需要同时读两个数据才能读取完整
//    - AGC 取值: 要通过调整磁铁位置和距离(0.5 ~ 3mm), 控制AGC的值在取值范围的中心, 以确保获得最稳定的性能
//          - 5V工作模式: 0 ~ 255, 最佳取值 128
//          - 3.3V工作模式: 0 ~ 128, 最佳取值 64
//
#include "stm32f10x.h"                  // Device header

#include "as5600_sensor.h"
#include "as5600_sensor_reg.h"

#ifdef AS5600_I2C_HW
#include "bsp_i2c_hw.h"
#else
#include "bsp_i2c.h"
#endif

static void _writeReg(uint8_t RegAddress, uint8_t Data)
{
#ifdef AS5600_I2C_HW
    HwI2C_Start();                                                                                              //硬件I2C生成起始条件 并 等待EV5

    HwI2C_SendAddress(AS5600_SLAVE_ADDR, I2C_Direction_Transmitter, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);//硬件I2C发送从机地址，方向为发送 并 等待EV6
    HwI2C_SendData(RegAddress, I2C_EVENT_MASTER_BYTE_TRANSMITTING);                                             //硬件I2C发送寄存器地址 并 等待EV8
    HwI2C_SendData(Data, I2C_EVENT_MASTER_BYTE_TRANSMITTED);                                                    //硬件I2C发送数据 并 等待EV8_2

    HwI2C_Stop();                                                                                               //硬件I2C生成终止条件
#else
	MyI2C_Start();						//I2C起始
	
    MyI2C_SendByte(AS5600_SLAVE_ADDR);  //发送从机地址，读写位为0，表示即将写入
	MyI2C_WaitAck();					//接收应答
	
    MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_WaitAck();					//接收应答

	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_WaitAck();					//接收应答
	
    MyI2C_Stop();						//I2C终止
#endif
}

static void _writeArrayReg(uint8_t RegAddress, uint8_t *Data, uint16_t length)
{
#ifdef AS5600_I2C_HW
#else
	MyI2C_Start();						//I2C起始
	
    MyI2C_SendByte(AS5600_SLAVE_ADDR);  //发送从机地址，读写位为0，表示即将写入
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


static uint8_t _readReg(uint8_t RegAddress)
{
	uint8_t Data;

#ifdef AS5600_I2C_HW
    HwI2C_Start();                                                                                              //硬件I2C生成起始条件 并 等待EV5
    HwI2C_SendAddress(AS5600_SLAVE_ADDR, I2C_Direction_Transmitter, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);//硬件I2C发送从机地址，方向为发送 并 等待EV6
    HwI2C_SendData(RegAddress, I2C_EVENT_MASTER_BYTE_TRANSMITTED);                                              //硬件I2C发送寄存器地址 并 等待EV8_2

    HwI2C_Start();                                                                                              //硬件I2C生成重复起始条件 并 等待EV5
    HwI2C_SendAddress(AS5600_SLAVE_ADDR, I2C_Direction_Receiver, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);      //硬件I2C发送从机地址，方向为接收 并 等待EV6

    HwI2C_Ack(DISABLE);									                                                        //在接收最后一个字节之前提前将应答失能
	HwI2C_Stop();										                                                        //在接收最后一个字节之前提前申请停止条件
    Data = HwI2C_ReceiveData();							                                                        //接收数据寄存器
    HwI2C_Ack(ENABLE);									                                                        //将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
#else
	MyI2C_Start();						//I2C起始
    MyI2C_SendByte(AS5600_SLAVE_ADDR);  //发送从机地址，读写位为0，表示即将写入
	MyI2C_WaitAck();					//接收应答
    MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_WaitAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
    MyI2C_SendByte(AS5600_SLAVE_ADDR | 0x01);//发送从机地址，读写位为1，表示即将读取
	MyI2C_WaitAck();					//接收应答

    Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_NAck();					    //发送应答，给从机非应答，终止从机的数据输出

	MyI2C_Stop();						//I2C终止
#endif

	return Data;
}

static void _readArrayReg(uint8_t StartRegAddress, uint8_t *Data, uint16_t length)
{
#ifdef AS5600_I2C_HW
    HwI2C_Start();                                                                                              //硬件I2C生成起始条件 并 等待EV5
    HwI2C_SendAddress(AS5600_SLAVE_ADDR, I2C_Direction_Transmitter, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);//硬件I2C发送从机地址，方向为发送 并 等待EV6
    HwI2C_SendData(StartRegAddress, I2C_EVENT_MASTER_BYTE_TRANSMITTED);                                     //硬件I2C发送寄存器地址 并 等待EV8_2

    HwI2C_Start();                                                                                          //硬件I2C生成重复起始条件 并 等待EV5
    HwI2C_SendAddress(AS5600_SLAVE_ADDR, I2C_Direction_Receiver, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);       //硬件I2C发送从机地址，方向为接收 并 等待EV6
    
    for(uint16_t i = 0; i < length; i++)
    {
        if (i == length - 1) {
            HwI2C_Ack(DISABLE);									                                            //在接收最后一个字节之前提前将应答失能
	        HwI2C_Stop();										                                            //在接收最后一个字节之前提前申请停止条件
        }

        Data[i] = HwI2C_ReceiveData();		                                                                //接收一个字节数据
    }

    HwI2C_Ack(ENABLE);									                                                    //将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
#else
	MyI2C_Start();						        //I2C起始

    MyI2C_SendByte(AS5600_SLAVE_ADDR);          //发送从机地址，读写位为0，表示即将写入
	MyI2C_WaitAck();					        //接收应答

    MyI2C_SendByte(StartRegAddress);	        //发送要读取的**起始**寄存器的地址
	MyI2C_WaitAck();						    //接收应答

    MyI2C_Start();						        //I2C重复起始
    
    MyI2C_SendByte(AS5600_SLAVE_ADDR | 0x01);   //发送从机地址，读写位为1，表示即将读取
    MyI2C_WaitAck();					        //接收应答

    for(uint16_t i = 0; i < length; i++)
    {
        Data[i] = MyI2C_ReceiveByte();		    //接收指定寄存器的数据

        if (i == length - 1) {
            MyI2C_NAck();                       //发送应答，给从机非应答，终止从机的数据输出
        }
        else {
            MyI2C_Ack();                        //发送应答，给从机应答，继续等待从机的数据输出
        }
    }
	
    MyI2C_Stop();						        //I2C终止
#endif
}


void AS5600_Init(void)
{
#ifdef AS5600_I2C_HW
    HwI2C_Init();                                   //先初始化底层的I2C
#else
	MyI2C_Init();									//先初始化底层的I2C
#endif

	// /*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	// _writeReg(AS5600_PWR_MGMT_1, 0x01);    //电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	// _writeReg(AS5600_PWR_MGMT_2, 0x00);    //电源管理寄存器2，保持默认值0，所有轴均不待机
	// _writeReg(AS5600_SMPLRT_DIV, 0x09);    //采样率分频寄存器，配置采样率 (10分频)
	// _writeReg(AS5600_CONFIG, 0x06);        //配置寄存器，配置DLPF (不启用外部同步00000, 滤波最平滑110)
	// _writeReg(AS5600_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s (000, 量程满11, 000)
	// _writeReg(AS5600_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g (自测000, 量程满11, 滤波无000)
}

/**
 * @brief 获取传感器数据
 * @param Data 传入数据结构体指针
 */
void AS5600_GetData(AS5600_Data_t *Data)
{
    // // 1. 逐个读取方式
	// uint8_t DataH, DataL;
    // DataH = _readReg(AS5600_RAW_ANGLE_H);
    // DataL = _readReg(AS5600_RAW_ANGLE_L);
    // Data->RawAngle = ((DataH & 0x0F) << 8) | DataL;
    // Data->ClacAngle = (Data->RawAngle * 360) / 4096;
    
    // DataH = _readReg(AS5600_ANGLE_H);
    // DataL = _readReg(AS5600_ANGLE_L);
    // Data->Angle = ((DataH & 0x0F) << 8) | DataL;

    // // 只关注有效位: bit5(MD), bit4(ML), bit3(MH), 则掩码: 0x38 = 0011 1000
    // // - 0x00 = 0000 0000 = MD:0, ML:0, MH:0 = 未检测到磁铁
    // // - 0x08 = 0000 1000 = MD:0, ML:0, MH:1 = 未检测到磁铁, AGC最小增益溢出, 磁铁太强
    // // - 0x10 = 0001 0000 = MD:0, ML:1, MH:0 = 未检测到磁铁, AGC最大增益溢出, 磁铁太弱
    // // - 0x20 = 0010 0000 = MD:1, ML:1, MH:0 = 检测到磁铁
    // // - 0x28 = 0010 1000 = MD:1, ML:0, MH:1 = 检测到磁铁, AGC最小增益溢出, 磁铁太强
    // // - 0x30 = 0011 0000 = MD:1, ML:1, MH:0 = 检测到磁铁, AGC最大增益溢出, 磁铁太弱
    // Data->Status = _readReg(AS5600_STATUS) & 0x38;
    // Data->Agc = _readReg(AS5600_AGC);

    // DataH = _readReg(AS5600_MGNITUDE_H);
    // DataL = _readReg(AS5600_MGNITUDE_L);
    // Data->Magnitude = ((DataH & 0x0F) << 8) | DataL;

    // // 2. 批量读取方式 (更效率)
    // // 自动递增角度、原始角度和方位寄存器的地址指针
    // // 只有当地址指针设置为寄存器的高字节时，指针的这种特殊处理才有效。
    // // 所以, 读取 STATUS, AGC 这种寄存器, 

    // 只关注有效位: bit5(MD), bit4(ML), bit3(MH), 则掩码: 0x38 = 0011 1000
    // - 0x00 = 0000 0000 = MD:0, ML:0, MH:0 = 未检测到磁铁
    // - 0x08 = 0000 1000 = MD:0, ML:0, MH:1 = 未检测到磁铁, AGC最小增益溢出, 磁铁太强
    // - 0x10 = 0001 0000 = MD:0, ML:1, MH:0 = 未检测到磁铁, AGC最大增益溢出, 磁铁太弱
    // - 0x20 = 0010 0000 = MD:1, ML:1, MH:0 = 检测到磁铁
    // - 0x28 = 0010 1000 = MD:1, ML:0, MH:1 = 检测到磁铁, AGC最小增益溢出, 磁铁太强
    // - 0x30 = 0011 0000 = MD:1, ML:1, MH:0 = 检测到磁铁, AGC最大增益溢出, 磁铁太弱
    Data->Status = _readReg(AS5600_STATUS) & 0x38;

    // 读取输出数据
    // 从 AS5600_RAW_ANGLE_H 开始读取4个字节数据, 即: [ AS5600_RAW_ANGLE_H, AS5600_RAW_ANGLE_L, AS5600_ANGLE_H, AS5600_ANGLE_L ]
    uint8_t buf[8];
    _readArrayReg(AS5600_RAW_ANGLE_H, buf, 8);
    Data->RawAngle = ((buf[0] & 0x0F) << 8) | buf[1];
    Data->Angle = ((buf[2] & 0x0F) << 8) | buf[3];
    Data->ClacAngle = Data->RawAngle * AS5600_DEGREE_PER_RAWANGLE;

    // 读取状态数据
    // - 状态寄存器 与 输出寄存器 地址不连续, 需要分批读取
    // 从 AS5600_AGC 开始读取4个字节数据, 即 [ AS5600_AGC, AS5600_MGNITUDE_H, AS5600_MGNITUDE_L ]
    _readArrayReg(AS5600_AGC, buf, 3);
    Data->Agc = buf[0];
    Data->Magnitude = ((buf[1] & 0x0F) << 8) | buf[2];
}

uint16_t AS5600_GetRawAngle(void)
{
    // // 1. 逐个读取方式
	// uint8_t DataH, DataL;
    // DataH = _readReg(AS5600_RAW_ANGLE_H);
    // DataL = _readReg(AS5600_RAW_ANGLE_L);
    // return ((DataH & 0x0F) << 8) | DataL;

    // 2. 批量读取方式 (更效率)
    // 从 AS5600_RAW_ANGLE_H 开始读取2个字节数据
    // 即: AS5600_RAW_ANGLE_H, AS5600_RAW_ANGLE_L
    uint8_t buf[2];
    _readArrayReg(AS5600_RAW_ANGLE_H, buf, 2);
    return ((buf[0] & 0x0F) << 8) | buf[1];
}

uint16_t AS5600_GetAngle(void)
{
    // // // 1. 逐个读取方式
	// // uint8_t DataH, DataL;
    // // DataH = _readReg(AS5600_ANGLE_H);
    // // DataL = _readReg(AS5600_ANGLE_L);
    // // return = ((DataH & 0x0F) << 8) | DataL;

    // 2. 批量读取方式 (更效率)
    // 从 AS5600_ANGLE_H 开始读取2个字节数据
    // 即: AS5600_ANGLE_H, AS5600_ANGLE_L
    uint8_t buf[2];
    _readArrayReg(AS5600_ANGLE_H, buf, 2);
    return ((buf[0] & 0x0F) << 8) | buf[1];
}

uint16_t AS5600_GetClacAngle(void) {
    return AS5600_GetRawAngle() * AS5600_DEGREE_PER_RAWANGLE;
}

uint16_t AS5600_GetStatus(void)
{
    // 只关注有效位: bit5(MD), bit4(ML), bit3(MH), 则掩码: 0x38 = 0011 1000
    // - 0x00 = 0000 0000 = MD:0, ML:0, MH:0 = 未检测到磁铁
    // - 0x08 = 0000 1000 = MD:0, ML:0, MH:1 = 未检测到磁铁, AGC最小增益溢出, 磁铁太强
    // - 0x10 = 0001 0000 = MD:0, ML:1, MH:0 = 未检测到磁铁, AGC最大增益溢出, 磁铁太弱
    // - 0x20 = 0010 0000 = MD:1, ML:1, MH:0 = 检测到磁铁
    // - 0x28 = 0010 1000 = MD:1, ML:0, MH:1 = 检测到磁铁, AGC最小增益溢出, 磁铁太强
    // - 0x30 = 0011 0000 = MD:1, ML:1, MH:0 = 检测到磁铁, AGC最大增益溢出, 磁铁太弱
    return _readReg(AS5600_STATUS) & 0x38;
}

uint16_t AS5600_GetMagnitude(void)
{
    // // 1. 逐个读取方式
	// uint8_t DataH, DataL;
    // DataH = _readReg(AS5600_MGNITUDE_H);
    // DataL = _readReg(AS5600_MGNITUDE_L);
    // return ((DataH & 0x0F) << 8) | DataL;

    // 2. 批量读取方式 (更效率)
    uint8_t buf[4];
    _readArrayReg(AS5600_MGNITUDE_H, buf, 2);
    return ((buf[0] & 0x0F) << 8) | buf[1];
}