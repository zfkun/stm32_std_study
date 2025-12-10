#ifndef __AS5600_REG_H
#define __AS5600_REG_H

#define AS5600_SLAVE_ADDR       0x36 << 1   // 从机地址 (7位地址, 已移位)

// 配置寄存器
//
// CONF (定制配置)
//   - 位说明
//     0x07:    Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
//              -----------------------------------------------------
//                   |      |  WD  |      FTH(2:0)      |   SF(1:0)
//     0x08:    Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
//              -----------------------------------------------------
//                PWMF(1:0) |  OUTS(1:0)  |  HYST(1:)   |   PM(1:0)
//   - PM: 电源模式
//     - 00: NOM
//     - 01: LPM1
//     - 10: LPM2
//     - 11: LPM3
//   - HYST: 滞后
//     - 00: OFF
//     - 01: 1LSB
//     - 10: 2LSBs
//     - 11: 3LSBs
//   - OUTS: 输出级
//     - 00: 模拟量 (GND和VDD之间的全量程范围 0% ~ 100%)
//     - 01: 模拟量 (GND和VDD之间的减小量程范围 0% ~ 90%)
//     - 10: 数字PWM
//   - PWMF: PWM频率
//     - 00: 115Hz
//     - 01: 230Hz
//     - 10: 460Hz
//     - 11: 920Hz
//   - SF: 慢速过滤器
//     - 00: 16x
//     - 01: 8x
//     - 10: 4x
//     - 11: 2x
//   - FTH: 快速滤波器阈值
//     - 000: 仅慢速过滤器
//     - 001: 6LSBs
//     - 010: 7LSBs
//     - 011: 9LSBs
//     - 100: 18LSBs
//     - 101: 21LSBs
//     - 110: 24LSBs
//     - 111: 10LSBs
//   - WD: 看门狗
//     - 00: OFF
//     - 01: ON
//
#define AS5600_ZMCO             0x00    // 校准完成
#define AS5600_ZPOS_H           0x01    // 配置起始位置高位字节 (11:8, 即0 ~ 4bit位有用)
#define AS5600_ZPOS_L           0x02    // 配置起始位置低位字节 (7:0, 8bit都有用)
#define AS5600_MPOS_H           0x03    // 配置结束位置高位字节 (11:8, 即0 ~ 4bit位有用)
#define AS5600_MPOS_L           0x04    // 配置结束位置低位字节 (7:0, 8bit都有用)
#define AS5600_MANG_H           0x05    // 配置最大角度高位字节 (11:8, 即0 ~ 4bit位有用)
#define AS5600_MANG_L           0x06    // 配置最大角度低位字节 (7:0, 8bit都有用)
#define AS5600_CONF_H           0x07    // 定制AS5600配置高位字节 (0 ~ 5bit位有用)
#define AS5600_CONF_L           0x08    // 定制AS5600配置低位字节 (8bit都有用)

// 输出寄存器
// RAW ANGLE (未缩放,未修改的角度值)
// ANGLE (缩放, 修改后的角度值)
// 角度寄存器在 360 度范围的极限处有一个 10-LSB 的滞后，以避免在一次旋转中出现不连续点或输出踌躇。
// 在一次旋转过程中，输出会出现混叠。
#define	AS5600_RAW_ANGLE_H	    0x0C        // 原始角度高位字节 (11:8, 即0 ~ 4bit位有用)
#define	AS5600_RAW_ANGLE_L	    0x0D        // 原始角度低位字节 (7:0, 8bit都有用)
#define	AS5600_ANGLE_H    	    0x0E        // 角度高位字节 (11:8, 即0 ~ 4bit位有用)
#define	AS5600_ANGLE_L    	    0x0F        // 角度低位字节 (7:0, 8bit都有用)

// 状态寄存器
// STATUS (状态)
//   - 位说明
//     0x0B:    Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
//              -----------------------------------------------------
//                   |      |  MD  |  ML  |  MH  |      |      |
//   - MH: AGC最小增益溢出, 磁铁太强
//   - ML: AGC最大增益溢出, 磁铁太弱
//   - MD: 检测到磁铁
// AGC (自动增益控制)
//   - 5V工作模式: 计数范围 0 ~ 255
//   - 3.3V工作模式: 计数范围 0 ~ 128
//   为了获得最稳定的性能，增益值应在其范围的中心。可以通过调整物理系统的气隙来达到该值。
// MAGNITUDE (磁铁磁场强度)
//   - 指示内部 CORDIC 的幅值
#define AS5600_STATUS           0x0B        // MD: bit5,  ML: bit4,  MH: bit3
#define AS5600_AGC              0x1A        // 自动增益控制
#define AS5600_MGNITUDE_H       0x1B        // 磁铁磁场强度高位字节 (11:8, 即0 ~ 4bit位有用)
#define AS5600_MGNITUDE_L       0x1C        // 磁铁磁场强度低位字节 (7:0, 8bit都有用)

#endif
