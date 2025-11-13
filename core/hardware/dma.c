// DMA

#include "dma.h"

// 一些基本概念
// DMA 正非常执行转运工作的必要条件:
// - 传输计数 (DMA_BufferSize) 大于0 (有效范围: 0 ~ 65535)
// - 触发源有触发信号
// - DMA使能


static uint16_t _bufferSize;

void MyDMA_Init(uint32_t fromAddr, uint32_t toAddr, uint16_t size)
{
    // 目标:
    // - 使用 DMA1
    // - 存储器(外设) -> 存储器 转运
    // - 软件触发 (通道可以任意选择)

    _bufferSize = size;

    // 1. 配置 时钟 (DMA)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 2. 配置 DMA
    DMA_InitTypeDef DMA_InitStructure;

    // 2.1 外设寄存器配置 (从)
    DMA_InitStructure.DMA_PeripheralBaseAddr = fromAddr;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 以 字节(8bit) 传输数据
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable; // 
    
    // 2.2 存储器寄存器配置 (到)
    DMA_InitStructure.DMA_MemoryBaseAddr = toAddr;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // 存储器站点地址自增

    // 2.3 传输配置
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // 外设寄存器 是数据源 (从 外设 到 存储器 的传输方向)
    DMA_InitStructure.DMA_BufferSize = size; // 有效范围: 0 ~ 65535,  (必要条件1 满足)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // 正常模式 (存储器 -> 存储器 的模式)
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable; // 软件触发 (软件触发模式会一直有触发信号, 必要条件2 满足)
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;

    // 3. 初始化 DMA
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    // 4. 使能 DMA
    DMA_Cmd(DMA1_Channel1, DISABLE); // 默认不使能 (必要条件3 不满足)
}

void MyDMA_Transfer(void) {
    DMA_Cmd(DMA1_Channel1, DISABLE); // 必须先失能, 才能修改配置
    DMA_SetCurrDataCounter(DMA1_Channel1, _bufferSize); // 重新设置数据传输计数
    DMA_Cmd(DMA1_Channel1, ENABLE); // 恢复使能

    while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET); // 等待转运完成
    DMA_ClearFlag(DMA1_FLAG_TC1);// 转运完成, 清除标志位
}