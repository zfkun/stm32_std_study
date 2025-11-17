// ADC

#include "bsp_utils.h"
#include "ad.h"

// 一些基本概念和公式
//
// AD转换步骤: 采样, 保持, 量化, 编码
//
// STM32 ADC的总转换时间: T_ADC = 采样时间 + 12.5个ADC周期(ADCCLK)
// - ADCCLK: ADC周期 (最大 14MHz)
// - 采样时间: 采样保持的时间 (越大越能减少毛刺干扰, 但转换时间就越长)
// - 12.5个ADC周期: 量化编码花费的时间 (因为是 12位ADC, 多的0.5周期其他用处)
//
// 举例:
///  当 ADCCLK = 14MHz, 采样时间为 1.5个ADC周期
//   则: T_ADC = 1.5 + 12.5 = 14.5个ADC周期 = 1us
//   这就是 时间最快1us的原因 (采样时间越长, 越达不到1us, 当然也可以 ADCCLK 超频高于14MHz, 但稳定性欠佳)
//
// ADC的通道对应关系表格 (STM32F103C8T6 仅支持 ADC1, ADC2):
//      Channel         |   ADC1    |   ADC2    |   ADC3
//      ------------------------------------------------
//      ADC_Channel_0   |   PA0     |   PA0     |   PA0
//      ADC_Channel_1   |   PA1     |   PA1     |   PA1
//      ADC_Channel_2   |   PA2     |   PA2     |   PA2
//      ADC_Channel_3   |   PA3     |   PA3     |   PA3
//      ADC_Channel_4   |   PA4     |   PA4     |   PA4
//      ADC_Channel_5   |   PA5     |   PA5     |   PA5
//      ADC_Channel_4   |   PA4     |   PA4     |   PF6
//      ADC_Channel_5   |   PA5     |   PA5     |   PF7
//      ADC_Channel_6   |   PA6     |   PA6     |   PF8
//      ADC_Channel_7   |   PA7     |   PA7     |   PF9
//      ADC_Channel_8   |   PB0     |   PB0     |   PF10
//      ADC_Channel_9   |   PB1     |   PB1     |   -
//      ADC_Channel_10  |   PC0     |   PC0     |   PC0
//      ADC_Channel_11  |   PC1     |   PC1     |   PC1
//      ADC_Channel_12  |   PC2     |   PC2     |   PC2
//      ADC_Channel_13  |   PC3     |   PC3     |   PC3
//      ADC_Channel_14  |   PC4     |   PC4     |   -
//      ADC_Channel_15  |   PC5     |   PC5     |   -
//      ADC_Channel_16  | 温度传感器 |   -       |   -
//      ADC_Channel_17  |内部参考电压|   -       |   -



void AD_Init(void)
{
    // 目标:
    // - 使用 ADC1
    // - 使用 PA0
    // - 单次转换模式
    // - 非扫描模式

    // 1. 配置 时钟 (ADC, GPIO)
    Utils_RCC_PeriphClock_Enable(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // ADC时钟为 APB2(PCLK2)时钟 / 6 = 12MHz
    
    // 2. 配置 GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // 单通道
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // 多通道
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 模拟输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 配置 多路开关 (填充通道任务)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5); // 只用PA0一个通道, 非扫描模式

    // 4. 配置 ADC转换器
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // 单路模式
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 外部触发源: 无
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 连续转换模式: 否 (单次转换模式)
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 扫描模式: 否
    ADC_InitStructure.ADC_NbrOfChannel = 1; // 通道数 (非扫描模式下这个参数多少都不起作用, 都按1扫的)
    ADC_Init(ADC1, &ADC_InitStructure);

    // 5. 配置 模拟看门狗
    // ADC_AnalogWatchdogInitTypeDef ADC_AnalogWatchdogInitStructure;
    // ADC_AnalogWatchdogInitStructure.ADC_AnalogWatchdog = ADC_AnalogWatchdog_SingleChannel;
    // ADC_AnalogWatchdogInitStructure.ADC_Channel = ADC_Channel_0;
    // ADC_AnalogWatchdogInit(ADC1, &ADC_AnalogWatchdogInitStructure);

    // 6. 开关控制
    ADC_Cmd(ADC1, ENABLE);

    // 7. 校准
    ADC_ResetCalibration(ADC1); // 重置校准
    while(ADC_GetResetCalibrationStatus(ADC1)); // 等待重置完成
    ADC_StartCalibration(ADC1); // 开始校准
    while(ADC_GetCalibrationStatus(ADC1)); // 等待校准完成

    // // 8. 如果设置的 连续转换模式, 则 这里调用一次 软件触发 即可, 后续会连续自动转并更新结果
    // ADC_SoftwareStartConvCmd(ADC1, ENABLE); // 软件触发转换
}

uint16_t AD_GetValue(void)
{
    // 若 ADC_ContinuousConvMode 设置的 DISABLE (单次转换模式)
    // 则 在获取结果前, 要执行 步骤1 (软件触发转换) 和 步骤2 (等待转换完成) 才能得到结果
    // 若 ADC_ContinuousConvMode 设置为 ENABLE (连续转换模式)
    // 则 步骤1 和 步骤2 就不需要了, Init 阶段执行一次 步骤1 即可

    // 1. 开始转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // 软件触发转换

    // 2. 等待转换完成
    // 转换耗时: 通道采样周期 + 转换周期 = 55.5 + 12.5 = 68.5个ADC周期
    // - 通道采样周期: ADC_SampleTime_55Cycles5 = 55.5个ADC周期
    // - 转换周期: 固定的12.5个ADC周期
    // 由于前面配置了 ADCCLK 是 72MHz的6分频 (RCC_PCLK2_Div6) 即 12MHz
    // 也就是, 转换要以 12MHz 进行 68 个周期才能完成
    // 那么:
    // 耗时 = 周期时间 * 周期数 = 1/12MHz * 68 ≈ 5.6us
    // - 周期时间: 1 / 频率 = 1 / 12MHz
    // - 周期数: 68
    // 所以, 耗时: 5.6us
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET); // 转换完成后, 会自动设置EOC标志位

    // 3. 读取转换结果
    return ADC_GetConversionValue(ADC1);
}

// 借助 单通道 + 非扫描 模式下, 自行手工更换 第一位通道 的方式 实现 多通道扫描 的效果
uint16_t  AD_GetValueWithChannel(uint8_t ADC_Channel)
{
    // 更新填充通道
    ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5);

    return AD_GetValue();
}
