// 输入捕获

#include "bsp_utils.h"
#include "ic.h"

// 基本公式 - 频率测量
//
// PS: 暂时都以 向上计数 做测量为大前提环境
//
// 测频法 (适合高频): fx = N / T
// - T: 闸门时间
// - N: 在闸门时间T内, 对上升沿计次
// 测周法 (适合低频): fx = fc / N
// - fc: 标准频率, 即 Freq = 72MHz / (ARR + 1) / (PSC + 1)
// - N: 两个上升沿内, 以标准频率fc计次
// 中界频率 (测频法与测周法误差相等的频率点): fm² = fc / T
// - fc: 标准频率
// - T: 闸门时间

typedef struct {
    uint16_t ARR; // 自动重装器
    uint16_t PSC; // 预分频器
    uint16_t GPIO_pin; // GPIO引脚
    TIM_TypeDef *TIM;
    uint16_t FREQ_Channel; // 频率捕获通道
} IC_Config;

void IC_Init(void)
{
    // 测试用例: 输入捕获模式测PWM数据
    //
    // 目标: 配置 TIM3 测量 TIM2 产生的 PWM 数据
    //
    // 捕获通道分配: (1 和 2 是一对, 3 和 4 是一对)
    // - 通道1: 测量频率, 上升沿捕获
    // - 通道2: 测量占空比, 下降沿捕获
    //
    // 当前环境 (时钟频率 72MHz)
    // - PSC: 预分频器 (TIM_Prescaler), 此值决定了 fc (标准频率) 的值
    // - ARR: 自动重装器 (TIM_Period)
    IC_Config config =
    {
        .ARR = 65536 - 1, // 设置最大量程65535, 防计数溢出
        .PSC = 72 - 1, // 为了测量 1MHz 的目标
        .GPIO_pin = GPIO_Pin_6, // TIM3_CH1 默认对应 PA6
        .TIM = TIM3,
        .FREQ_Channel = TIM_Channel_1, // 通道1用于频率测量捕获
    };


    // 1. 配置 时钟 (定时器, GPIO)
    Utils_RCC_PeriphClock_Enable(RCC_APB1Periph_TIM3 | RCC_APB2Periph_GPIOA); // TIM2 被用于PWM输出, 要换个TIM3用于捕获

    // 2. 配置 GPIO (复用推挽输出)
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = config.GPIO_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 时钟源
	TIM_InternalClockConfig(config.TIM);

    // 4. 配置 时基单元 (PSC, ARR, CNT)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// 不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseStructure.TIM_Period = config.ARR;
	TIM_TimeBaseStructure.TIM_Prescaler = config.PSC;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 不重复 (仅TIM1和TIM8有效)
	TIM_TimeBaseInit(config.TIM, &TIM_TimeBaseStructure);

    // PS: 基于当前配置的 (ARR + 1) = 65536 和 (PSC + 1) = 72
    //     可以算出实际可测量的最低频率:
    //         Freq / 65535 = 72MHz / (ARR + 1) / (PSC + 1) / 65535
    //                      = 1MHz / 65535
    //                      ≈ 15Hz
    //     想提高可测量频率 上限, 可以增大 ARR, 但会增加计数器的开销
    //     想提高可测量频率 下限, 可以增大 PSC, ti 

    // 5. 配置 输入捕获比较单元
    TIM_ICInitTypeDef TIM_ICInitStructure;
    // 5.1 通道1, 上升沿捕获, 直连输入 (为实现计算频率)
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = config.FREQ_Channel;
    TIM_ICInitStructure.TIM_ICFilter = 0xF; // 滤波器, 默认值 0xF (8个时钟周期)
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿捕获
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 不分频, 希望每次触发都有效
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直连输入
    TIM_ICInit(config.TIM, &TIM_ICInitStructure);
    // // 5.2 通道2, 下降沿捕获, 交叉输入 (为实现计算占空比) (手工, 但更清晰)
    // TIM_ICStructInit(&TIM_ICInitStructure);
    // TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; // TIM2_CH2
    // TIM_ICInitStructure.TIM_ICFilter = 0xF; // 滤波器, 默认值 0xF (8个时钟周期)
    // TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; // 上升沿捕获
    // TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 不分频, 希望每次触发都有效
    // TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI; // 交叉输入
    // TIM_ICInit(config.TIM, &TIM_ICInitStructure);
    // 5.2 通道2, 下降沿捕获, 交叉输入 (为实现计算占空比) (内置函数, 更简洁, 自动配置)
    TIM_PWMIConfig(config.TIM, &TIM_ICInitStructure);

    // 6. 配置 触发源
    TIM_SelectInputTrigger(config.TIM, TIM_TS_TI1FP1); // 滤波后的 定时器输入2 (TIM2) 作为触发源

    // 7. 配置 主从模式
    TIM_SelectSlaveMode(config.TIM, TIM_SlaveMode_Reset); // 复位模式, 选中的触发输入的上升沿重新初始化计数器, 并产生一个更新寄存器信号

    // 6. 启动定时器
    TIM_Cmd(config.TIM, ENABLE);
}


uint32_t IC_GetFreq(void)
{
    // 根据测周法: fx = fc / N
    // - PSC = 72 - 1
    // - fc = 72MHz / (PSC + 1) = 1MHz = 1000000
    // - N = ARR + 1 (从API读取输入捕获值)
    
    // // 最终值无偏差
    // return 1000000 / TIM_GetCapture1(TIM3);

    // 最终值有误差 1
    return 1000000 / (TIM_GetCapture1(TIM3) + 1);
}

/**
 * 获取当前占空比 (百分比整数)
 * 范围: 0 ~ 100
 */
uint32_t IC_GetDuty(void)
{
    // 通道1 捕获的是 一个周期时长
    // 通道2 捕获的是 一个周期内的高电平持续时长
    // 那么, 占空比 = 高电平持续时长 / 总时长

    // // 占空比无误差
    // uint16_t c1 = TIM_GetCapture1(TIM3);
    // uint16_t c2 = TIM_GetCapture2(TIM3);

    // 占空比有偏差 1
    uint16_t c1 = TIM_GetCapture1(TIM3) + 1;
    uint16_t c2 = TIM_GetCapture2(TIM3) + 1;

    if (c1 == 0 || c2 == 0) return 0;

    return c2 * 100 / c1;
}

/**
 * 获取当前占空比 (浮点数)
 * 范围: 0 ~ 1
 */
float IC_GetDutyFloat(void)
{
    // 通道1 捕获的是 一个周期时长
    // 通道2 捕获的是 一个周期内的高电平持续时长
    // 那么, 占空比 = 高电平持续时长 / 总时长

    // // 占空比无误差
    // uint16_t c1 = TIM_GetCapture1(TIM3);
    // uint16_t c2 = TIM_GetCapture2(TIM3);

    // 占空比有偏差 1
    uint16_t c1 = TIM_GetCapture1(TIM3) + 1;
    uint16_t c2 = TIM_GetCapture2(TIM3) + 1;

    if (c1 == 0 || c2 == 0) return 0.0f;

    return (float)c2 / (float)c1;
}
