#include "bsp_utils.h"
#include "pwm.h"

// 基本公式
//
// PWM频率: Freq = CK_PSC / (PSC + 1) / (ARR + 1)
// PWM占空比: Duty = CCR / (ARR + 1)
// PWM分辨率: Reso = 1 / (ARR + 1)
// - CK_PSC: 定时器输入时钟频率 (单位: Hz)
// - PSC: 预分频器 (TIM_Prescaler)
// - ARR: 自动重装器 (TIM_Period)
// - CCR:  输出比较 (TIM_CCR1)
// - Freq: PWM频率 (单位: Hz)
// - Duty: PWM占空比 (范围: 0 ~ 1)
// - Reso: PWM分辨率 (范围: 0 ~ 1)
//

void PWM_Init(void)
{
    // // 测试用例: 基本PWM波形输出 (1kHz, 50%占空比, 1%分辨率)
    // //
    // // 目标: 配置 TIM2 产生 PWM 波形, 频率 1kHz, 占空比 50%, 分辨率 1%
    // //
    // // 在当前环境(时钟频率 72MHz)下, 套用上述的公式:
    // // - 1kHz = 1000Hz = 72MHz / (PSC + 1) / (ARR + 1)
    // // - 50% = CCR / (ARR + 1)
    // // - 1% = 1 / (ARR + 1)
    // // 那么: (借助 Timer 的输出频率计算公式, 可以推导合适的 PSC 和 ARR 组合)
    // // - ARR + 1 = 100
    // // - PSC + 1 = 720
    // // 最终 ARR, PSC, CCR 的值为:
    // // - ARR = 99   (有效值: 0 ~ 65535)
    // // - PSC = 719  (有效值: 0 ~ 65535)
    // // - CCR = 50
    // PWM_Config config =
    // {
    //     .ARR = 100 - 1,
    //     .PSC = 720 - 1,
    //     .CCR = 50,
    //     .GPIO_port = GPIOA,
    //     .GPIO_pin = GPIO_Pin_0, // TIM2_CH1_ETR 默认对应 PA0
    //     // .GPIO_pin = GPIO_Pin_15; // TIM2_CH1_ETR 重映射到 PA15
    // };

    // // 测试用例: SG90舵机PWM输出 (SG90, 180°)
    // // - 脉冲周期: 20ms
    // // - 旋转角度:
    // //   - 0.5ms: 0°
    // //   - 1ms: 45°
    // //   - 1.5ms: 90°
    // //   - 2ms: 135°
    // //   - 2.5ms: 180°
    // //
    // // 在当前环境(时钟频率 72MHz)下:
    // // - PWM频率: 1s / 20ms = 1s / 0.02s = 50Hz
    // //   
    // //   即: 50 = 72MHz / (PSC + 1) / (ARR + 1)
    // //          = 72000000 / (PSC + 1) / (ARR + 1)
    // //
    // // 选取合适的 `ARR + 1` 和 `PSC + 1` 的组合即可:
    // // 比如 (好计算):
    // // - PSC + 1 = 72
    // // - ARR + 1 = 20000
    // //
    // // 即 20000 (一次计数重装) 对应 20ms (一个脉冲周期)
    // // 也就是 1ms 计数1000 (即计算比例 1000 / 1 = 1000), 比较方便后续计算占空比
    // //
    // // 那么:
    // // - 0.5ms(0度) 对应 500 = (1000 / 1) * 0.5
    // // ..
    // // - 1ms(45度) 对应 1000 = (1000 / 1) * 1
    // // ..
    // // - 1.5ms(90度) 对应 1500 = (1000 / 1) * 1.5
    // // ..
    // // - 2ms(135度) 对应 2000 = (1000 / 1) * 2
    // // ..
    // // - 2.5ms(180度) 对应 2500 = (1000 / 1) * 2.5
    // //
    // // 所以:
    // // - CCR 取值范围: 500 ~ 2500
    // //
    // PWM_Config config =
    // {
    //     .ARR = 20000 - 1,
    //     .PSC = 72 - 1,
    //     .CCR = 0, // 初始默认给0, 后续按需修改调整占空比
    //     .GPIO_port = GPIOA,
    //     .GPIO_pin = GPIO_Pin_1, // TIM2_CH2_ETR 默认对应 PA1
    // };

    // // 测试用例: TB6612 驱动直流电机PWM输出
    // // 驱动电机需要频率高一些才行, 而且蜂鸣声在 20kHz 之上人耳就不容易感知了, 
    // // 所以这里选择频率 20kHz = 72MHz / 100 / 36
    // PWM_Config config =
    // {
    //     .ARR = 100 - 1,
    //     .PSC = 36 - 1,
    //     .CCR = 0, // 
    //     .GPIO_port = GPIOA,
    //     .GPIO_pin = GPIO_Pin_2, // 使用 TIM2_CH3 通道产生PWM
    // };

    // 测试用例: TB6612 驱动平衡车2轮子(TT直流电机)PWM输出
    PWM_Config config =
    {
        .ARR = 100 - 1,
        .PSC = 36 - 1,
        .CCR = 0, // 
        .GPIO_port = GPIOA,
        .GPIO_pin = GPIO_Pin_0 | GPIO_Pin_1, // 使用 TIM2_CH1_ETR (PA0) 和 TIM2_CH2 (PA1) 2个通道产生2个PWM
    };

    // 1. 配置 时钟 (定时器, GPIO)
    Utils_RCC_PeriphClock_Enable(RCC_APB1Periph_TIM2);
    Utils_RCC_PeriphClock_Enable(RCC_APB2Periph_GPIOA);

    // // 1.1 GPIO部分重映射 (将TIM2_CH1从PA0重映射到PA15)
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 使能复用功能时钟
    // GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); // TIM2部分重映射到PA15
    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 禁用JTAG, 释放PA15

    // 2. 配置 GPIO (复用推挽输出)
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = config.GPIO_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出 (引脚输出控制权交给外设)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(config.GPIO_port, &GPIO_InitStructure);

    // 3. 配置 时钟源
	// 3.1 内部时钟源
	TIM_InternalClockConfig(PWM_TIM);

    // 4. 配置 时基单元 (PSC, ARR, CNT)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// 不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseStructure.TIM_Period = config.ARR; // 自动重装器的值 (ARR)
	TIM_TimeBaseStructure.TIM_Prescaler = config.PSC; // 预分频器的值 (PSC)
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 不重复 (仅TIM1和TIM8有效)
	TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);

    // 5. 配置 输出比较单元
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure); // 设置默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = config.CCR; // 初始输出比较值 (CCR)
    // TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure); // 使用输出比较单元1 (测试用例: 基本PWM)
    // TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure); // 使用输出比较单元2 (测试用例: SG90舵机PWM)
    // TIM_OC3Init(PWM_TIM, &TIM_OCInitStructure); // 使用输出比较单元3 (测试用例: TB6612直流电机PWM)
    TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);    // 初始化PWM1, 对应PA0 (测试用例: TB6612 驱动平衡车2轮子(TT直流电机)PWM输出)
    TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);    // 初始化PWM2, 对应PA1 (测试用例: TB6612 驱动平衡车2轮子(TT直流电机)PWM输出)

    // 6. 启动定时器
    TIM_Cmd(PWM_TIM, ENABLE);


    // ===== 常用的单独配置 =====
	// 修改CCR寄存器的值 (常用于运行时修改占空比)
    // TIM_SetCompare1(PWM_TIM, 5000);
    // TIM_SetCompare2(PWM_TIM, 5000);
    // TIM_SetCompare3(PWM_TIM, 5000);
    // TIM_SetCompare4(PWM_TIM, 5000);
}

uint16_t PWM_GetTIMCount(void)
{
    return TIM_GetCounter(PWM_TIM);
}

void PWM_SetCompare1(uint16_t compare)
{
    // TIM_SetCompare1(PWM_TIM, compare); // 测试用例: 基本PWM
    // TIM_SetCompare2(PWM_TIM, compare); // 测试用例: SG90舵机PWM
    // TIM_SetCompare3(PWM_TIM, compare); // 测试用例: TB6612直流电机PWM
    TIM_SetCompare1(PWM_TIM, compare); // 测试用例: 测试用例: TB6612 驱动平衡车2轮子(TT直流电机)PWM输出
}

void PWM_SetCompare2(uint16_t compare)
{
    TIM_SetCompare2(PWM_TIM, compare); // 测试用例: 测试用例: TB6612 驱动平衡车2轮子(TT直流电机)PWM输出
}

uint16_t PWM_GetCompare1(void)
{
    // return TIM_GetCapture1(PWM_TIM); // 测试用例: 基本PWM
    // return TIM_GetCapture2(PWM_TIM); // 测试用例: SG90舵机PWM
    // return TIM_GetCapture3(PWM_TIM); // 测试用例: TB6612直流电机PWM
    return TIM_GetCapture1(PWM_TIM); // 测试用例: 测试用例: TB6612 驱动平衡车2轮子(TT直流电机)PWM输出
}

uint16_t PWM_GetCompare2(void)
{
    return TIM_GetCapture2(PWM_TIM); // 测试用例: 测试用例: TB6612 驱动平衡车2轮子(TT直流电机)PWM输出
}

void PWM_SetPrescaler(uint16_t prescaler)
{
    TIM_PrescalerConfig(PWM_TIM, prescaler, TIM_PSCReloadMode_Immediate);
}
