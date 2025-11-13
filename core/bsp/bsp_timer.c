#include "stm32f10x.h"
#include "bsp_timer.h"

static volatile uint16_t _count = 0;

void Timer_Init(void)
{
	// 1. 使能定时器时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// // 1.1 外部时钟源引脚配置
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// GPIO_InitTypeDef GPIO_InitStructure;
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // TIM2_ETR (PA0 支持TIM2的外部时钟源引脚)
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 内部上拉输入
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 2. 时钟源选择
	// 2.1 内部时钟源
	TIM_InternalClockConfig(TIM2);
	// // 2.1 外部时钟源 (无预分频, 高电平计数, 无滤波)
	// TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);

	// 3. 初始化时基单元 (PSC, ARR, CNT)
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// 不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	// 计算说明
	// - 公式:
	//       t = 1 / f
	//         = 1 / (CK_PSC * (PSC + 1) * (ARR + 1))
	// - t: 定时器周期 (单位: 秒)
	// - f: 时钟频率 (TIM_ClockDivision)
	// - CK_PSC: 定时器输入时钟频率 (单位: Hz)
	// - PSC: 预分频器 (TIM_Prescaler)
	// - ARR: 自动重装器 (TIM_Period)
	// 
	// 举例: 1ms定时器
	// - 目标:
	//        t = 1ms = 0.001s
	//        CK_PSC = 72MHz = 72,000,000Hz
	// - 计算:
	//        0.001 = 1 / (72,000,000 * (PSC + 1) * (ARR + 1))
	// - 选择:
	//        PSC = 7200 - 1 = 7199   (有效值: 0 ~ 65535)
	//        ARR = 10000 - 1 = 9999  (有效值: 0 ~ 65535)
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1; // 自动重装器的值
	TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1; // 预分频器的值
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 不重复 (仅TIM1和TIM8有效)
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// 清除更新标志 (避免上电即触发中断)
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	// 4. 中断输出控制
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	// 5. NVIC配置
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置 NVIC 优先级分组 (全局只需配置一次)
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; // 定时器2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// 6. 启动定时器
	TIM_Cmd(TIM2, ENABLE);
}

uint16_t Timer_GetTIMCount(void)
{
	return TIM_GetCounter(TIM2);
}

uint16_t Timer_GetCount(void)
{
	return _count;
}

uint16_t Timer_GetCountWithReset(void)
{
	uint16_t count = _count;
	_count = 0;
	return count;
}

static void _update(void) {
	_count++;
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		_update();

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}