// Timer

// 一些基础概念
//
// 标识
//   - CK_PSC: 定时器输入时钟频率 (单位: Hz)
//   - CK_CNT: 计数器计数频率 (单位: Hz) (通常就是 希望获得的 输出频率 Freq)
//	 - CNT: 计数器当前值
//   - PSC: 预分频器 (TIM_Prescaler)
//   - ARR: 自动重装器 (TIM_Period)
//
// 公式
//   - 计数器计数频率:  CK_CNT = CK_PSC / (PSC + 1)
//   - 计数器溢出频率:  CK_CNT_OV = CK_CNT / (ARR + 1)
//                               = CK_PSC / (PSC + 1) / (ARR + 1)
//							     = CK_PSC / ((PSC + 1) * (ARR + 1))
//   - 计数器计数时间:  CK_CNT_T = 1 / CK_CNT_OV
//                     因频率 Hz 就是 1s 内的计数次数, 所以计数时间就是 1s / 次数 = 1 / CK_CNT_OV
//
//   通过公式可以得到最终公式 (常用, 需熟悉, 大部分情况下 设置参数都是为了 需求一对比较合适的 PSC + ARR 组合而已)
//   -------------------------------------------
//   Freq =  CK_PSC / ( (PSC + 1) * (ARR + 1) )
//   -------------------------------------------
//
// 举例
//   - 实现 1s 的定时器: (即 CK_CNT_T = 1)
//     那么:
//       - CK_CNT_T = 1
//       - CK_CNT_OV = 1
//     由于:
//       - CK_PSC = 72MHz = 72 000 000
//     所以: (根据 计数器溢出频率 公式)
//       - (PSC + 1) * (ARR + 1) = 72 000 000
//     最终: (选择比较合适的组合)
//       - 组合1:
//         - PSC + 1 = 10000
//         - ARR + 1 = 7200
//       - 组合2:
//         - PSC + 1 = 100
//         - ARR + 1 = 720000
//       - 组合3:
//         - PSC + 1 = 1000000
//         - ARR + 1 = 72
//       ...
//
//   - 实现 1ms 的定时器: (即 CK_CNT_T = 0.001)
//     那么:
//       - CK_CNT_T = 0.001
//       - CK_CNT_OV = 1000
//     由于:
//       - CK_PSC = 72MHz = 72 000 000
//     所以: (根据 计数器溢出频率 公式)
//       - (PSC + 1) * (ARR + 1) = 72 000
//     最终: (选择比较合适的组合)
//       - 组合1:
//         - PSC + 1 = 1000
//         - ARR + 1 = 72
//       - 组合2:
//         - PSC + 1 = 10
//         - ARR + 1 = 7200
//       - 组合3:
//         - PSC + 1 = 100
//         - ARR + 1 = 720
//       ...

#include <stddef.h>
#include "bsp_timer.h"

static void (*onTimerTick)(void) = NULL;					// 定时器中断处理回调 (标志位清除之前)
static void (*afterTimerTick)(void) = NULL;					// 定时器中断后处理回调 (标志位清除后)

static volatile uint16_t _count = 0;

void Timer_Init(void)
{
	// 1. 使能定时器时钟
	RCC_APB2PeriphClockCmd(BSP_TIMER_RCC_PERIPH, ENABLE);

	// // 1.1 外部时钟源引脚配置
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// GPIO_InitTypeDef GPIO_InitStructure;
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // TIM2_ETR (PA0 支持TIM2的外部时钟源引脚)
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 内部上拉输入
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 2. 时钟源选择
	// 2.1 内部时钟源
	TIM_InternalClockConfig(BSP_TIMER_TIM);
	// // 2.1 外部时钟源 (无预分频, 高电平计数, 无滤波)
	// // - 外部时钟模式1: TRGI
	// //   - 输入可以是:
	// //       - ETR引脚
	// //       - 其他定时器 (为实现级联)
	// // TIM_ETRClockMode1Config(BSP_TIMER_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	// // - 外部时钟模式2: TGI    (推荐, 最简单, 最直接)
	// //    - 输入可以是:
	// //        - ETR引脚
	// TIM_ETRClockMode2Config(BSP_TIMER_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	
	// // 2.2 定时器级联
	// TIM_SelectInputTrigger(BSP_TIMER_TIM, TIM_TS_ITR2);

	// 3. 初始化时基单元 (PSC, ARR, CNT)
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
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// 不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseStructure.TIM_Period = BSP_TIMER_ARR; // 自动重装器的值
	TIM_TimeBaseStructure.TIM_Prescaler = BSP_TIMER_PSC; // 预分频器的值
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 不重复 (仅TIM1和TIM8有效)
	TIM_TimeBaseInit(BSP_TIMER_TIM, &TIM_TimeBaseStructure);

	// 清除更新标志 (避免上电即触发中断)
	TIM_ClearFlag(BSP_TIMER_TIM, TIM_FLAG_Update);

	// 4. 中断输出控制
	TIM_ITConfig(BSP_TIMER_TIM, TIM_IT_Update, ENABLE);

	// 5. NVIC配置
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置 NVIC 优先级分组 (全局只需配置一次)
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = BSP_TIMER_NVIC_IRQ_CHANNEL; // 定时器中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// 6. 启动定时器
	TIM_Cmd(BSP_TIMER_TIM, ENABLE);
}

uint16_t Timer_GetTIMCount(void)
{
	return TIM_GetCounter(BSP_TIMER_TIM);
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

void Timer_SetOnTimerTick(void (*callback)(void)) {
	onTimerTick = callback;
}

void Timer_SetAfterTimerTick(void (*callback)(void)) {
	afterTimerTick = callback;
}

static inline void _update(void) {
	_count++;
}

void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(BSP_TIMER_TIM, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(BSP_TIMER_TIM, TIM_IT_Update);
		
		_update();

		if (onTimerTick != NULL) {
			onTimerTick();
		}

		// 防止中断内运行逻辑耗时过长, 造成中断触发堆叠
		if (TIM_GetITStatus(BSP_TIMER_TIM, TIM_IT_Update) == SET)
		{
			TIM_ClearITPendingBit(BSP_TIMER_TIM, TIM_IT_Update);
		}

		if (afterTimerTick != NULL) {
			afterTimerTick();
		}
	}
}
