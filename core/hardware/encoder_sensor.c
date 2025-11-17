// AB相编码器 (编码器接口)

#include "encoder_sensor.h"

void EncoderSensor_Init(void)
{
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // 2. 配置 GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 配置时钟源
    // 编码器接口就是一个带方向控制的外部时钟, 所以不需要配置

    // 4. 配置时基单元 (PSC, ARR, CNT)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// 不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseStructure.TIM_Period = 65536 - 1; // 自动重装器的值 (ARR), 满量程计数
	TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1; // 预分频器的值 (PSC), 不分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 不重复 (仅TIM1和TIM8有效)
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 5. 配置 输入捕获比较单元 (AB相配置)
    TIM_ICInitTypeDef TIM_ICInitStructure;
    // 5.1 通道1, 上升沿捕获, 极性不反转 (A相)
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0xF; // 滤波器, 默认值 0xF (8个时钟周期)
    // 因为用的编码器接口, 上升沿和下降沿都有效
    // 这里设置 上升沿有效 代表的其实是: 高低电平极性不反转 (即 TI1, TI2 不反相)
    // 如果需要极性反转, 则改变一个通道的即可
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 极性不反相
    // 因为用的编码器接口, 以下2个参数用不到也没作用, 设置不设置都行
    // TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 不分频, 希望每次触发都有效
    // TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直连输入
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    // 5.2 通道1, 上升沿捕获, 极性不反转 (B相)
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 极性不反相
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    // 6. 配置 编码器接口
    // 因为用的编码器接口, 上升沿和下降沿都有效
    // 这里设置 上升沿有效 代表的其实是: 高低电平极性不反转 (即 TI1, TI2 不反相)
    // 如果需要极性反转, 则改变一个通道的即可
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 7. 启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

uint16_t EncoderSensor_GetCount(void)
{
    return TIM_GetCounter(TIM3);
}

uint16_t EncoderSensor_GetCountWithReset(void)
{
    uint16_t count;
    count = TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);
    return count;
}

// static volatile uint16_t _speed;
// void TIM2_IRQHandler(void)
// {
//     if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
//     {
//         // 获取编码器计数值并重置计数
//         _speed = EncoderSensor_GetCountWithReset();

//         // 清除中断标志位
//         TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//     } 
// }
