// 定时器计数器

#include "counter_sensor.h"

static volatile uint32_t _count;

void CounterSensor_Init(void)
{
    // 1. 配置 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // EXTI外部中断线需要开启AFIO时钟

    // 2. 配置 GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 配置外部中断线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14); // 不要忘记开启 AFIO时钟

    // 4. 配置外部中断
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // 中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 5. 配置 NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; // EXTI10到15中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CounterSensor_Reset(void)
{
    _count = 0;
}

uint32_t CounterSensor_GetCount(void)
{
    return _count;
}

uint32_t CounterSensor_GetCountWithReset(void)
{
    uint32_t count = _count;
    _count = 0;
    return count;
}

static void _update(void)
{
    _count++;
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14) == SET)
    {
        // 处理中断
        _update();

        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
}
