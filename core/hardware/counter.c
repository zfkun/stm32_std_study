// 对射式红外计数测速模块
// 借助外部中断实现电机测速

#include <stddef.h>
#include "bsp_utils.h"
#include "bsp_timer.h"
#include "counter.h"

static void (*onCounter1Tick)(void) = NULL;			// 定时器中断处理回调1 (标志位清除之前)
static void (*afterCounter1Tick)(void) = NULL;		// 定时器中断后处理回调1 (标志位清除后)
static void (*onCounter2Tick)(void) = NULL;			// 定时器中断处理回调2 (标志位清除之前)
static void (*afterCounter2Tick)(void) = NULL;		// 定时器中断后处理回调2 (标志位清除后)

static volatile uint32_t _count1, _count2;

void Counter_Init(void)
{
    // 1. 配置 时钟 (GPIO, AFIO)
    Utils_RCC_PeriphClock_Enable(COUNTER_RCC_PERIPH); // EXTI外部中断线需要开启AFIO时钟

    // 2. 配置 GPIO
    // - 外部输入中断, GPIO 官方手册推荐 浮空输入 | 上拉输入 | 下拉输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = COUNTER_PIN_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 上拉输入
    GPIO_Init(COUNTER_PORT_1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = COUNTER_PIN_2;
    GPIO_Init(COUNTER_PORT_2, &GPIO_InitStructure);

    // 3. 配置 AFIO (外部中断线)
    GPIO_EXTILineConfig(COUNTER_PORT_SOURCE_1, COUNTER_PIN_SOURCE_1); // 不要忘记开启AFIO时钟
    GPIO_EXTILineConfig(COUNTER_PORT_SOURCE_2, COUNTER_PIN_SOURCE_2); // 不要忘记开启AFIO时钟

    // 4. 配置 EXTI (外部中断)
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = COUNTER_EXTI_LINE_1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // 中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = COUNTER_EXTI_LINE_2;
    EXTI_Init(&EXTI_InitStructure);

    // 5. 配置 NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = COUNTER_NVIC_IRQ_1; // 选择匹配的中断通道 (根据 EXTI_Line)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = COUNTER_NVIC_IRQ_2; // 选择匹配的中断通道 (根据 EXTI_Line)
    NVIC_Init(&NVIC_InitStructure);
}

void Counter_Reset(void)
{
    _count1 = 0;
    _count2 = 0;
}

uint32_t Counter_GetCount1(void)
{
    return _count1;
}

uint32_t Counter_GetCount1WithReset(void)
{
    uint32_t count = _count1;
    _count1 = 0;
    return count;
}

uint32_t Counter_GetCount2WithReset(void)
{
    uint32_t count = _count2;
    _count2 = 0;
    return count;
}

void Counter_SetOnCounter1Tick(void (*callback)(void)) {
	onCounter1Tick = callback;
}

void Counter_SetOnCounter2Tick(void (*callback)(void)) {
	onCounter2Tick = callback;
}

void Counter_SetAfterCounter1Tick(void (*callback)(void)) {
	afterCounter1Tick = callback;
}

void Counter_SetAfterCounter2Tick(void (*callback)(void)) {
	afterCounter2Tick = callback;
}


#if COUNTER_NVIC_IRQ_1 == COUNTER_NVIC_IRQ_2
void COUNTER_NVIC_IRQ_HANDLER_1(void)
{
    if (EXTI_GetITStatus(COUNTER_EXTI_LINE_1) == SET)
    {
        EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_1);

        _count1++;

        if (onCounter1Tick != NULL) onCounter1Tick();

        // 防止冲断嵌套重叠
        if (EXTI_GetITStatus(COUNTER_EXTI_LINE_1) == SET)
        {
            EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_1);
        }

        if (afterCounter1Tick != NULL) afterCounter1Tick();
    }

    if (EXTI_GetITStatus(COUNTER_EXTI_LINE_2) == SET)
    {
        EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_2);

        _count2++;

        if (onCounter2Tick != NULL) onCounter2Tick();

        // 防止冲断嵌套重叠
        if (EXTI_GetITStatus(COUNTER_EXTI_LINE_2) == SET)
        {
            EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_2);
        }

        if (afterCounter2Tick != NULL) afterCounter2Tick();
    }
}
#else
void COUNTER_NVIC_IRQ_HANDLER_1(void)
{
    if (EXTI_GetITStatus(COUNTER_EXTI_LINE_1) == SET)
    {
        EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_1);

        _count1++;

        if (onCounter1Tick != NULL) onCounter1Tick();

        // 防止冲断嵌套重叠
        if (EXTI_GetITStatus(COUNTER_EXTI_LINE_1) == SET)
        {
            EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_1);
        }

        if (afterCounter1Tick != NULL) afterCounter1Tick();
    }
}

void COUNTER_NVIC_IRQ_HANDLER_2(void)
{

    if (EXTI_GetITStatus(COUNTER_EXTI_LINE_2) == SET)
    {
        EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_2);

        _count2++;

        if (onCounter2Tick != NULL) onCounter2Tick();

        // 防止冲断嵌套重叠
        if (EXTI_GetITStatus(COUNTER_EXTI_LINE_2) == SET)
        {
            EXTI_ClearITPendingBit(COUNTER_EXTI_LINE_2);
        }

        if (afterCounter2Tick != NULL) afterCounter2Tick();
    }
}
#endif

