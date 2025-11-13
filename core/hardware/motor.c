// TB6612 驱动直流电机

#include "pwm.h"
#include "motor.h"

#define MOTOR_FORWARD_PIN   GPIO_Pin_5 // 正向信号引脚
#define MOTOR_BACK_PIN      GPIO_Pin_6 // 反转信号引脚

void Motor_Init(void)
{
    PWM_Init();

    // 配置电机方向控制引脚
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = MOTOR_FORWARD_PIN | MOTOR_BACK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 需要外部5V单独供电, 靠 ST-Link 的5V 带不动
// 速度范围 -100 ~ 100
// 实际情况是 在 -50 ~ 50 范围转不起来
void Motor_SetSpeed(int8_t speed)
{
    if(speed > 0)
    {
        GPIO_SetBits(GPIOA, MOTOR_FORWARD_PIN);
        GPIO_ResetBits(GPIOA, MOTOR_BACK_PIN);
        PWM_SetCompare(speed);
    }
    else
    {
        GPIO_SetBits(GPIOA, MOTOR_BACK_PIN);
        GPIO_ResetBits(GPIOA, MOTOR_FORWARD_PIN);
        PWM_SetCompare(-speed);
    }
}