// TB6612 驱动直流电机

#include "bsp_utils.h"
#include "pwm.h"
#include "motor.h"

#define MOTOR_FORWARD_PORT  GPIOA
#define MOTOR_FORWARD_PIN   GPIO_Pin_5 // 正向信号引脚

#define MOTOR_BACK_PORT     GPIOA
#define MOTOR_BACK_PIN      GPIO_Pin_6 // 反转信号引脚

void Motor_Init(void)
{
    PWM_Init();

    // 1. 配置电机方向控制引脚
    Utils_GPIO_CLOCK_Enable(MOTOR_FORWARD_PORT);
    Utils_GPIO_CLOCK_Enable(MOTOR_BACK_PORT);

    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = MOTOR_FORWARD_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_FORWARD_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_BACK_PIN;
    GPIO_Init(MOTOR_BACK_PORT, &GPIO_InitStructure);
}

// 需要外部5V单独供电, 靠 ST-Link 的5V 带不动
// 速度范围 -100 ~ 100
// 实际情况是 在 -50 ~ 50 范围转不起来
void Motor_SetSpeed(int8_t speed)
{
    if(speed > 0)
    {
        GPIO_SetBits(MOTOR_FORWARD_PORT, MOTOR_FORWARD_PIN);
        GPIO_ResetBits(MOTOR_BACK_PORT, MOTOR_BACK_PIN);
        PWM_SetCompare1(speed);
    }
    else
    {
        GPIO_SetBits(MOTOR_BACK_PORT, MOTOR_BACK_PIN);
        GPIO_ResetBits(MOTOR_FORWARD_PORT, MOTOR_FORWARD_PIN);
        PWM_SetCompare1(-speed);
    }
}
