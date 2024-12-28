/**-------------------------------------------------------------------------------------------
** Created by:          qinyx
** Last modified Date:  2014-02-28
** Last Version:        V1.00
** Descriptions:        STM32F407嵌入式实验箱
**	  Gpio配置文件
**
**-------------------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "TIME.h"

/**********************************************************************************************************
函数名称：定时器初始化函数
输入参数：无
输出参数：无
定时器：TIM2
**********************************************************************************************************/
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 配置中断优先级分组 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); // 抢占优先级和子优先级为1:1的比例

    /* 配置 Timer2 中断 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                                // 指定中断请求通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                      // 抢占优先级设置为2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                             // 响应优先级设置为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                // 使能中断
    NVIC_Init(&NVIC_InitStructure);

    /* 开启 TIM2 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 配置 TIM2 的基础定时参数 */
    // 假设主时钟为 84MHz，预分频和自动重装值计算如下：
    // t = 1 / (主时钟频率) * 预分频值 * 自动重装值
    // t = 1 / (84M) * 8400 * 10000 = 1ms
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;                                  // 自动重装值，计数器从 0 计数到 (Period) 后溢出
    TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;                                // 预分频值，输入时钟频率为主时钟频率/(Prescaler + 1)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                        // 时钟分频因子为1
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                    // 向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* 使能自动重装值预装载 */
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* 清除溢出中断标志位 */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    /* 配置定时器更新中断（溢出中断） */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* 启动定时器 */
    TIM_Cmd(TIM2, ENABLE); // 使能 TIM2 工作
}

