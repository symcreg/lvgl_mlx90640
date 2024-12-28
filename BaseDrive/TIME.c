/**-------------------------------------------------------------------------------------------
** Created by:          qinyx
** Last modified Date:  2014-02-28
** Last Version:        V1.00
** Descriptions:        STM32F407Ƕ��ʽʵ����
**	  Gpio�����ļ�
**
**-------------------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "TIME.h"

/**********************************************************************************************************
�������ƣ���ʱ����ʼ������
�����������
�����������
��ʱ����TIM2
**********************************************************************************************************/
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* �����ж����ȼ����� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); // ��ռ���ȼ��������ȼ�Ϊ1:1�ı���

    /* ���� Timer2 �ж� */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                                // ָ���ж�����ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                      // ��ռ���ȼ�����Ϊ2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                             // ��Ӧ���ȼ�����Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                // ʹ���ж�
    NVIC_Init(&NVIC_InitStructure);

    /* ���� TIM2 ʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* ���� TIM2 �Ļ�����ʱ���� */
    // ������ʱ��Ϊ 84MHz��Ԥ��Ƶ���Զ���װֵ�������£�
    // t = 1 / (��ʱ��Ƶ��) * Ԥ��Ƶֵ * �Զ���װֵ
    // t = 1 / (84M) * 8400 * 10000 = 1ms
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;                                  // �Զ���װֵ���������� 0 ������ (Period) �����
    TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;                                // Ԥ��Ƶֵ������ʱ��Ƶ��Ϊ��ʱ��Ƶ��/(Prescaler + 1)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                        // ʱ�ӷ�Ƶ����Ϊ1
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                    // ���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* ʹ���Զ���װֵԤװ�� */
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* �������жϱ�־λ */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    /* ���ö�ʱ�������жϣ�����жϣ� */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* ������ʱ�� */
    TIM_Cmd(TIM2, ENABLE); // ʹ�� TIM2 ����
}

