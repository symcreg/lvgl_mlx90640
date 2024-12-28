#include "stm32f4xx.h"
#include "DCMotor.h"
#include "delay.h"
#include "PWM.h"

/******************************************************************************************
*�������ƣ�void RelayGpio_Init(void)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵����LED��ʼ��
//DCMOTOR_A      PC13           //  ��ת
//DCMOTOR_B      PE2            //  ��ת
//DCMOTOR_S      PE4            //  �����������
*******************************************************************************************/
void DCMotorGpio_Init(void)
{    
    GPIO_InitTypeDef   GPIO_InitStructure;
    
    //DCMOTOR_A      PC13
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
//    DCMOTOR_A_L;
    DCMOTOR_B_L;
    
    TIM10_PWM_Init(1000 - 1, 84 - 1);                                   //  84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ1000������PWMƵ��Ϊ 1M/1000 = 1Khz.
    
    TIM_SetCompare1(TIM10, 0);	                                    //  �޸ıȽ�ֵ���޸�ռ�ձ�
}

/**********************************************************************************************************
�������ƣ�ֱ��������Ժ���
���������ģʽ��0Ϊֹͣ��1Ϊ��ת��2Ϊ��ת
�����������
**********************************************************************************************************/
void DCMotorTest(unsigned char mode)
{
    if (mode == 0)
    {
        DCMOTOR_A_L;
        DCMOTOR_B_L;
    }
    else if (mode == 1)
    {
        DCMOTOR_A_L;
        DCMOTOR_B_H;
    }
    else
    {
        DCMOTOR_A_H;
        DCMOTOR_B_L;
    }
}
