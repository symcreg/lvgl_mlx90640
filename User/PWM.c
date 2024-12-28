#include "PWM.h"

/**********************************************************************************************************
�������ƣ�TIM10_PWM_Init��ʼ������
����������Զ���װֵ��ʱ��Ԥ��Ƶ��
�����������
�������أ���
TIM10_CH1    PB8
**********************************************************************************************************/
void TIM10_PWM_Init(unsigned int arr, unsigned int psc)
{		 					 
	GPIO_InitTypeDef            GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	            //  TIM10ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	            //  ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);            //  GPIOF9����Ϊ��ʱ��10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                           //  GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //  ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                //  �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //  ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //  ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);                              //  ��ʼ��PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler = psc;                          //  ��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;         //  ���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = arr;                             //  �Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);                    //  ��ʼ����ʱ��10
	
	//  ��ʼ��TIM10 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                   //  ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;       //  �Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //  �������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);                           //  ����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);                  //  ʹ��TIM10��CCR1�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM10, ENABLE);                                //  ARPEʹ�� 
	
	TIM_Cmd(TIM10, ENABLE);                                             //  ʹ��TIM10
}
