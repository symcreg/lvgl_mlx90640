#include "stm32f4xx.h"
#include "USART.h"
#include "stdio.h"

unsigned char Uart3ReceiveBuf[300] = {0};
unsigned char Uart5ReceiveBuf[1544] = {0};

//  �ṹ�嶨��
USARTDATA   Uart3;
USARTDATA   Uart6;
USARTDATA   Uart5;
/**********************************************************************************************************
�������ƣ�UART3����
�����������
�����������
�������أ���
**********************************************************************************************************/
// USART3_TX	 PB10	//  out
// USART3_RX	 PB11	//  in
void UART3_Configuration(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
	USART_InitTypeDef   USART_InitStructure;

    //Uart3��ʼ�� 
    Uart3.ReceiveFinish = 0;// ��ReceiveFinish��־λ����Ϊ0����ʾδ�������
    Uart3.RXlenth = 0;// �����ճ�������Ϊ0
    Uart3.Time = 0;// ��ʱ�����������Ϊ0
    Uart3.Rxbuf = Uart3ReceiveBuf;// �����ջ�����ָ��ReceiveBuffer



	//  ����GPIO_D��ʱ�� 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
	//  ��������3��ʱ�� 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);


	USART_InitStructure.USART_BaudRate   = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;
	USART_InitStructure.USART_Parity     = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART3, &USART_InitStructure);

	/* ʹ�ܴ���3 */
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/**********************************************************************************************************
�������ƣ�putchar�����ض���
�����������
�����������
�������أ���
**********************************************************************************************************/
int fputc(int ch, FILE *f)
{
    USART3->SR;                                                         // ��ֹ��λ���޷���ӡ���ַ�
    
    USART_SendData(USART3, (u8) ch);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
        ; 
    }
    
    return (ch);
}
// int fputc(int ch, FILE *f)
// {
//     UART5->SR;// ��ȡ״̬�Ĵ�������ֹ��λ���޷���ӡ���ַ�                                                         // ��ֹ��λ���޷���ӡ���ַ�
    
//     USART_SendData(UART5, (u8) ch); // ͨ��USART3����һ���ַ�
//     while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET)// �ȴ��������
//     {
//         ; 
//     }
    
//     return (ch); // ���ط��͵��ַ�
// }


/**********************************************************************************************************
�������ƣ�USART3�������ݺ���
������������������׵�ַ�����ݳ���
�����������
**********************************************************************************************************/
void USART3_Senddata(unsigned char *Data, unsigned int length)
{
    while(length--)
    {
        USART_SendData(USART3,*Data++);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
    }
}

/**********************************************************************************************************
�������ƣ�UART6����
�����������
�����������
�������أ���
**********************************************************************************************************/
// USART6_TX	 PC6	//  out
// USART6_RX	 PC7	//  in
void UART6_Configuration(unsigned int baud)
{
	//  GPIO��ʼ������
	//  USART��ʼ������
	//  NVIC��ʼ������
    
    // Uart6.ReceiveFinish = 0;
    // Uart6.RXlenth = 0;
    // Uart6.Time = 0;
    // Uart6.Rxbuf = Uart6ReceiveBuf;

	// //  ����GPIOA��ʱ�� 
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
	//  ��������6��ʱ�� 
	
	/*  ʱ��ʹ��
			
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
			
			GPIO��ʼ��������6��7��
			
			GPIO��������

			USART��ʼ��
	USART_InitStructure.USART_BaudRate   = baud;
	

	USART_Init(USART6, &USART_InitStructure);

	/* ʹ�ܴ���6 */
	

    /* NVIC configuration */
    /* Configure the Priority Group to 2 bits */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USARTx Interrupt */
    

    /* Enable USART */
    
}

/**********************************************************************************************************
�������ƣ�USART6�������ݺ���
������������������׵�ַ�����ݳ���
�����������
**********************************************************************************************************/
void USART6_Senddata(unsigned char *Data, unsigned int length)
{
    while(length--)
    {
        USART_SendData(USART6,*Data++);
        while (USART_GetFlagStatus(USART6, USART_FLAG_TC)==RESET);
    }
}

/**********************************************************************************************************
�������ƣ�UART1	����
�����������
�����������
�������أ���
**********************************************************************************************************/
// USART5_TX	 PC12	//  out
// USART5_RX	 PD2	//  in

void UART5_Configuration(unsigned int baud)
{

    Uart5.ReceiveFinish = 0;
    Uart5.RXlenth = 0;
    Uart5.Time = 0;
    Uart5.Rxbuf = Uart5ReceiveBuf;

    // 1. ���� UART5 �� GPIO ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE); // GPIOC �� GPIOD ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); // UART5 ʱ��

    // 2. ���� UART5 TX (PC12) �� RX (PD2) ����
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // ���� PC12 Ϊ���ù��� (TX)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // ���� PD2 Ϊ���ù��� (RX)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    // �� PC12 �� PD2 ����Ϊ UART5 �ĸ��ù���
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); // PC12 -> UART5_TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);  // PD2 -> UART5_RX

    // 3. ���� UART5
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = baud; // ������
    USART_InitStruct.USART_WordLength = USART_WordLength_8b; // 8 λ����
    USART_InitStruct.USART_StopBits = USART_StopBits_1; // 1 λֹͣλ
    USART_InitStruct.USART_Parity = USART_Parity_No; // ��У��
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ʹ�ܽ��պͷ���
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������

    USART_Init(UART5, &USART_InitStruct);

    // 4. ���� UART5
    USART_Cmd(UART5, ENABLE);

    // ���ý����ж�
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// ����NVIC�����ȼ�����Ϊ2λ
    
    // 5. �����жϣ������Ҫ��
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn; // UART5 �ж�ͨ��
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; // ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 6. ���� UART5
    USART_Cmd(UART5, ENABLE);
}

/**********************************************************************************************************
�������ƣ�USART6�������ݺ���
������������������׵�ַ�����ݳ���
�����������
**********************************************************************************************************/
void UART5_Senddata(unsigned char *Data, unsigned int length)
{
    while(length--)
    {
        USART_SendData(UART5,*Data++);
        while (USART_GetFlagStatus(UART5, USART_FLAG_TC)==RESET);
    }
}

