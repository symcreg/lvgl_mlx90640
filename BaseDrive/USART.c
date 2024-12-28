#include "stm32f4xx.h"
#include "USART.h"
#include "stdio.h"

unsigned char Uart3ReceiveBuf[300] = {0};
unsigned char Uart5ReceiveBuf[1544] = {0};

//  结构体定义
USARTDATA   Uart3;
USARTDATA   Uart6;
USARTDATA   Uart5;
/**********************************************************************************************************
函数名称：UART3配置
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
// USART3_TX	 PB10	//  out
// USART3_RX	 PB11	//  in
void UART3_Configuration(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
	USART_InitTypeDef   USART_InitStructure;

    //Uart3初始化 
    Uart3.ReceiveFinish = 0;// 将ReceiveFinish标志位设置为0，表示未接收完成
    Uart3.RXlenth = 0;// 将接收长度设置为0
    Uart3.Time = 0;// 将时间计数器设置为0
    Uart3.Rxbuf = Uart3ReceiveBuf;// 将接收缓冲区指向ReceiveBuffer



	//  开启GPIO_D的时钟 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
	//  开启串口3的时钟 
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

	/* 使能串口3 */
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/**********************************************************************************************************
函数名称：putchar函数重定义
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
int fputc(int ch, FILE *f)
{
    USART3->SR;                                                         // 防止复位后无法打印首字符
    
    USART_SendData(USART3, (u8) ch);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
        ; 
    }
    
    return (ch);
}
// int fputc(int ch, FILE *f)
// {
//     UART5->SR;// 读取状态寄存器，防止复位后无法打印首字符                                                         // 防止复位后无法打印首字符
    
//     USART_SendData(UART5, (u8) ch); // 通过USART3发送一个字符
//     while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET)// 等待发送完成
//     {
//         ; 
//     }
    
//     return (ch); // 返回发送的字符
// }


/**********************************************************************************************************
函数名称：USART3发送数据函数
输入参数：发送数据首地址和数据长度
输出参数：无
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
函数名称：UART6配置
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
// USART6_TX	 PC6	//  out
// USART6_RX	 PC7	//  in
void UART6_Configuration(unsigned int baud)
{
	//  GPIO初始化定义
	//  USART初始化定义
	//  NVIC初始化定义
    
    // Uart6.ReceiveFinish = 0;
    // Uart6.RXlenth = 0;
    // Uart6.Time = 0;
    // Uart6.Rxbuf = Uart6ReceiveBuf;

	// //  开启GPIOA的时钟 
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
	//  开启串口6的时钟 
	
	/*  时钟使能
			
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
			
			GPIO初始化（引脚6、7）
			
			GPIO引脚设置

			USART初始化
	USART_InitStructure.USART_BaudRate   = baud;
	

	USART_Init(USART6, &USART_InitStructure);

	/* 使能串口6 */
	

    /* NVIC configuration */
    /* Configure the Priority Group to 2 bits */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USARTx Interrupt */
    

    /* Enable USART */
    
}

/**********************************************************************************************************
函数名称：USART6发送数据函数
输入参数：发送数据首地址和数据长度
输出参数：无
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
函数名称：UART1	配置
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
// USART5_TX	 PC12	//  out
// USART5_RX	 PD2	//  in

void UART5_Configuration(unsigned int baud)
{

    Uart5.ReceiveFinish = 0;
    Uart5.RXlenth = 0;
    Uart5.Time = 0;
    Uart5.Rxbuf = Uart5ReceiveBuf;

    // 1. 启用 UART5 和 GPIO 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE); // GPIOC 和 GPIOD 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); // UART5 时钟

    // 2. 配置 UART5 TX (PC12) 和 RX (PD2) 引脚
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 配置 PC12 为复用功能 (TX)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 配置 PD2 为复用功能 (RX)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 将 PC12 和 PD2 设置为 UART5 的复用功能
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); // PC12 -> UART5_TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);  // PD2 -> UART5_RX

    // 3. 配置 UART5
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = baud; // 波特率
    USART_InitStruct.USART_WordLength = USART_WordLength_8b; // 8 位数据
    USART_InitStruct.USART_StopBits = USART_StopBits_1; // 1 位停止位
    USART_InitStruct.USART_Parity = USART_Parity_No; // 无校验
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 使能接收和发送
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控

    USART_Init(UART5, &USART_InitStruct);

    // 4. 启用 UART5
    USART_Cmd(UART5, ENABLE);

    // 启用接收中断
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 配置NVIC的优先级分组为2位
    
    // 5. 配置中断（如果需要）
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn; // UART5 中断通道
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; // 优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 6. 启用 UART5
    USART_Cmd(UART5, ENABLE);
}

/**********************************************************************************************************
函数名称：USART6发送数据函数
输入参数：发送数据首地址和数据长度
输出参数：无
**********************************************************************************************************/
void UART5_Senddata(unsigned char *Data, unsigned int length)
{
    while(length--)
    {
        USART_SendData(UART5,*Data++);
        while (USART_GetFlagStatus(UART5, USART_FLAG_TC)==RESET);
    }
}

