/**
  ******************************************************************************
  * @file    SysTick/SysTick_Example/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "../BaseDrive/USART.h"
#include "string.h"
#include "../BaseDrive/mlx90640/mlx90640.h"
#include "delay.h"
#include "LED.h"
#include "../lvgl/src/hal/lv_hal_tick.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup SysTick_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    
    lv_tick_inc(1);
    
    static unsigned int cnt = 0;
    
    cnt++;
    
    if(cnt == 500)
    {
        cnt = 0;
        
        LED4_REVERSE;
    }
    if(0 != Uart5.Time)
    {
      Uart5.Time--;
      if(Uart5.Time == 0)
      {
        Uart5.ReceiveFinish = 1;
      }
    }
    TimingDelay_Decrement();
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        static unsigned int cnt = 0;
        
        cnt++;
        
        if(cnt == 500)
        {
            cnt = 0;
            
            LED1_REVERSE;
        }
        // lv_tick_inc(1);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {        
        EXTI_ClearITPendingBit(EXTI_Line5);
        
        //DO SOMETHING
    }
}
unsigned int test;
void UART5_IRQHandler(void)
{
  
  if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)                //  若接收数据寄存器满
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    
    // if(Uart5.RXlenth < 300){
    //   Uart5.Rxbuf[Uart5.RXlenth++] = USART_ReceiveData(UART5);
    // }
    // Uart5.Time = 3;

    Uart5.Rxbuf[Uart5.RXlenth] = USART_ReceiveData(UART5);
    
    // USART_SendData(USART3, Uart5.Rxbuf[Uart5.RXlenth]);
    switch (Uart5.RXlenth)
    {
      case 0:
        if(Uart5.Rxbuf[0] != 0x5a)
        {
          Uart5.RXlenth = 0;
        }else{
          Uart5.RXlenth++;
        }
        break;
      case 1:
        if(Uart5.Rxbuf[1] != 0x5a)
        {
          Uart5.RXlenth = 0;
        }else{
          Uart5.RXlenth++;
        }
        break;
      case 2:
        if(Uart5.Rxbuf[2] != 0x02)
        {
          Uart5.RXlenth = 0;
        }else{
          Uart5.RXlenth++;
        }
        break;
      case 3:
        if(Uart5.Rxbuf[3] != 0x06)
        {
          Uart5.RXlenth = 0;
        }else{
          Uart5.RXlenth++;
        }
        break;
      default:
        Uart5.RXlenth++;
        if(Uart5.RXlenth == 1544)
        {

          // memcopy(mlx90640_buf, Uart5.Rxbuf, 1544);
          if(is_update){
            for(int i = 0; i < 1544; i++)
            {
              mlx90640_buf[i] = Uart5.Rxbuf[i];
            }
            // update_buffer(&mlx90640_buf[4]); // 更新缓冲区
            if(disp_finish_flag){ // 显示完成
              update_buffer(&mlx90640_buf[4]); // 更新缓冲区
              // swap_buffers(lv_scr_act()); // 交换缓冲区
              disp_finish_flag = 0;
            }
            // swap_buffers(lv_scr_act()); // 交换缓冲区
          }
          
          Uart5.ReceiveFinish = 1;
          state = 1;
          test < Uart5.RXlenth ? test = Uart5.RXlenth : test;
          Uart5.RXlenth = 0;
        }
        break;
    }
  }

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
