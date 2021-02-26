/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"
#include "UartApp.h"
#include "HardApi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// UART_HandleTypeDef huart1;
// UART_HandleTypeDef huart3;
// UART_HandleTypeDef huart6;
// DMA_HandleTypeDef hdma_usart3_rx;
// DMA_HandleTypeDef hdma_usart3_tx;

void MX_USART1_UART_Init(void)
{
    // huart1.Instance = USART1;
    // huart1.Init.BaudRate = 115200;
    // huart1.Init.WordLength = UART_WORDLENGTH_8B;
    // huart1.Init.StopBits = UART_STOPBITS_1;
    // huart1.Init.Parity = UART_PARITY_NONE;
    // huart1.Init.Mode = UART_MODE_TX_RX;
    // huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    // huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    // huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    // huart1.AdvancedInit.OverrunDisable = (1<<12);
    
    // if (HAL_UART_Init(&huart1) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }
  
    // /* USART1 interrupt Init */
    // __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);  
    // HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    // HAL_NVIC_EnableIRQ(USART1_IRQn);

    // __HAL_UART_DISABLE_IT(&huart1,UART_IT_ERR | UART_IT_ORE); 

}

/* USART3 init function */

void MX_USART3_UART_Init(void)
{
//     huart3.Instance = USART3;
//     huart3.Init.BaudRate = 115200;
//     huart3.Init.WordLength = UART_WORDLENGTH_8B;
//     huart3.Init.StopBits = UART_STOPBITS_1;
//     huart3.Init.Parity = UART_PARITY_NONE;
//     huart3.Init.Mode = UART_MODE_TX_RX;
//     huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//     huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//     huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
// //    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//     huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
//     huart3.AdvancedInit.OverrunDisable = (1<<12);

//     if (HAL_UART_Init(&huart3) != HAL_OK)
//     {
//         _Error_Handler(__FILE__, __LINE__);
//     }

//     /* USART3 interrupt Init */ 
//     HAL_NVIC_SetPriority(USART3_IRQn, 5, 1);
//     HAL_NVIC_EnableIRQ(USART3_IRQn);

//     __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 
	 
// //	__HAL_UART_DISABLE_IT(&huart3,UART_IT_ERR | UART_IT_ORE); 
	        
//     HAL_UART_Receive_DMA(&huart3, sUartApp.RecvBuf, UART_RECV_MAX_NUM);
    
  
}

// void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
// {

//   GPIO_InitTypeDef GPIO_InitStruct;
//   if(uartHandle->Instance==USART1)
//   {
//   /* USER CODE BEGIN USART1_MspInit 0 */

//   /* USER CODE END USART1_MspInit 0 */
//     /* USART1 clock enable */
//     __HAL_RCC_USART1_CLK_ENABLE();
  
//     /**USART1 GPIO Configuration    
//     PA9     ------> USART1_TX
//     PA10     ------> USART1_RX 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* USER CODE BEGIN USART1_MspInit 1 */

//   /* USER CODE END USART1_MspInit 1 */
//   }
//   else if(uartHandle->Instance==USART3)
//   {
//   /* USER CODE BEGIN USART3_MspInit 0 */

//   /* USER CODE END USART3_MspInit 0 */
//     /* USART3 clock enable */
//     __HAL_RCC_USART3_CLK_ENABLE();
  
//     /**USART3 GPIO Configuration    
//     PD8     ------> USART3_TX
//     PD9     ------> USART3_RX 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//     HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
      
//     /* USART3 DMA Init */
//     /* USART3_RX Init */
//     hdma_usart3_rx.Instance = DMA1_Stream1;
//     hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
//     hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//     hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
//     hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//     hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//     hdma_usart3_rx.Init.Mode = DMA_NORMAL;
//     hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
//     hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
//     {
//       _Error_Handler(__FILE__, __LINE__);
//     }

//     __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

//     /* USART3_TX Init */
//     hdma_usart3_tx.Instance = DMA1_Stream3;
//     hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
//     hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//     hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
//     hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//     hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//     hdma_usart3_tx.Init.Mode = DMA_NORMAL;
//     hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
//     hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
//     {
//       _Error_Handler(__FILE__, __LINE__);
//     }

//     __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

//   }

// }

// void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
// {

//   if(uartHandle->Instance==USART1)
//   {
//   /* USER CODE BEGIN USART1_MspDeInit 0 */

//   /* USER CODE END USART1_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_USART1_CLK_DISABLE();
  
//     /**USART1 GPIO Configuration    
//     PA9     ------> USART1_TX
//     PA10     ------> USART1_RX 
//     */
//     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

//   /* USER CODE BEGIN USART1_MspDeInit 1 */

//   /* USER CODE END USART1_MspDeInit 1 */
//   }
//   else if(uartHandle->Instance==USART3)
//   {
//   /* USER CODE BEGIN USART3_MspDeInit 0 */

//   /* USER CODE END USART3_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_USART3_CLK_DISABLE();
  
//     /**USART3 GPIO Configuration    
//     PD8     ------> USART3_TX
//     PD9     ------> USART3_RX 
//     */
//     HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

//   /* USART3 DMA DeInit */
//     HAL_DMA_DeInit(uartHandle->hdmatx);

//     /* USART3 interrupt Deinit */
//     HAL_NVIC_DisableIRQ(USART3_IRQn);
//   /* USER CODE BEGIN USART3_MspDeInit 1 */

//   /* USER CODE END USART3_MspDeInit 1 */
//   }

// } 

/***********************************************************************
 * DESCRIPTION:This function handles USART3 global interrupt.
 *             NVIC_Priority : (5, 1)
 * RETURNS:
 *
***********************************************************************/
void USART3_IRQHandler(void)    
{
    // uint32_t temp=0;
    // uint8_t *p=NULL;
    // uint16_t size=0;   
     
    // // if occur ORE NF FE PE error, then clear
    // if(huart3.Instance->ISR & 0xF)
    // {    
    //     huart3.Instance->ICR = 0xF;
    // }
    
    // HAL_UART_IRQHandler(&huart3);
        
    // if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))
    // { 
        
    //     __HAL_UART_CLEAR_IDLEFLAG(&huart3);   
    //     HAL_UART_DMAStop(&huart3);                     
    //     temp = huart3.Instance->ISR;  
    //     temp = huart3.Instance->RDR; 
    //     temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);  
        
    //     sUartApp.RecvBufNum +=  UART_RECV_MAX_NUM - temp; 
     
    //     if( (0xAA == sUartApp.RecvBuf[0]) && (sUartApp.RecvBufNum > 0) )
    //     {
    //         if(sUartApp.RecvBufNum > 3)
    //         {
    //             uint16_t FrameLen = sUartApp.RecvBuf[1] | (sUartApp.RecvBuf[2]<<8);
    //             if((FrameLen+1) <= sUartApp.RecvBufNum)
    //             {
                    
    //                 UartRecvDispatch(sUartApp.RecvBuf);  
                    
    //                 sUartApp.RecvBufNum = 0;
    //                 p = sUartApp.RecvBuf;
    //                 size = UART_RECV_MAX_NUM;                    
    //             }
    //             else
    //             {
    //                 p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
    //                 size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
    //             } // end if((FrameLen+1)..
    //         }
    //         else
    //         {
    //             p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
    //             size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
    //         } // end if(sUartApp.RecvBufNum..  
    //     }
    //     else
    //     {
    //         sUartApp.RecvBufNum = 0;
    //         p = sUartApp.RecvBuf;
    //         size = UART_RECV_MAX_NUM;
            
    //     } // end if((0xAA == sUartApp..
           
    //     HAL_UART_Receive_DMA(&huart3, p, size);
        
    // } // end if((__HAL_UART_GET_FLAG...

}
 

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void UartSendData(uint8_t * buf, uint16_t size)
{
  //   UINT32 StartTime = ReadTimeStampTimer();
	// while(huart3.gState!=HAL_UART_STATE_READY) 
  //   {
  //       if((ReadTimeStampTimer() - StartTime) > 27*5000000)  // 5s
  //       {
  //           break;
  //       }
  //   }
  //   HAL_UART_Transmit_DMA(&huart3, buf, size);
    
  //   StartTime = ReadTimeStampTimer();
  //   while (HAL_DMA_GetState(&hdma_usart3_tx) != HAL_DMA_STATE_READY)
  //   {
  //       if((ReadTimeStampTimer() - StartTime) > 27*5000000)  // 5s
  //       {
  //           break;
  //       }
  //   }

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
