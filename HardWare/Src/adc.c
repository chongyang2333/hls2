/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// ADC_HandleTypeDef hadc1;
// ADC_HandleTypeDef hadc2;
// ADC_HandleTypeDef hadc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
//  ADC_ChannelConfTypeDef sConfig;
  //   ADC_InjectionConfTypeDef sConfigInjected;

  //   /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  //   */
  //   hadc1.Instance = ADC1;
  //   hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  //   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  //   hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  //   hadc1.Init.ContinuousConvMode = DISABLE;
  //   hadc1.Init.DiscontinuousConvMode = DISABLE;
  //   hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  //   hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  //   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  //   hadc1.Init.NbrOfConversion = 0;
  //   hadc1.Init.DMAContinuousRequests = DISABLE;
  //   hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  //   if (HAL_ADC_Init(&hadc1) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  //   sConfigInjected.InjectedNbrOfConversion = 4;
  //   sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  //   sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  //   sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  //   sConfigInjected.AutoInjectedConv = DISABLE;
  //   sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  //   sConfigInjected.InjectedOffset = 0;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
	// */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

	// HAL_ADC_Start(&hadc1);
	
}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  //   ADC_InjectionConfTypeDef sConfigInjected;

  //   /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  //   */
  //   hadc2.Instance = ADC2;
  //   hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  //   hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  //   hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  //   hadc2.Init.ContinuousConvMode = DISABLE;
  //   hadc2.Init.DiscontinuousConvMode = DISABLE;
  //   hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  //   hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  //   hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  //   hadc2.Init.NbrOfConversion = 0;
  //   hadc2.Init.DMAContinuousRequests = DISABLE;
  //   hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  //   if (HAL_ADC_Init(&hadc2) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

	
  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  //   sConfigInjected.InjectedNbrOfConversion = 4;
  //   sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  //   sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  //   sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  //   sConfigInjected.AutoInjectedConv = DISABLE;
  //   sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  //   sConfigInjected.InjectedOffset = 0;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  //   {
  //   _Error_Handler(__FILE__, __LINE__);
  //   }

  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
	// HAL_ADC_Start(&hadc2);

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{

  //   ADC_InjectionConfTypeDef sConfigInjected;

  //   /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  //   */
  //   hadc3.Instance = ADC3;
  //   hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  //   hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  //   hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  //   hadc3.Init.ContinuousConvMode = DISABLE;
  //   hadc3.Init.DiscontinuousConvMode = DISABLE;
  //   hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  //   hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  //   hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  //   hadc3.Init.NbrOfConversion = 0;
  //   hadc3.Init.DMAContinuousRequests = DISABLE;
  //   hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  //   if (HAL_ADC_Init(&hadc3) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  //   */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  //   sConfigInjected.InjectedNbrOfConversion = 4;
  //   sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  //   sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  //   sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  //   sConfigInjected.AutoInjectedConv = DISABLE;
  //   sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  //   sConfigInjected.InjectedOffset = 0;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  //   {
  //   _Error_Handler(__FILE__, __LINE__);
  //   }

	// /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
	// */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
	// /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
	// */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
	
  //   /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
	// */
  //   sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  //   sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  //   if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
    
	// HAL_ADC_Start(&hadc3);
//	ADC3->CR2 |= (1<<0);

}

// void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
// {

//   GPIO_InitTypeDef GPIO_InitStruct;
//   if(adcHandle->Instance==ADC1)
//   {
//   /* USER CODE BEGIN ADC1_MspInit 0 */

//   /* USER CODE END ADC1_MspInit 0 */
//     /* ADC1 clock enable */
//     __HAL_RCC_ADC1_CLK_ENABLE();
  
//     /**ADC1 GPIO Configuration    
//     PA4     ------> ADC1_IN4
//     PC3     ------> ADC1_IN13 
//     PC5     ------> ADC1_IN15 
//     PA2     ------> ADC1_IN2 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
//     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      
//     GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
//     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//   /* USER CODE BEGIN ADC1_MspInit 1 */

//   /* USER CODE END ADC1_MspInit 1 */
//   }
//   else if(adcHandle->Instance==ADC2)
//   {
//   /* USER CODE BEGIN ADC2_MspInit 0 */

//   /* USER CODE END ADC2_MspInit 0 */
//     /* ADC2 clock enable */
//     __HAL_RCC_ADC2_CLK_ENABLE();
  
//     /**ADC2 GPIO Configuration    
//     PC4     ------> ADC2_IN14
//     PA6     ------> ADC2_IN6
//     PA7     ------> ADC2_IN7 
//     PA3     ------> ADC2_IN3
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_4;
//     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
      
//     GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
//     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* USER CODE BEGIN ADC2_MspInit 1 */

//   /* USER CODE END ADC2_MspInit 1 */
//   }
//   else if(adcHandle->Instance==ADC3)
//   {
//   /* USER CODE BEGIN ADC3_MspInit 0 */

//   /* USER CODE END ADC3_MspInit 0 */
//     /* ADC3 clock enable */
//     __HAL_RCC_ADC3_CLK_ENABLE();
  
//     /**ADC3 GPIO Configuration    
//     PC0     ------> ADC3_IN10
//     PC1     ------> ADC3_IN11 
//     PC2     ------> ADC3_IN12   
//     PF3     ------> ADC3_IN9
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
//     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
      
//     GPIO_InitStruct.Pin = GPIO_PIN_3;
//     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 


//   /* USER CODE BEGIN ADC3_MspInit 1 */

//   /* USER CODE END ADC3_MspInit 1 */
//   }
// }

// void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
// {

//   if(adcHandle->Instance==ADC1)
//   {

//     __HAL_RCC_ADC1_CLK_DISABLE();

//     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
//     HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3|GPIO_PIN_5);

//   }
//   else if(adcHandle->Instance==ADC2)
//   {

//     __HAL_RCC_ADC2_CLK_DISABLE();
  
//     HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
//     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7);

//   }
//   else if(adcHandle->Instance==ADC3)
//   {

//     __HAL_RCC_ADC3_CLK_DISABLE();
  
//     /**ADC3 GPIO Configuration    
//     PC0     ------> ADC3_IN10
//     PC1     ------> ADC3_IN11 
//     PC2     ------> ADC3_IN12
//     */
//     HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

//   }
// } 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
