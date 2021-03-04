/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"
#include "gpio.h"
#include "ControlRun.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// TIM_HandleTypeDef htim1;
// TIM_HandleTypeDef htim2;
// TIM_HandleTypeDef htim3;
// TIM_HandleTypeDef htim4;
// TIM_HandleTypeDef htim5;
// TIM_HandleTypeDef htim7;
// TIM_HandleTypeDef htim8;

#define DEAD_TIME_FOR_TIM   172   // 214:2us   172: 1us

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  //   TIM_MasterConfigTypeDef sMasterConfig = {0};
  //   TIM_OC_InitTypeDef sConfigOC = {0};
  //   TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  //   htim1.Instance = TIM1;
  //   htim1.Init.Prescaler = 0;
  //   htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  //   htim1.Init.Period = PWM_PERIOD_VALUE;
  //   htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //   htim1.Init.RepetitionCounter = 0;
  //   htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  //   if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   sConfigOC.OCMode = TIM_OCMODE_PWM1;
  //   sConfigOC.Pulse = PWM_PERIOD_VALUE/2;
  //   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  //   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  //   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  //   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  //   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  //   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  //   sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  //   sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  //   sBreakDeadTimeConfig.DeadTime = DEAD_TIME_FOR_TIM;  // 172:1us   214: 2us
  //   sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  //   sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  //   sBreakDeadTimeConfig.BreakFilter = 0;
  //   sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  //   sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  //   sBreakDeadTimeConfig.Break2Filter = 0;
  //   sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  //   if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
	
	// sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  //   sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  //   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  //   if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
	
  //   HAL_TIM_MspPostInit(&htim1);
	
	// HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
    
	// HAL_TIM_Base_Start_IT(&htim1);    //  start timer interrupt 

	// TIM1->RCR = 1;   // for generate underflow update event
	
	// TIM1->BDTR |= (1<<15);
	
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{
    // TIM_Encoder_InitTypeDef sConfig = {0};

    // htim2.Instance = TIM2;
    // htim2.Init.Prescaler = 0;
    // htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    // htim2.Init.Period = 0xFFFF;
    // htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }

    // sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    // sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    // sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    // sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    // sConfig.IC1Filter = 9;
    // sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    // sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    // sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    // sConfig.IC2Filter = 9;
    // if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }

    // /* Enable the Peripheral */
    // __HAL_TIM_ENABLE(&htim2); 

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
    // TIM_Encoder_InitTypeDef sConfig = {0};

    // htim3.Instance = TIM3;
    // htim3.Init.Prescaler = 0;
    // htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    // htim3.Init.Period = 0xFFFF;
    // htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }

    // sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    // sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    // sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    // sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    // sConfig.IC1Filter = 9;
    // sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    // sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    // sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    // sConfig.IC2Filter = 9;
    // if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }

    // /* Enable the Peripheral */
    // __HAL_TIM_ENABLE(&htim3); 

}
/* TIM4 init function */
void MX_TIM4_Init(void)
{
    // TIM_Encoder_InitTypeDef sConfig = {0};

    // htim4.Instance = TIM4;
    // htim4.Init.Prescaler = 0;
    // htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    // htim4.Init.Period = 0xFFFF;
    // htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }

    // sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    // sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    // sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    // sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    // sConfig.IC1Filter = 9;
    // sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    // sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    // sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    // sConfig.IC2Filter = 9;
    // if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }

    // /* Enable the Peripheral */
    // __HAL_TIM_ENABLE(&htim4); 

}

/* TIM5 init function */
void MX_TIM5_Init_weak(void)
{
	// htim5.Instance = TIM5;
	// htim5.Init.Prescaler = 4-1;  // // freq:27M
	// htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	// htim5.Init.Period = 0xFFFFFFFF;
	// htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
	// htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	// if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	// {
	// 	_Error_Handler(__FILE__, __LINE__);
	// }
	
	// HAL_TIM_Base_Start(&htim5); 

}



/* TIM7 init function */
void MX_TIM7_Init_weak(void)
{
    // htim7.Instance = TIM7;
    // htim7.Init.Prescaler = (1080-1);  // freq:100K
    // htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    // htim7.Init.Period = (2500-1);      // 25ms :40hz
    // htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }
  
    //   /* TIM7 interrupt Init */
    // HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);
     
    // HAL_TIM_Base_Start_IT(&htim7);

}

/***********************************************************************
 * DESCRIPTION: 用于延迟计时  在GD中timer4 对应st的timer5 计数器宽度32位
 *
 * RETURNS:
 *
***********************************************************************/
void MX_TIM5_Init(void)
{
	timer_parameter_struct timer_initpara;
    
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    
    rcu_periph_clock_enable(RCU_TIMER4);
    
    timer_deinit(TIMER4);

    timer_initpara.prescaler = (8 - 1);    // 25mhz
    timer_initpara.period = 0xFFFFFFFF; 
    timer_initpara.counterdirection = TIMER_COUNTER_CENTER_UP;
    timer_initpara.repetitioncounter = 0;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER4,&timer_initpara);
    
    timer_enable(TIMER4);

}

/***********************************************************************
 * DESCRIPTION: 40HZ timer中断  在GD中timer8 对应st的timer7
 *
 * RETURNS:
 *
***********************************************************************/
void MX_TIM7_Init(void)
{
    timer_parameter_struct timer_initpara;
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    
    rcu_periph_clock_enable(RCU_TIMER8);
    
    nvic_irq_enable(TIMER0_BRK_TIMER8_IRQn,0,2);
    
    timer_deinit(TIMER8);

    timer_initpara.prescaler = (2000 - 1);    // 100khz
    timer_initpara.period = (2500 - 1); 
    timer_initpara.counterdirection = TIMER_COUNTER_CENTER_UP;
    timer_initpara.repetitioncounter = 0;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    
    timer_init(TIMER8,&timer_initpara);
    
    timer_interrupt_disable(TIMER8,TIMER_INT_UP);

    timer_enable(TIMER8);
    
}

/* TIM8 init function */
void MX_TIM8_Init(void)
{
  //   TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  //   TIM_OC_InitTypeDef sConfigOC = {0};
  //   TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  //   htim8.Instance = TIM8;
  //   htim8.Init.Prescaler = 0;
  //   htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  //   htim8.Init.Period = PWM_PERIOD_VALUE;
  //   htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //   htim8.Init.RepetitionCounter = 0;
  //   htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  //   if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   sConfigOC.OCMode = TIM_OCMODE_PWM1;
  //   sConfigOC.Pulse = PWM_PERIOD_VALUE/2;
  //   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  //   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  //   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  //   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  //   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  //   if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }

  //   sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  //   sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  //   sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  //   sBreakDeadTimeConfig.DeadTime = DEAD_TIME_FOR_TIM;
  //   sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  //   sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  //   sBreakDeadTimeConfig.BreakFilter = 0;
  //   sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  //   sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  //   sBreakDeadTimeConfig.Break2Filter = 0;
  //   sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  //   if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  //   {
  //       _Error_Handler(__FILE__, __LINE__);
  //   }
	
  //   /* Configure TIM8 as slave & use the update event as Trigger Input (TRGO) */
	// sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	// sSlaveConfig.InputTrigger     = TIM_TS_ITR0;
	// sSlaveConfig.TriggerPolarity  = TIM_TRIGGERPOLARITY_BOTHEDGE;
	// sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	// sSlaveConfig.TriggerFilter    = 0;

	// if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)
	// {
	// 	_Error_Handler(__FILE__, __LINE__);
	// }

  //   HAL_TIM_MspPostInit(&htim8);
	
	// TIM8->RCR = 1;   // for generate underflow update event
	
	// TIM8->BDTR |= (1<<15);
  
}

// void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
// {

//     if(tim_pwmHandle->Instance==TIM1)
//     {
//         __HAL_RCC_TIM1_CLK_ENABLE();
//     }
//     else if(tim_pwmHandle->Instance==TIM8)
//     {
//         __HAL_RCC_TIM8_CLK_ENABLE();
//     }
// }

// void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
// {

//   GPIO_InitTypeDef GPIO_InitStruct;
//   if(tim_icHandle->Instance==TIM2)
//   {
//     /* TIM2 clock enable */
//     __HAL_RCC_TIM2_CLK_ENABLE();
  
//     /**TIM2 GPIO Configuration    
//     PA0-WKUP     ------> TIM2_CH1
//     PA1     ------> TIM2_CH2
//     PA2     ------> TIM2_CH3 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   }
//   else if(tim_icHandle->Instance==TIM3)
//   {
//     __HAL_RCC_TIM3_CLK_ENABLE();
  
//     /**TIM3 GPIO Configuration    
//     PC9     ------> TIM3_CH4
//     PB4     ------> TIM3_CH1
//     PB5     ------> TIM3_CH2 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_9;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//   }
//   else if(tim_icHandle->Instance==TIM4)
//   {
//     __HAL_RCC_TIM4_CLK_ENABLE();
      
//     /**TIM4 GPIO Configuration    
//     PD12     ------> TIM4_CH1
//     PD13     ------> TIM4_CH2 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//     HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//   }

// }

// void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
// {

// //  GPIO_InitTypeDef GPIO_InitStruct;
//   if(htim_base->Instance==TIM5)
//   {
//     __HAL_RCC_TIM5_CLK_ENABLE();
  
//   }
//   else if(htim_base->Instance==TIM7)
//   {
//     /* TIM7 clock enable */
//     __HAL_RCC_TIM7_CLK_ENABLE();
//   }

// }

// void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
// {

//   GPIO_InitTypeDef GPIO_InitStruct;
//   if(timHandle->Instance==TIM1)
//   {
//     /**TIM1 GPIO Configuration    
//     PE8     ------> TIM1_CH1N
//     PE9     ------> TIM1_CH1
//     PE10     ------> TIM1_CH2N
//     PE11     ------> TIM1_CH2
//     PE12     ------> TIM1_CH3N
//     PE13     ------> TIM1_CH3 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
//                           |GPIO_PIN_12|GPIO_PIN_13;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
//     HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//   }
//   else if(timHandle->Instance==TIM8)
//   { 
//     /**TIM8 GPIO Configuration    
//     PA5     ------> TIM8_CH1N
//     PB0     ------> TIM8_CH2N
//     PB1     ------> TIM8_CH3N
//     PC6     ------> TIM8_CH1
//     PC7     ------> TIM8_CH2
//     PC8     ------> TIM8_CH3 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_5;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//   }

// }

// void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
// {

//   if(tim_pwmHandle->Instance==TIM1)
//   {
//     __HAL_RCC_TIM1_CLK_DISABLE();
//   }
//   else if(tim_pwmHandle->Instance==TIM4)
//   {
//     __HAL_RCC_TIM4_CLK_DISABLE();
//   }
//   else if(tim_pwmHandle->Instance==TIM8)
//   {
//     __HAL_RCC_TIM8_CLK_DISABLE();
//   }
// }

// void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
// {

//   if(tim_icHandle->Instance==TIM2)
//   {
//     __HAL_RCC_TIM2_CLK_DISABLE();
  
//     /**TIM2 GPIO Configuration    
//     PA0-WKUP     ------> TIM2_CH1
//     PA1     ------> TIM2_CH2
//     PA2     ------> TIM2_CH3 
//     */
//     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
//   }
//   else if(tim_icHandle->Instance==TIM3)
//   {
//     __HAL_RCC_TIM3_CLK_DISABLE();
		
//     /**TIM3 GPIO Configuration    
//     PC9     ------> TIM3_CH4
//     PB4     ------> TIM3_CH1
//     PB5     ------> TIM3_CH2 
//     */
//     HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
//     HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4|GPIO_PIN_5);
//   }
// } 

// void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
// {

//   if(htim_base->Instance==TIM5)
//   {
//     __HAL_RCC_TIM5_CLK_DISABLE();
  
//   }
//   else if(htim_base->Instance==TIM7)
//   {
//     /* Peripheral clock disable */
//     __HAL_RCC_TIM7_CLK_DISABLE();

//     /* TIM7 interrupt Deinit */
//     HAL_NVIC_DisableIRQ(TIM7_IRQn);
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
