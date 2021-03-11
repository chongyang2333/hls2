/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#include "gd32f4xx.h"
#include "can.h"
#include "gpio.h"
#include "CanApp.h"
#include "delay.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//CAN_HandleTypeDef hcan1;


/**
 * \brief      initialize CAN and filter
 * \prarm[in]  can_periph
 * \param[out] none
 * \retval     none
*/
static void can_networking_init(uint32_t can_periph)
{
    can_parameter_struct            can_parameter;
    can_filter_parameter_struct     can_filter;
    
    /* reset CAN */
    can_deinit(can_periph);

    /* initialize CAN */
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_parameter.time_triggered        = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up          = DISABLE;
    can_parameter.no_auto_retrans       = DISABLE;
    can_parameter.rec_fifo_overwrite    = DISABLE;
    can_parameter.trans_fifo_order      = ENABLE;
    can_parameter.working_mode          = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width     = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1        = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2        = CAN_BT_BS2_4TQ;
    /* baudrate 1Mbps */
    can_parameter.prescaler = 5;
    can_init(can_periph, &can_parameter);

    /* initialize filter */
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    if (can_periph == CAN0)
    {
        can_filter.filter_number = 0;
    }
    else
    {
        can_filter.filter_number = 15;
    }
    can_filter.filter_mode        = CAN_FILTERMODE_MASK;
    can_filter.filter_bits        = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high   = 0xFFFF;
    can_filter.filter_list_low    = 0xFFFF;
    can_filter.filter_mask_high   = 0x0000;
    can_filter.filter_mask_low    = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable      = ENABLE;
    can_filter_init(&can_filter);
}

/**
 * \brief      initialize CAN gpio pin
 * \prarm[in]  can_periph
 * \param[out] none
 * \retval     none
*/
static void can_gpio_init(uint32_t can_periph)
{
    if (can_periph == CAN0)
    {
        #define __CAN0_Rx_PORT  GPIOD
        #define __CAN0_Rx_GPIO  GPIO_PIN_0
        #define __CAN0_Rx_AF    GPIO_AF_9
        #define __CAN0_Tx_PORT  GPIOD
        #define __CAN0_Tx_GPIO  GPIO_PIN_1
        #define __CAN0_Tx_AF    GPIO_AF_9

        /* enable can clock */
        rcu_periph_clock_enable(RCU_CAN0);
        rcu_periph_clock_enable(RCU_GPIOD);

        /* configure CAN0 GPIO */
        gpio_output_options_set(__CAN0_Rx_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, __CAN0_Rx_GPIO);
        gpio_mode_set(__CAN0_Rx_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, __CAN0_Rx_GPIO);
        gpio_af_set(__CAN0_Rx_PORT, __CAN0_Rx_AF, __CAN0_Rx_GPIO);
        
        gpio_output_options_set(__CAN0_Tx_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, __CAN0_Tx_GPIO);
        gpio_mode_set(__CAN0_Tx_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, __CAN0_Tx_GPIO);
        gpio_af_set(__CAN0_Tx_PORT, __CAN0_Tx_AF, __CAN0_Tx_GPIO);
    }
}

/**
 * \brief      this function handles CAN0 Rx0 ecxeption
 * \prarm[in]  none
 * \param[out] none
 * \retval     none
*/
void CAN0_RX1_IRQHandler(void)
{
    CanAppDispatch();
}

/* CAN1 init function */

void MX_CAN1_Init(void)
{
    /* stm32 can1 <==> gd32 can0 */

    /* configure CAN gpio */
    can_gpio_init(CAN0);

    /* initialize CAN network */
    can_networking_init(CAN0);

    /* enable CAN receive FIFO1 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INT_RFNE1);

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX1_IRQn, 1, 1);

    delay_ms(10);
    JumpAppFb(GetResetType());
    delay_ms(10);
}


void MX_CAN1_Init_Weak(void)
{
//     CAN_FilterTypeDef  sFilterConfig;
 
    
//     hcan1.Instance = CAN1;
// //    hcan1.Init.Prescaler = 18;   // 54M/18 =3M
// //    hcan1.Init.Mode = CAN_MODE_NORMAL;
// //    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;  // baud rate = 3M/(SJW+BS1+BS2)
// //    hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;       // baud rate = 3M/(1+3+2) = 500K
// //    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;       // (SJW+BS1)/(SJW+BS1+BS2) =66.7%
//     hcan1.Init.Prescaler = 3;    // 54M/3 = 18M
//     hcan1.Init.Mode = CAN_MODE_NORMAL;
//     hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
//     hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;       // baud rate = 18M/(SJW+BS1+BS2) = 18M/18 = 1000K
//     hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;       // (SJW+BS1)/(SJW+BS1+BS2) = (15)/(18) =83.3%
 
//     /* for canopen */    
// //    hcan1.Instance = CAN1;
// //    hcan1.Init.Prescaler = 6;   // 54M/6 =9M
// //    hcan1.Init.Mode = CAN_MODE_NORMAL;
// //    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;  // baud rate = 9M/(SJW+BS1+BS2)
// //    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;       // baud rate = 9M/(1+6+2)
// //    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;       
//     hcan1.Init.TimeTriggeredMode = DISABLE;
//     hcan1.Init.AutoBusOff = ENABLE;
//     hcan1.Init.AutoWakeUp = DISABLE;
//     hcan1.Init.AutoRetransmission = ENABLE;
//     hcan1.Init.ReceiveFifoLocked = DISABLE;
//     hcan1.Init.TransmitFifoPriority = ENABLE;
//     if (HAL_CAN_Init(&hcan1) != HAL_OK)
//     {
//         _Error_Handler(__FILE__, __LINE__);
//     }
    
//       /*##-2- Configure the CAN Filter ###########################################*/
//     sFilterConfig.FilterBank = 1;
//     sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//     sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//     sFilterConfig.FilterIdHigh = 0xFFFF;
//     sFilterConfig.FilterIdLow = 0xFFFF;
//     sFilterConfig.FilterMaskIdHigh = 0x0000;
//     sFilterConfig.FilterMaskIdLow = 0x0000;
//     sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//     sFilterConfig.FilterActivation = ENABLE;
//     sFilterConfig.SlaveStartFilterBank = 14;
  
//     if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
//     {
//         /* Filter configuration Error */
//         _Error_Handler(__FILE__, __LINE__);
//     }
 
    
//     /* CAN1 interrupt Init */
//     HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 3, 0);
//     HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    
//     __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    
//     /*Start the CAN peripheral */
//     if (HAL_CAN_Start(&hcan1) != HAL_OK)
//     {
//         /* Start Error */
//         _Error_Handler(__FILE__, __LINE__);
//     }  

    delay_ms(10);
    JumpAppFb(GetResetType());
    delay_ms(10);
}

// void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
// {

//     GPIO_InitTypeDef GPIO_InitStruct;
//     if(canHandle->Instance==CAN1)
//     {
//         /* CAN1 clock enable */
//         __HAL_RCC_CAN1_CLK_ENABLE();

//         /**CAN1 GPIO Configuration    
//         PD0     ------> CAN1_RX
//         PD1     ------> CAN1_TX 
//         */
//         GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
//         GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//         GPIO_InitStruct.Pull = GPIO_NOPULL;
//         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//         GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
//         HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//     }
// }

// void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
// {

//   if(canHandle->Instance==CAN1)
//   {
//     /* Peripheral clock disable */
//     __HAL_RCC_CAN1_CLK_DISABLE();
  
//     /**CAN1 GPIO Configuration    
//     PD0     ------> CAN1_RX
//     PD1     ------> CAN1_TX 
//     */
//     HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

//     /* CAN1 interrupt Deinit */
//     HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
//   }
// } 



/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
