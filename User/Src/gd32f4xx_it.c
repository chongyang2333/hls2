/*!
    \file    gd32f4xx_it.c
    \brief   interrupt service routines
    
    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f4xx_it.h"
#include "main.h"
#include "delay.h"
#include "ControlRun.h"

extern struct AxisCtrlStruct sAxis[MAX_AXIS_NUM];


/*!
    \brief    this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief    this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */
   unsigned long i=0;
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
      gpio_bit_toggle(GPIOE, GPIO_PIN_0);
      for(i=0;i<200000;i++) ;
      
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/*!
    \brief    this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief    this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief    this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief    this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief    this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief    this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief    this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
//    delay_decrement();
}

void TIMER7_BRK_TIMER11_IRQHandler(void)
{
    if(timer_interrupt_flag_get(TIMER11,TIMER_INT_FLAG_UP) == SET)
    {
			  
        TimerIsrExec();
        timer_interrupt_flag_clear(TIMER11,TIMER_INT_FLAG_UP);
    }
}

void TIMER0_UP_TIMER9_IRQHandler(void)
{
    if(timer_interrupt_flag_get(TIMER0,TIMER_INT_FLAG_UP) == SET)
    {
        ControlRunExec();
        timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_UP);
    }
}

/**
 * \brief      this function handles CAN0 Rx0 ecxeption
 * \prarm[in]  none
 * \param[out] none
 * \retval     none
*/
#include "CanApp.h"
void CAN0_RX1_IRQHandler(void)
{
    CanAppDispatch();
}


/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  RecordEdgeInfo(&sAxis[0]);
  EXTI_PD = (uint32_t)EXTI_0;
	sAxis[0].sAlarm.RisingCnt++;
	sAxis[0].sAlarm.RisingCnt = sAxis[0].sAlarm.RisingCnt > 2000 ? 2000 : sAxis[0].sAlarm.RisingCnt;
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  RecordEdgeInfo(&sAxis[1]);
  EXTI_PD = (uint32_t)EXTI_3;
	sAxis[1].sAlarm.RisingCnt++;
	sAxis[1].sAlarm.RisingCnt = sAxis[1].sAlarm.RisingCnt > 2000 ? 2000 : sAxis[1].sAlarm.RisingCnt;
}

/*!
    \brief      this function handles TIMER2 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/



void TIMER4_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER4,TIMER_INT_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER4,TIMER_INT_CH0);
        
        __disable_irq();
        
        sAxis[0].sAlarm.PwmoutDisconnectCnt = 0;
        
        /* read channel 0 capture value */
        sAxis[0].sEncoder.PwmoutPd = timer_channel_capture_value_register_read(TIMER4,TIMER_CH_0)+1;
        
        if (sAxis[0].sEncoder.PwmoutPd != 0)
        {
            /* read channel 1 capture value */
            sAxis[0].sEncoder.PwmoutPPW = timer_channel_capture_value_register_read(TIMER4,TIMER_CH_1)+1;
        }
        else
        {
            sAxis[0].sAlarm.ErrReg.bit.PwmoutBreak = 1;
        }
        sAxis[0].sEncoder.PwmoutAB_Cnt_old = sAxis[0].sEncoder.PwmoutAB_Cnt;       
        sAxis[0].sEncoder.PwmoutAB_Cnt = TIMER_CNT(TIMER3);  
        sAxis[0].sEncoder.RisingCnt++;
        
        __enable_irq();
               
    }
}


void TIMER0_BRK_TIMER8_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER8,TIMER_INT_CH1)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER8,TIMER_INT_CH1);
        
       __disable_irq();
        
        sAxis[1].sAlarm.PwmoutDisconnectCnt = 0;
        
        /* read channel 0 capture value */
        sAxis[1].sEncoder.PwmoutPd = timer_channel_capture_value_register_read(TIMER8,TIMER_CH_1)+1;
        
        if (sAxis[1].sEncoder.PwmoutPd != 0)
        {
            /* read channel 1 capture value */
            sAxis[1].sEncoder.PwmoutPPW = timer_channel_capture_value_register_read(TIMER8,TIMER_CH_0)+1;
        }
        else
        {
            sAxis[1].sAlarm.ErrReg.bit.PwmoutBreak = 1;
        }
        
        sAxis[1].sEncoder.PwmoutAB_Cnt_old = sAxis[1].sEncoder.PwmoutAB_Cnt;       
        sAxis[1].sEncoder.PwmoutAB_Cnt = TIMER_CNT(TIMER2);
        sAxis[1].sEncoder.RisingCnt++;
        __enable_irq();
    }
}