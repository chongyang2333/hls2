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
#include "delay.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/***********************************************************************
 * DESCRIPTION:ADC GPIO定义
 *
 * RETURNS:
 *
***********************************************************************/
void gpio_adc_config(void)
{
    /* ADC0: PA4,PA5,PC4,PC5    IN4 IN5  IN14 IN15
    ** ADC1: PA1,PA2 PA3        IN1 IN2 IN3
    ** ADC2: PC0,PC1,PC2        IN10 IN11 IN12
    ** PC3 IN13
    ** PA0 IN0
	  ** PA6 IN6
    **/
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
   
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_1|GPIO_PIN_2
                  |GPIO_PIN_6);
    gpio_mode_set(GPIOC,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_0|GPIO_PIN_1
                    |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
    
}

/***********************************************************************
 * DESCRIPTION:ADC 定义
 *
 * RETURNS:
 *
***********************************************************************/
void adc_config(void)
{

    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);

    /*adc prescale*/
    adc_clock_config(ADC_ADCCK_HCLK_DIV5); //200/5 = 40M

    /* configure the ADC sync mode */
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1,ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC2,ADC_DATAALIGN_RIGHT);
    /* ADC SCAN function enable */
    adc_special_function_config(ADC0,ADC_SCAN_MODE,ENABLE);
    adc_special_function_config(ADC1,ADC_SCAN_MODE,ENABLE);
    adc_special_function_config(ADC2,ADC_SCAN_MODE,ENABLE);

    /* ADC channel length config */
    adc_channel_length_config(ADC0,ADC_INSERTED_CHANNEL,2);
    adc_channel_length_config(ADC1,ADC_INSERTED_CHANNEL,3);
    adc_channel_length_config(ADC2,ADC_INSERTED_CHANNEL,3);
    /* ADC insert channel config */
    adc_inserted_channel_config(ADC0,0,ADC_CHANNEL_14,ADC_SAMPLETIME_3);   //MT_BUS_V_SAMPLE 
    adc_inserted_channel_config(ADC0,1,ADC_CHANNEL_15,ADC_SAMPLETIME_3);   //MAIN_V_SAMPLE 
		
		adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_10,ADC_SAMPLETIME_3);   //L_U_I_SAMPLE
    adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_11,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
    adc_inserted_channel_config(ADC1,2,ADC_CHANNEL_6,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
		
    adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_1,ADC_SAMPLETIME_3);    //R_U_I_SAMPLE
    adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_2,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
    adc_inserted_channel_config(ADC2,2,ADC_CHANNEL_13,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE

    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
    adc_interrupt_flag_clear(ADC2, ADC_INT_FLAG_EOIC);
    adc_interrupt_disable(ADC1, ADC_INT_EOIC);
		adc_interrupt_disable(ADC2, ADC_INT_EOIC);
    /* enable ADC interface */
    adc_enable(ADC0);
    /* wait for ADC stability */
    delay_ms(2);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    /* enable ADC interface */
    adc_enable(ADC1);
    /* wait for ADC stability */
    delay_ms(2);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
    /* enable ADC interface */
    adc_enable(ADC2);
    /* wait for ADC stability */
    delay_ms(2);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC2);
}

/***********************************************************************
 * DESCRIPTION:ADC 重定义 使用高级定时器CH3通道触发，用于电阻采样电流
 *
 * RETURNS:
 *
***********************************************************************/
void adc_reconfig(void)
{   
    adc_external_trigger_config(ADC1,ADC_INSERTED_CHANNEL,EXTERNAL_TRIGGER_FALLING);
    adc_external_trigger_config(ADC2,ADC_INSERTED_CHANNEL,EXTERNAL_TRIGGER_FALLING);
    adc_external_trigger_source_config(ADC2,ADC_INSERTED_CHANNEL,ADC_EXTTRIG_INSERTED_T7_CH3);
    adc_external_trigger_source_config(ADC1,ADC_INSERTED_CHANNEL,ADC_EXTTRIG_INSERTED_T0_CH3);

    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
	  adc_interrupt_flag_clear(ADC2, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC2, ADC_INT_FLAG_EOIC);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
