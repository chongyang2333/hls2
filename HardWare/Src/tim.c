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

#define DEAD_TIME_FOR_TIM   192   // 214:2us   172: 1us
// timer 时钟 0 7 8 9 10 100Mhz  1 2 3 4 5 6 11 12 13  200Mhz
void gpio_timer0_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOE);

    /*configure PE8(TIMER0 CH0) as alternate function*/
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_8);

    /*configure PE9(TIMER0 CH0N) as alternate function*/
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_9);

   /*configure PE10(TIMER0 CH1) as alternate function*/
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_10);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_10);

    /*configure PE11(TIMER0 CH1N) as alternate function*/
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_11);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_11);
		
		/*configure PE12(TIMER0 CH2) as alternate function*/
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_12);

    /*configure PE13(TIMER0 CH2N) as alternate function*/
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_13);
}
/* TIM1 init function */
void MX_TIM0_Init(void)
{
		timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;
	  /*gpio set*/
	  gpio_timer0_config();
	
	  rcu_periph_clock_enable(RCU_TIMER0);
	  rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    
       nvic_irq_enable(TIMER0_UP_TIMER9_IRQn,0,0);
    
    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 0; // 200M
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = PWM_PERIOD_VALUE;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV2;  //Tds = fclk/2
    timer_initpara.repetitioncounter = 1;
    timer_init(TIMER0,&timer_initpara);

    /* CH0/CH0N configuration in PWM mode0 */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,4999); //设定比较值
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,4999);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,4999);

    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_ENABLE);

    /* automatic output enable, break, dead time and lock configuration*/
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_ENABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_ENABLE ;
    timer_breakpara.deadtime         = 138; //tdts = 1/100M  1.5us死区
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_0;
    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
    timer_break_config(TIMER0,&timer_breakpara);

		 timer_master_slave_mode_config(TIMER0,TIMER_MASTER_SLAVE_MODE_ENABLE);
		 timer_slave_mode_select(TIMER0,TIMER_TRI_OUT_SRC_UPDATE);
    /* TIMER0 primary output function enable */
    timer_primary_output_config(TIMER0,ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);

    /* TIMER0 counter enable */
    timer_enable(TIMER0);
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{
		/* TIMER2 configuration: PWM input mode ------------------------
     the external signal is connected to TIMER2 CH0 pin(PB4)
     the rising edge is used as active edge
     the TIMER2 CH0CV is used to compute the frequency value 
     the TIMER2 CH1CV is used to compute the duty cycle value
  ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    /*configure PB4  B5(TIMER2 CH0) as alternate function*/
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_4);
	
	gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_5);
	
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);


    /* TIMER2 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 65535;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* TIMER2 configuration */
    /* TIMER2 CH0 PWM input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x8;
    timer_input_pwm_capture_config(TIMER2,TIMER_CH_0,&timer_icinitpara);
    timer_input_pwm_capture_config(TIMER2,TIMER_CH_1,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2,TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_slave_mode_select(TIMER2,TIMER_ENCODER_MODE2);

    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER2,TIMER_MASTER_SLAVE_MODE_ENABLE);

    /* TIMER2 counter enable */
    timer_enable(TIMER2); 

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
	timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    /*configure PD12  D13(TIMER3 CH0) as alternate function*/
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
    gpio_af_set(GPIOD, GPIO_AF_2, GPIO_PIN_12);
	
		gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_af_set(GPIOD, GPIO_AF_2, GPIO_PIN_13);
	
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);

    timer_deinit(TIMER3);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 65535;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* TIMER2 configuration */
    /* TIMER2 CH0 PWM input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x8;
    timer_input_pwm_capture_config(TIMER3,TIMER_CH_0,&timer_icinitpara);
    timer_input_pwm_capture_config(TIMER3,TIMER_CH_1,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER3,TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_slave_mode_select(TIMER3,TIMER_ENCODER_MODE2);

    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER3,TIMER_MASTER_SLAVE_MODE_ENABLE);

    /* TIMER2 counter enable */
    timer_enable(TIMER3); 

}


void gpio_timer7_config(void)
{
    /*configure Pc6(TIMER7 CH0) as alternate function*/
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_6);
    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_6);

    /*configure PA7(TIMER7 CH0N) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
    gpio_af_set(GPIOA, GPIO_AF_3, GPIO_PIN_7);

   /*configure PC7(TIMER7 CH1) as alternate function*/
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
    gpio_af_set(GPIOC, GPIO_AF_3, GPIO_PIN_7);

    /*configure PB0(TIMER7 CH1N) as alternate function*/
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_3, GPIO_PIN_0);
		
		/*configure PC8(TIMER7 CH2) as alternate function*/
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_af_set(GPIOC, GPIO_AF_3, GPIO_PIN_8);

    /*configure PB1(TIMER7 CH2N) as alternate function*/
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
    gpio_af_set(GPIOB, GPIO_AF_3, GPIO_PIN_1);
}

/* TIM7 init function */
void MX_TIM7_Init(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;
	  /*gpio set*/
	  gpio_timer7_config();
	
	  rcu_periph_clock_enable(RCU_TIMER7);
	  rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    
    timer_deinit(TIMER7);

    /* TIMER7 configuration */
    timer_initpara.prescaler         = 0; // 200M
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = PWM_PERIOD_VALUE;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV2;  //Tds = fclk/2
    timer_initpara.repetitioncounter = 1;
    timer_init(TIMER7,&timer_initpara);

    /* CH0/CH0N configuration in PWM mode0 */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER7,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER7,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER7,TIMER_CH_2,&timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_0,4999); //设定比较值
    timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_1,4999);
    timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_2,4999);
    timer_channel_output_mode_config(TIMER7,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER7,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER7,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER7,TIMER_CH_0,TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER7,TIMER_CH_1,TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER7,TIMER_CH_2,TIMER_OC_SHADOW_ENABLE);

    /* automatic output enable, break, dead time and lock configuration*/
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_ENABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_ENABLE ;
    timer_breakpara.deadtime         = 138; //tdts = 1/100M  1.5us死区
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_0;
    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
    timer_break_config(TIMER7,&timer_breakpara);

    timer_master_slave_mode_config(TIMER7,TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_slave_mode_select(TIMER7,TIMER_TRI_OUT_SRC_UPDATE);
    /* TIMER7 primary output function enable */
    timer_primary_output_config(TIMER7,ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER7);

    /* TIMER7 counter enable */
    timer_enable(TIMER7);

}

/***********************************************************************
 * DESCRIPTION: 用于延迟计时  在GD中timer4 对应st的timer5 计数器宽度32位
 *
 * RETURNS:
 *
***********************************************************************/
void MX_TIM4_Init(void)
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
void MX_TIM8_Init(void)
{
    timer_parameter_struct timer_initpara;
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    rcu_periph_clock_enable(RCU_TIMER8);
    
    nvic_irq_enable(TIMER0_BRK_TIMER8_IRQn,1,2);
    
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
