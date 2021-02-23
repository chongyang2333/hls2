/*!
    \file    main.c
    \brief   Demo--GPIO
    
    \version 
*/

#include "gd32f4xx.h"

void gpio_config(void);

int main()
{
    NVIC_SetPriorityGrouping(NVIC_PRIGROUP_PRE4_SUB0); 
    
    SystemInit(); // 时钟配置
    
    systick_config(); //用于延时
    gpio_config();
    
    while(1)
    {
       gpio_bit_set(GPIOD,GPIO_PIN_15);
       delay_1ms(1000);
       gpio_bit_reset(GPIOD,GPIO_PIN_15);
       delay_1ms(1000);
    }
    
}

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    
    gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_15);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_200MHZ,GPIO_PIN_15);
    
    gpio_bit_set(GPIOD,GPIO_PIN_15);
}
