/*!
    \file    systick.c
    \brief   the systick configuration file

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

#include "gd32f4xx.h"
#include "systick.h"

#define SYSTICK_LOAD_VALUE 0XFFFFFF

volatile static uint32_t delay;

/*!
    \brief    configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for  interrupts */
    if (SysTick_Config(SYSTICK_LOAD_VALUE))
    { //  max 0xFFFFFFUL
        /* capture error */
        while (1)
        {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/**
 * @brief 
 * 
 */
volatile static uint32_t TimeStampBase = 0;
void SysTick_Handler(void)
{
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
    {
        TimeStampBase += SYSTICK_LOAD_VALUE;
    }
}

/**
 * @brief 获取滴答定时器的时间戳,会关总中断，最好不要循环执行。
 * 
 * @return  
 */
uint32_t ReadSystickStampTimer(void)
{
    uint32_t systick_tmp;
    uint32_t ret;

    __set_PRIMASK(1); // 关闭所有中断

    systick_tmp = SysTick->VAL; // 第一次获取

    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) // 禁止中断这段时间产生了systick重装载初值的情况
    {

        // 再获取一次滴答值，此次一定是重装载后的值，第一次获取有可能是重装载前的值
        systick_tmp = SysTick->VAL;
        TimeStampBase += SYSTICK_LOAD_VALUE;
        ret = TimeStampBase + SYSTICK_LOAD_VALUE - systick_tmp;
    }
    else
    {
        ret = TimeStampBase + SYSTICK_LOAD_VALUE - systick_tmp;
    }
    //return TIMER1->TIMER_CNT;
    __set_PRIMASK(0);

    return ret;
}

/**
 * @brief 延迟1us
 * 
 * @param count 
 */
void delay_1us(uint32_t count)
{
    uint32_t delay_cnt = count;
    uint32_t time_begin;
    uint32_t time_tmp;
    uint32_t systick_cnt = 0;
    uint32_t delt_cnt = 0;
    
    while ( systick_cnt < delay_cnt * (SystemCoreClock/1000000)) //执行时间大概0.12us
    {
        time_begin = SysTick->VAL;
        systick_cnt += delt_cnt;
        delt_cnt = 0;
        
        while( delt_cnt <= (SystemCoreClock/1000000)  ) // 循环等待1us
        {   
            time_tmp = SysTick->VAL;

            if (time_tmp > time_begin)
            {
                delt_cnt =  time_begin + SYSTICK_LOAD_VALUE - time_tmp;
            }
            else
            {
                delt_cnt = time_begin - time_tmp;
            }
        }
        
    }
}

// /*!
//     \brief    delay a time in milliseconds
//     \param[in]  count: count in milliseconds
//     \param[out] none
//     \retval     none
// */
// void delay_1ms(uint32_t count)
// {
//     delay = count;

//     while (0U != delay)
//     {
//     }
// }

/*!
    \brief    delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void delay_decrement(void)
{
    if (0U != delay)
    {
        delay--;
    }
}
