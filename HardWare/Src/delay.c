
/* Includes ------------------------------------------------------------------*/
#include "delay.h"
#include "HardApi.h"

#if 0
void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = 27*nus;
    SysTick->VAL = 0X00;
    SysTick->CTRL = 0X01;
    do
    {
        temp = SysTick->CTRL;
    } while((temp&0x01)&&(!(temp&(1<<16))));
    SysTick->CTRL = 0X00;
    SysTick->VAL = 0X00;
}

void delay_ms(uint32_t nms)
{
    uint32_t temp;
    SysTick->LOAD = 27000*nms;
    SysTick->VAL = 0X00;
    SysTick->CTRL = 0X01;
    do
    {
        temp = SysTick->CTRL;
    } while((temp&0x01)&&(!(temp&(1<<16))));
    SysTick->CTRL = 0X00;
    SysTick->VAL = 0X00;
}
#endif

void delay_us(uint32_t nus)
{
    uint32_t temp = ReadTimeStampTimer();

    while((ReadTimeStampTimer()-temp) < nus*100) ;
}

void delay_ms(uint32_t nms)
{
    delay_us(nms*1000);
}
