
/* Includes ------------------------------------------------------------------*/
#include "delay.h"
#include "HardApi.h"

void delay_us(uint32_t nus)
{
	uint32_t temp = ReadTimeStampTimer();
    
    while((ReadTimeStampTimer()-temp) < nus*27) ;
}

void delay_ms(uint32_t nms)
{
    delay_us(nms*1000);
}
