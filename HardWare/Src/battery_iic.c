
/* Includes ------------------------------------------------------------------*/
#include "battery_iic.h"
#include "delay.h"

#define  DELAY_TIME 5
#define  IIC_DELAY  delay_us(DELAY_TIME)
#define  CLOCK_STRETCH_TIME (100*1000/DELAY_TIME-1)  //max clock time limit to 25ms


/***********************************************************************
 * DESCRIPTION:IIC SDA GPIO定义
 *
 * RETURNS:
 *
***********************************************************************/
void sda_out(void)
{
    gpio_output_options_set(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_PIN_9);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_9); 
}

/***********************************************************************
 * DESCRIPTION:IIC start
 *
 * RETURNS:
 *
***********************************************************************/
void battery_iic_start(BatteryIICStruct* p)
{ 
    p->SDA_WritePin(1);
	  IIC_DELAY;
    p->SCL_WritePin(1);
    
    IIC_DELAY;
    p->SDA_WritePin(0);
    IIC_DELAY;
	  p->SCL_WritePin(0);
	  IIC_DELAY;
}

/***********************************************************************
 * DESCRIPTION:IIC stop
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t battery_iic_stop(BatteryIICStruct* p)
{
    uint16_t clk_stretch_cnt = 0;     
    
    p->SDA_WritePin(0);
    IIC_DELAY;
    p->SCL_WritePin(1);
    while(!p->SCL_ReadPinState())
    {
        if(clk_stretch_cnt >= CLOCK_STRETCH_TIME)
        {
            return 0;
        }
        clk_stretch_cnt++;
        IIC_DELAY;
    }
    
    IIC_DELAY;
    p->SDA_WritePin(1);
    IIC_DELAY;
    return 1;
}

/***********************************************************************
 * DESCRIPTION:IIC stop
 *
 * RETURNS:
 *
***********************************************************************/
void battery_iic_ack(BatteryIICStruct* p)
{  
    p->SCL_WritePin(0);
    IIC_DELAY;
    p->SDA_WritePin(0);
    IIC_DELAY;
    p->SCL_WritePin(1);
    
    IIC_DELAY;
    p->SCL_WritePin(0);
    IIC_DELAY;
}

/***********************************************************************
 * DESCRIPTION:IIC no ack
 *
 * RETURNS:
 *
***********************************************************************/
void battery_iic_noack(BatteryIICStruct* p)
{   
    p->SCL_WritePin(0);
    IIC_DELAY;
    p->SDA_WritePin(1);
    
    IIC_DELAY;
    p->SCL_WritePin(1);
    
    IIC_DELAY;
    p->SCL_WritePin(0);
    IIC_DELAY;
}

/***********************************************************************
 * DESCRIPTION:IIC wait ack
 *
 * RETURNS:
 *
***********************************************************************/
_Bool battery_iic_waitack(BatteryIICStruct* p)
{
    uint16_t time=0;
    uint16_t clk_stretch_cnt = 0;     
    
    p->SDA_WritePin(1); 
    IIC_DELAY;
    p->SCL_WritePin(1);
    
    while(!p->SCL_ReadPinState())
    {
        if(clk_stretch_cnt >= CLOCK_STRETCH_TIME)
        {
            return 0;
        }
        clk_stretch_cnt++;
        IIC_DELAY;
    }    
    
    /*max wait time:2ms*/
    while(p->SDA_ReadPinState())
    {
        if(time++ > 100)
        {
            battery_iic_stop(p);
            return 0;
        }
        IIC_DELAY;
    }
    IIC_DELAY;
    p->SCL_WritePin(0);
    IIC_DELAY;
    return 1;
}

/***********************************************************************
 * DESCRIPTION:IIC WriteByte
 *
 * RETURNS:
 *
***********************************************************************/
void battery_iic_WriteByte(BatteryIICStruct* p, uint8_t txd)
{
    uint8_t len=0;   
    
    delay_us(50);
    for(len = 0; len < 8; len++)
    {
        if( txd & 0x80)
        {
            p->SDA_WritePin(1);
        }
        else
            p->SDA_WritePin(0);
        IIC_DELAY;
        
        p->SCL_WritePin(1); 
        IIC_DELAY;
        IIC_DELAY;
        p->SCL_WritePin(0);
        txd <<= 1;
        IIC_DELAY;
    }
}

/***********************************************************************
 * DESCRIPTION:IIC ReadByte
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t battery_iic_ReadByte(BatteryIICStruct* p, uint8_t *pRdata)
{
    uint8_t len=0;
    uint16_t clk_stretch_cnt = 0;      

    p->SDA_WritePin(1);
    
    delay_us(100);
    for(len=0;len<8;len++) 
    {
        p->SCL_WritePin(0);  
        IIC_DELAY;
        p->SCL_WritePin(1);
        
        clk_stretch_cnt = 0;        
        while(!p->SCL_ReadPinState())
        {
            if(clk_stretch_cnt >= CLOCK_STRETCH_TIME)
            {
                return 0;
            }
            clk_stretch_cnt++;
            IIC_DELAY;
        }        
        
        *pRdata <<= 1;
        *pRdata |= p->SDA_ReadPinState(); 
        IIC_DELAY; 
    }   
    
    p->SCL_WritePin(0);  
    return 1;
}

