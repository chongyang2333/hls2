
/* Includes ------------------------------------------------------------------*/
#include "battery_iic.h"
#include "systick.h"

#define  DELAY_TIME 20
#define  IIC_DELAY  delay_us(DELAY_TIME)
#define  CLOCK_STRETCH_TIME (25*1000/DELAY_TIME-1)  //max clock time limit to 25ms

void sda_out(void)
{
    // GPIO_InitTypeDef GPIO_InitStruct;
    
    // GPIO_InitStruct.Pin = GPIO_PIN_9;
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
}

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

