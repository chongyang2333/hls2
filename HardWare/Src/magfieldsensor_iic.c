
/* Includes ------------------------------------------------------------------*/
#include "magfieldsensor_iic.h"
#include "delay.h"
#include "gpio.h"
#include "gd_hal.h"

#define  DELAY_TIME 5
#define  IIC_DELAY  delay_us(DELAY_TIME)
#define  CLOCK_STRETCH_TIME (100*1000/DELAY_TIME-1)  //max clock time limit to 25ms

//MagfieldsensorIICStruct MagSensor_I2c={
//.SDA_WritePin=Mag_SDA_WritePin
//	


//}



void Mag_I2C_GPIO_Init(void)
{  
    /* enable gpio clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* configure I2C GPIO */
    gpio_output_options_set(MAG_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, MAG_SCL_GPIO);
    gpio_mode_set(MAG_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, MAG_SCL_GPIO);
    
    gpio_output_options_set(MAG_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, MAG_SDA_GPIO);
    gpio_mode_set(MAG_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, MAG_SDA_GPIO);

    gpio_bit_write(MAG_SCL_PORT, MAG_SCL_GPIO, SET);
    gpio_bit_write(MAG_SDA_PORT, MAG_SDA_GPIO, SET);
}

void sda_out(void)
{
    gpio_output_options_set(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_PIN_9);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_9);

    
    
    // GPIO_InitTypeDef GPIO_InitStruct;
    
    // GPIO_InitStruct.Pin = GPIO_PIN_9;
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
}

void magfieldsensor_iic_start(MagfieldsensorIICStruct* p)
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

uint8_t magfieldsensor_iic_stop(MagfieldsensorIICStruct* p)
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

void magfieldsensor_iic_ack(MagfieldsensorIICStruct* p)
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

void magfieldsensor_iic_noack(MagfieldsensorIICStruct* p)
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

_Bool magfieldsensor_iic_waitack(MagfieldsensorIICStruct* p)
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
            magfieldsensor_iic_stop(p);
            return 0;
        }
        IIC_DELAY;
    }
    IIC_DELAY;
    p->SCL_WritePin(0);
    IIC_DELAY;
    return 1;
}

void magfieldsensor_iic_WriteByte(MagfieldsensorIICStruct* p, uint8_t txd)
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


uint8_t magfieldsensor_iic_ReadByte(MagfieldsensorIICStruct* p, uint8_t *pRdata)
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

