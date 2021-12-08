
/* Includes ------------------------------------------------------------------*/
#include "gpioiic.h"
#include "delay.h"

void gpioiic_start(GpioIICStruct* GpioIIC)
{  
    GpioIIC->SDA_WritePin(1);  
    GpioIIC->SCL_WritePin(1);
    delay_us(3);
    GpioIIC->SDA_WritePin(0);
    delay_us(3);
    GpioIIC->SCL_WritePin(0);
    delay_us(3);
}

void gpioiic_stop(GpioIICStruct* GpioIIC)
{
    GpioIIC->SCL_WritePin(0);
    GpioIIC->SDA_WritePin(0);
    delay_us(3);
    GpioIIC->SCL_WritePin(1);
		delay_us(2);
    GpioIIC->SDA_WritePin(1);
    delay_us(3);
}

void gpioiic_ack(GpioIICStruct* GpioIIC)
{  
    GpioIIC->SCL_WritePin(0);
    delay_us(3);
    GpioIIC->SDA_WritePin(0);
    delay_us(3);
    GpioIIC->SCL_WritePin(1);
    delay_us(3);
    GpioIIC->SCL_WritePin(0);
    delay_us(3);
}

void gpioiic_noack(GpioIICStruct* GpioIIC)
{
    GpioIIC->SCL_WritePin(0);
    delay_us(3);
    GpioIIC->SDA_WritePin(1);
    delay_us(3);
    GpioIIC->SCL_WritePin(1);
    delay_us(3);
    GpioIIC->SCL_WritePin(0);
    delay_us(3);
}

#if 1 // add EEPROM_IMPROVE 2019-12-21
_Bool gpioiic_waitack(GpioIICStruct* GpioIIC)
{
    uint32_t time=0;
    uint32_t AckCnt = 0;

    GpioIIC->SDA_SetPinDir( 0 );
    GpioIIC->SDA_WritePin(1);
    GpioIIC->SCL_WritePin(1);
    delay_us(2);

    if( GpioIIC->SDA_ReadPinState() )
    {
        AckCnt++;
    }
    delay_us(1);
    
    if( GpioIIC->SDA_ReadPinState() )
    {
        AckCnt++;
    }
    delay_us(1);
    
    if( GpioIIC->SDA_ReadPinState() )
    {
        AckCnt++;
    }
    delay_us(1);

    GpioIIC->SCL_WritePin( 0 );
    GpioIIC->SDA_SetPinDir( 1 );
    
    if( AckCnt >= 2 )
    {
        return( 0 );
    }
    else
    {
        return( 1 );
    }
}
#else
_Bool gpioiic_waitack(GpioIICStruct* GpioIIC)
{
    uint32_t time=0;

    GpioIIC->SCL_WritePin(0);
    delay_us(3);
    
    while(GpioIIC->SDA_ReadPinState())
    {
        if(time++>60000)
        {
            myiic_stop(MyIIC);
            return 0;
        }
    }
    GpioIIC->SCL_WritePin(1);
    delay_us(3);
    GpioIIC->SCL_WritePin(0);
    delay_us(3);
    return 1;
}


#endif
void gpioiic_WriteByte(GpioIICStruct* GpioIIC, uint8_t txd)
{
    uint8_t len=0;

    for (len=0;len<8;len++)
    {
        GpioIIC->SCL_WritePin(0);  //SCL_L
        delay_us(3); 
        if(txd & 0x80)
            GpioIIC->SDA_WritePin(1);  //SDA_H
        else
            GpioIIC->SDA_WritePin(0);  //SDA_L
        delay_us(3);   
        GpioIIC->SCL_WritePin(1);    //SCL_H
        txd <<= 1;
        delay_us(3); 
    }
    GpioIIC->SCL_WritePin(0);  //SCL_L
    delay_us(3);

}

#if 0 // EEPROM_IMPROVE 
uint8_t gpioiic_ReadByte(GpioIICStruct* GpioIIC)
{
    uint8_t len=0,dat = 0;

    GpioIIC->SDA_WritePin(1);
    for(len=0;len<8;len++) 
    {
        GpioIIC->SCL_WritePin(0);  //SCL_L
        delay_us(3);
        GpioIIC->SCL_WritePin(1);  //SCL_H
        delay_us(3); 
        dat <<= 1;
        dat |= GpioIIC->SDA_ReadPinState(); 
        delay_us(3); 
    }   
    
    GpioIIC->SCL_WritePin(0);  //SCL_L
    delay_us(3);
    
    return dat;
}
#else
uint8_t gpioiic_ReadByte(GpioIICStruct* GpioIIC)
{
    uint8_t len=0,dat = 0;

    GpioIIC->SDA_SetPinDir( 0 );
    GpioIIC->SDA_WritePin(1);
    for(len=0;len<8;len++) 
    {
        int Cnt = 0;
        
        GpioIIC->SCL_WritePin(0);  //SCL_L
        delay_us(2);

        GpioIIC->SCL_WritePin(1);  //SCL_H
        delay_us(2); 
        dat <<= 1;
        if( GpioIIC->SDA_ReadPinState() )
        {
            Cnt++;
        }
        delay_us(1); 
        if( GpioIIC->SDA_ReadPinState() )
        {
            Cnt++;
        }
        delay_us(1); 
        if( GpioIIC->SDA_ReadPinState() )
        {
            Cnt++;
        }
        if( Cnt >= 2 )
        {
            dat |= 0x01; 
        }
    }   
    
    GpioIIC->SCL_WritePin(0);  //SCL_L
    delay_us(3);
    GpioIIC->SDA_SetPinDir( 1 );
    return dat;
}


#endif

