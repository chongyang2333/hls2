
/* Includes ------------------------------------------------------------------*/
#include "myiic.h"
#include "delay.h"

void myiic_start(MyIICStruct* MyIIC)
{
    MyIIC->SDA_WritePin(1);
    MyIIC->SCL_WritePin(1);
    delay_us(5);
    MyIIC->SDA_WritePin(0);
    delay_us(5);
    MyIIC->SCL_WritePin(0);
    delay_us(5);
}

void myiic_stop(MyIICStruct* MyIIC)
{
    MyIIC->SCL_WritePin(0);
    MyIIC->SDA_WritePin(0);
    delay_us(5);
    MyIIC->SCL_WritePin(1);
    delay_us(4);
    MyIIC->SDA_WritePin(1);
    delay_us(5);
}

void myiic_ack(MyIICStruct* MyIIC)
{
    MyIIC->SCL_WritePin(0);
    delay_us(5);
    MyIIC->SDA_WritePin(0);
    delay_us(5);
    MyIIC->SCL_WritePin(1);
    delay_us(5);
    MyIIC->SCL_WritePin(0);
    delay_us(5);
}

void myiic_noack(MyIICStruct* MyIIC)
{
    MyIIC->SCL_WritePin(0);
    delay_us(5);
    MyIIC->SDA_WritePin(1);
    delay_us(5);
    MyIIC->SCL_WritePin(1);
    delay_us(5);
    MyIIC->SCL_WritePin(0);
    delay_us(5);
}

#if 1 // add EEPROM_IMPROVE 2019-12-21
_Bool myiic_waitack(MyIICStruct* MyIIC)
{
    uint32_t time=0;
    uint32_t AckCnt = 0;

    MyIIC->SDA_SetPinDir( 0 );
    MyIIC->SDA_WritePin(1);
    MyIIC->SCL_WritePin(1);
    delay_us(2);

    if( MyIIC->SDA_ReadPinState() )
    {
        AckCnt++;
    }
    delay_us(1);

    if( MyIIC->SDA_ReadPinState() )
    {
        AckCnt++;
    }
    delay_us(1);

    if( MyIIC->SDA_ReadPinState() )
    {
        AckCnt++;
    }
    delay_us(1);

    MyIIC->SCL_WritePin( 0 );
    MyIIC->SDA_SetPinDir( 1 );

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
_Bool myiic_waitack(MyIICStruct* MyIIC)
{
    uint32_t time=0;

    MyIIC->SCL_WritePin(0);
    delay_us(5);

    while(MyIIC->SDA_ReadPinState())
    {
        if(time++>60000)
        {
            myiic_stop(MyIIC);
            return 0;
        }
    }
    MyIIC->SCL_WritePin(1);
    delay_us(5);
    MyIIC->SCL_WritePin(0);
    delay_us(5);
    return 1;
}


#endif
void myiic_WriteByte(MyIICStruct* MyIIC, uint8_t txd)
{
    uint8_t len=0;

    for (len=0; len<8; len++)
    {
        MyIIC->SCL_WritePin(0);  //SCL_L
        delay_us(5);
        if(txd & 0x80)
            MyIIC->SDA_WritePin(1);  //SDA_H
        else
            MyIIC->SDA_WritePin(0);  //SDA_L
        delay_us(5);
        MyIIC->SCL_WritePin(1);    //SCL_H
        txd <<= 1;
        delay_us(5);
    }
    MyIIC->SCL_WritePin(0);  //SCL_L
    delay_us(5);

}

#if 0 // EEPROM_IMPROVE 
uint8_t myiic_ReadByte(MyIICStruct* MyIIC)
{
    uint8_t len=0,dat = 0;

    MyIIC->SDA_WritePin(1);
    for(len=0; len<8; len++)
    {
        MyIIC->SCL_WritePin(0);  //SCL_L
        delay_us(5);
        MyIIC->SCL_WritePin(1);  //SCL_H
        delay_us(5);
        dat <<= 1;
        dat |= MyIIC->SDA_ReadPinState();
        delay_us(5);
    }

    MyIIC->SCL_WritePin(0);  //SCL_L
    delay_us(5);

    return dat;
}
#else
uint8_t myiic_ReadByte(MyIICStruct* MyIIC)
{
    uint8_t len=0,dat = 0;

    MyIIC->SDA_SetPinDir( 0 );
    MyIIC->SDA_WritePin(1);
    for(len=0; len<8; len++)
    {
        int Cnt = 0;

        MyIIC->SCL_WritePin(0);  //SCL_L
        delay_us(2);

        MyIIC->SCL_WritePin(1);  //SCL_H
        delay_us(2);
        dat <<= 1;
        if( MyIIC->SDA_ReadPinState() )
        {
            Cnt++;
        }
        delay_us(1);
        if( MyIIC->SDA_ReadPinState() )
        {
            Cnt++;
        }
        delay_us(1);
        if( MyIIC->SDA_ReadPinState() )
        {
            Cnt++;
        }
        if( Cnt >= 2 )
        {
            dat |= 0x01;
        }
    }

    MyIIC->SCL_WritePin(0);  //SCL_L
    delay_us(5);
    MyIIC->SDA_SetPinDir( 1 );
    return dat;
}


#endif

