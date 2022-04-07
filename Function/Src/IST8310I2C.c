/*******************************************************************
 *
 * FILE NAME:  IST8310I2C.c
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2018.10.23
 *
 * AUTHOR:      Denny
 *
 * History:
------------------------------------------------------------------------
23-10-2018 Version 1.00 : Created by Denny
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/

#include "IST8310I2C.h"
#include "gpioiic.h"
#include "delay.h"
#include "gpio.h"
#include "gd_hal.h"

GpioIICStruct  sGpioIIC;

UINT8 IST8310I2C_WADDR;
UINT8 IST8310I2C_RADDR;
UINT16 IST8310_Cfg = 0;

#define IST8310I2C_RETRY_NUM 3
//PRIVATE void At24c02_Write_Byte(UINT16 addr,UINT8 dat);
//PRIVATE UINT8 At24c02_Read_Byte(UINT16 addr);
//PRIVATE void EEPROM_Page_Write(UINT16 Writeaddr,UINT8 *Str,UINT16 Len);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void IST8310I2C_Init(void)
{
    UINT8 dataread = 0;
    UINT8 i = 0;

    sGpioIIC.SDA_WritePin = IST8310_SDA_WritePin;
    sGpioIIC.SCL_WritePin = IST8310_SCL_WritePin;
    sGpioIIC.SDA_ReadPinState = IST8310_SDA_ReadPin;
    sGpioIIC.SCL_SetPinDir = IST8310_SCL_SetPinDir;
    sGpioIIC.SDA_SetPinDir = IST8310_SDA_SetPinDir;

    IST8310_Cfg = 0;

    IST8310I2C_WADDR = 0x18;
    ExtVEnable();//打开3.3V
    while(dataread != 0x10)
    {
        IST8310I2C_Serial_Read(0x0,&dataread,1);
        i++;
        if(dataread == 0x10)
        {
            IST8310_Cfg |= 0x1;
            break;
        }
        if(i>=3)
        {
            break;
        }
    }

    IST8310I2C_WADDR = 0x1C;
    i = 0;
    dataread = 0;
    while(dataread != 0x10)
    {
        IST8310I2C_Serial_Read(0x0,&dataread,1);
        i++;
        if(dataread == 0x10)
        {
            IST8310_Cfg |= 0x2;
            break;
        }
        if(i>=3)
        {
            break;
        }
    }
    ExtVDisable();//关闭3.3V
}


unsigned char IST8310_IsBusy( void )
{
    unsigned char i = 0;

    gpioiic_start(&sGpioIIC);
    gpioiic_WriteByte(&sGpioIIC,IST8310I2C_WADDR);
    i = gpioiic_waitack(&sGpioIIC);
    gpioiic_stop(&sGpioIIC);

    return( !i );
}
/***********************************************************************
 * DESCRIPTION: This function can not write eeprom across page
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void IST8310_Byte_Write(UINT16 Writeaddr,UINT8 Writedata)
{
    unsigned char RetryCnt = 0;

    for( RetryCnt = 0; RetryCnt < 10; RetryCnt++ )
    {
        // if( !IST8310_IsBusy() )
        {
            break;
        }

        // delay_ms(1);
    }

    for( RetryCnt = 0; RetryCnt < IST8310I2C_RETRY_NUM; RetryCnt++ )
    {
        gpioiic_start( &sGpioIIC );
        gpioiic_WriteByte( &sGpioIIC,IST8310I2C_WADDR);
        if( !gpioiic_waitack(&sGpioIIC) )
        {
            gpioiic_stop(&sGpioIIC);
            continue;
        }
        gpioiic_WriteByte( &sGpioIIC,Writeaddr);
        if( !gpioiic_waitack(&sGpioIIC) )
        {
            gpioiic_stop(&sGpioIIC);
            continue;
        }
        gpioiic_WriteByte(&sGpioIIC, Writedata);
        if(!gpioiic_waitack(&sGpioIIC))
        {
            break;
        }
        gpioiic_stop(&sGpioIIC);
        break;
    }
    for( RetryCnt = 0; RetryCnt < 10; RetryCnt++ )
    {
        // delay_ms(1);
        // if( !IST8310_IsBusy() )
        {
            break;
        }
    }
}
PUBLIC void IST8310I2C_Serial_Read(UINT16 Readaddr,UINT8 *Str,UINT16 Len)
{
    unsigned int  Num;
    unsigned char RetryCnt;
    unsigned char *pBuff;

    if( ( ( UINT8 * )0 == Str ) || ( 0 == Len ) )
    {
        return;
    }

    for( RetryCnt = 0; RetryCnt < IST8310I2C_RETRY_NUM; RetryCnt++ )
    {
        Num = Len;
        pBuff = ( unsigned char * )Str;

        gpioiic_start(&sGpioIIC);
        gpioiic_WriteByte(&sGpioIIC,IST8310I2C_WADDR);
        if( !gpioiic_waitack(&sGpioIIC) )
        {
            gpioiic_stop(&sGpioIIC);
            continue;
        }
        //gpioiic_start(&sGpioIIC);
        gpioiic_WriteByte(&sGpioIIC,Readaddr);
        if( !gpioiic_waitack(&sGpioIIC) )
        {
            gpioiic_stop(&sGpioIIC);
            continue;
        }
        gpioiic_start(&sGpioIIC);
        gpioiic_WriteByte(&sGpioIIC,IST8310I2C_WADDR+1);
        if( !gpioiic_waitack(&sGpioIIC) )
        {
            gpioiic_stop(&sGpioIIC);
            continue;
        }
        for( ; Num > 0; Num-- )
        {
            *pBuff++ = gpioiic_ReadByte(&sGpioIIC);
            if( 1 == Num )
            {
                gpioiic_noack( &sGpioIIC );
            }
            else
            {
                gpioiic_ack( &sGpioIIC );
            }
        }

        gpioiic_stop(&sGpioIIC);
        break;
    }
}

//SDA
void IST8310_SDA_WritePin(uint8_t PinState)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, PinState);
}

//SCL
void IST8310_SCL_WritePin(uint8_t PinState)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, PinState);
}

//SDA Read
uint8_t IST8310_SDA_ReadPin(void)
{
    return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
}

void IST8310_SDA_SetPinDir( uint8_t IsOut )
{
//    GPIO_InitTypeDef GPIO_Config;
//
//    /*Configure GPIO pins : PB9 PB8:SDA SCL */
//    GPIO_Config.Pin = GPIO_PIN_2;
//    if( IsOut )
//    {
//        GPIO_Config.Mode = GPIO_MODE_OUTPUT_OD;
//    }
//    else
//    {
//        GPIO_Config.Mode = GPIO_MODE_INPUT;
//    }
//    GPIO_Config.Pull = GPIO_NOPULL;
//    GPIO_Config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    HAL_GPIO_Init( GPIOE, &GPIO_Config );
}

void IST8310_SCL_SetPinDir( uint8_t IsOut )
{
//    GPIO_InitTypeDef GPIO_Config;
//
//    /*Configure GPIO pins : PB8:SDA SCL */
//    GPIO_Config.Pin = GPIO_PIN_6;
//    if( IsOut )
//    {
//        GPIO_Config.Mode = GPIO_MODE_OUTPUT_OD;
//    }
//    else
//    {
//        GPIO_Config.Mode = GPIO_MODE_INPUT;
//    }
//    GPIO_Config.Pull = GPIO_NOPULL;
//    GPIO_Config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    HAL_GPIO_Init( GPIOE, &GPIO_Config);
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/



