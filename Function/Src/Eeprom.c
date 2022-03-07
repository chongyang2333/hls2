/*******************************************************************
 *
 * FILE NAME:  Eeprom.c
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

#include "Eeprom.h"
#include "myiic.h"
#include "gpio.h"
#include "delay.h"
#include "gd_hal.h"

MyIICStruct sMyIIC;

#define EEPROM_WADDR   0XA0
#define EEPROM_RADDR   0XA1
#define EEPROM_PAGE_SIZE  32
#define EEPROM_RETRY_NUM        ( 3 )

PRIVATE void At24c02_Init(void);
//PRIVATE void At24c02_Write_Byte(UINT16 addr,UINT8 dat);
//PRIVATE UINT8 At24c02_Read_Byte(UINT16 addr);
PRIVATE void EEPROM_Page_Write(UINT16 Writeaddr,UINT8 *Str,UINT16 Len);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void EepromInit(void)
{
    At24c02_Init();
}
/*==================================================================================================
���ƣ�EEPROM_IsBusy
���ܣ��ж�EEPROMоƬ�Ƿ����ڲ�æ״̬
���룺��
�����0:��æ,��0��æ
==================================================================================================*/
unsigned char EEPROM_IsBusy( void )
{
    unsigned char i = 0;

    myiic_start(&sMyIIC);
    myiic_WriteByte(&sMyIIC,EEPROM_WADDR);
    i = myiic_waitack(&sMyIIC);
    myiic_stop(&sMyIIC);

    return( !i );
}
/***********************************************************************
 * DESCRIPTION: This function can not write eeprom across page
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void EEPROM_Page_Write(UINT16 Writeaddr,UINT8 *Str,UINT16 Len)
{
    unsigned int  Num;
    unsigned char RetryCnt = 0;
    unsigned char *pBuff;

    if( 0 == Len )
    {
        return;
    }

    for( RetryCnt = 0; RetryCnt < 10; RetryCnt++ )
    {
        if( !EEPROM_IsBusy() )
        {
            break;
        }

        delay_ms(1);
    }

    for( RetryCnt = 0; RetryCnt < EEPROM_RETRY_NUM; RetryCnt++ )
    {
        Num = Len;
        pBuff = ( unsigned char * )Str;
        myiic_start( &sMyIIC );
        myiic_WriteByte( &sMyIIC,EEPROM_WADDR);
        if( !myiic_waitack(&sMyIIC) )
        {
            myiic_stop(&sMyIIC);
            continue;
        }

        myiic_WriteByte(&sMyIIC,Writeaddr>>8);
        if( !myiic_waitack(&sMyIIC) )
        {
            myiic_stop(&sMyIIC);
            continue;
        }
        myiic_WriteByte(&sMyIIC,Writeaddr);
        if( !myiic_waitack(&sMyIIC) )
        {
            myiic_stop(&sMyIIC);
            continue;
        }

        while( Num--)
        {
            myiic_WriteByte(&sMyIIC, *pBuff++ );
            if( !myiic_waitack(&sMyIIC) )
            {
                break;
            }
        }

        myiic_stop(&sMyIIC);
        if( 0 == Num )
        {
            break;
        }
    }

    for( RetryCnt = 0; RetryCnt < 10; RetryCnt++ )
    {
        delay_ms(1);
        if( !EEPROM_IsBusy() )
        {
            break;
        }
    }
}
/***********************************************************************
 * DESCRIPTION: This function can write eeprom across page
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void EEPROM_Serial_Write(UINT16 Writeaddr,UINT8 *Str,UINT16 Len)
{
	UINT8 NumOfPage=0,NumOfSingle=0,Addr=0,count=0; 
    UINT16 NumByteToWrite = Len;
    
	Addr = Writeaddr % EEPROM_PAGE_SIZE; 
	count = EEPROM_PAGE_SIZE - Addr; 
	NumOfPage = NumByteToWrite/EEPROM_PAGE_SIZE; 
	NumOfSingle = NumByteToWrite % EEPROM_PAGE_SIZE; 
    
	if(Addr == 0) 
	{
		if(NumOfPage == 0) 
		{
			EEPROM_Page_Write(Writeaddr, Str, NumOfSingle); 		
		}
		else 
		{
			while(NumOfPage) 
			{
				EEPROM_Page_Write(Writeaddr, Str, EEPROM_PAGE_SIZE);
				Writeaddr += EEPROM_PAGE_SIZE;
				Str += EEPROM_PAGE_SIZE;
				NumOfPage--;
				delay_ms(10);
			}
			if(NumOfSingle!=0) 
			{
				EEPROM_Page_Write(Writeaddr, Str, NumOfSingle);	
				delay_ms(10);
			}
		}
	}
	else 
	{
		if(NumOfPage == 0) 
		{
            if(count >= NumByteToWrite)
            {
                EEPROM_Page_Write(Writeaddr, Str, NumByteToWrite);
            }
            else
            {
                EEPROM_Page_Write(Writeaddr, Str, count);
                Writeaddr = Writeaddr + count;
                Str = Str + count;
                EEPROM_Page_Write(Writeaddr, Str, NumByteToWrite-count);
            }
		}
		else 
		{
			NumByteToWrite -= count;
			NumOfPage = NumByteToWrite / EEPROM_PAGE_SIZE;
			NumOfSingle = NumByteToWrite % EEPROM_PAGE_SIZE;
			
			if(count!=0)
			{
				EEPROM_Page_Write(Writeaddr, Str, count);
				Writeaddr += count;
				Str +=count;
			}
			
			while(NumOfPage--) 
			{
				EEPROM_Page_Write(Writeaddr, Str, EEPROM_PAGE_SIZE); 
				
				Writeaddr += EEPROM_PAGE_SIZE;
				Str += EEPROM_PAGE_SIZE;				
			}
			
			if(NumOfSingle != 0) 
			{
				EEPROM_Page_Write(Writeaddr, Str, NumOfSingle); 
			}
		}
		
	}
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void EEPROM_Serial_Read(UINT16 Readaddr,UINT8 *Str,UINT16 Len)
{
    unsigned int  Num;
    unsigned char RetryCnt;
    unsigned char *pBuff;

    if( ( ( UINT8 * )0 == Str ) || ( 0 == Len ) )
    {
        return;
    }

    for( RetryCnt = 0; RetryCnt < EEPROM_RETRY_NUM; RetryCnt++ )
    {
        Num = Len;
        pBuff = ( unsigned char * )Str;
        myiic_start(&sMyIIC);
        myiic_WriteByte(&sMyIIC,EEPROM_WADDR);
        if( !myiic_waitack(&sMyIIC) )
        {
            myiic_stop(&sMyIIC);
            continue;
        }
        myiic_WriteByte(&sMyIIC,Readaddr>>8);
        if( !myiic_waitack(&sMyIIC) )
        {
            myiic_stop(&sMyIIC);
            continue;
        }
        myiic_WriteByte(&sMyIIC,Readaddr);
        if( !myiic_waitack(&sMyIIC) )
        {
            myiic_stop(&sMyIIC);
            continue;
        }

        myiic_start(&sMyIIC);
        myiic_WriteByte(&sMyIIC,EEPROM_RADDR);
        if( !myiic_waitack(&sMyIIC) )
        {
            myiic_stop(&sMyIIC);
            continue;
        }

        for( ; Num > 0; Num-- )
        {
            *pBuff++ = myiic_ReadByte(&sMyIIC);
            if( 1 == Num )
            {
                myiic_noack( &sMyIIC );
            }
            else
            {
                myiic_ack( &sMyIIC );
            }
        }

        myiic_stop(&sMyIIC);
        break;
    }
}

#if 0
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void At24c02_Write_Byte(UINT16 addr,UINT8 dat)
{
    myiic_start(&sMyIIC);    
    myiic_WriteByte(&sMyIIC,EEPROM_WADDR);  
    myiic_waitack(&sMyIIC);  
    myiic_WriteByte(&sMyIIC,addr>>8);  
    myiic_waitack(&sMyIIC);
    myiic_WriteByte(&sMyIIC,addr);  
    myiic_waitack(&sMyIIC);
    myiic_WriteByte(&sMyIIC,dat);  
    myiic_waitack(&sMyIIC);
    myiic_stop(&sMyIIC);
    delay_ms(50);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 At24c02_Read_Byte(UINT16 addr)
{
    UINT8 dat=0;

    myiic_start(&sMyIIC); 
    myiic_WriteByte(&sMyIIC,EEPROM_WADDR);  
    myiic_waitack(&sMyIIC);
    myiic_WriteByte(&sMyIIC,addr>>8);  
    myiic_waitack(&sMyIIC);
    myiic_WriteByte(&sMyIIC,addr);  
    myiic_waitack(&sMyIIC);
    myiic_start(&sMyIIC); 
    myiic_WriteByte(&sMyIIC,EEPROM_RADDR);  
    myiic_waitack(&sMyIIC);
    dat = myiic_ReadByte(&sMyIIC);
    myiic_stop(&sMyIIC);

    return dat;
}
#endif

// Eeprom module: SDA
void EEPROM_SDA_WritePin(uint8_t PinState)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, PinState?GPIO_PIN_SET:GPIO_PIN_RESET);
}

// Eeprom module: SCL
void EEPROM_SCL_WritePin(uint8_t PinState)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, PinState?GPIO_PIN_SET:GPIO_PIN_RESET);
}

// Eeprom module: SDA Read
uint8_t EEPROM_SDA_ReadPin(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
}
void EEPROM_SDA_SetPinDir( uint8_t IsOut )
{

    /*Configure GPIO pins : PB9 PB8: EEPROM SDA SCL */
    if( IsOut )
    {
         gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_9);
			 
    }
    else
    {
				 gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_9);
    }
}

void EEPROM_SCL_SetPinDir( uint8_t IsOut )
{

    /*Configure GPIO pins : PB9 PB8: EEPROM SDA SCL */
    if( IsOut )
    {
         gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_8);
			 
    }
    else
    {
				 gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_8);
    }
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void At24c02_Init(void)
{
    sMyIIC.SDA_WritePin = EEPROM_SDA_WritePin;
    sMyIIC.SCL_WritePin = EEPROM_SCL_WritePin;
    sMyIIC.SDA_ReadPinState = EEPROM_SDA_ReadPin;
    sMyIIC.SCL_SetPinDir = EEPROM_SCL_SetPinDir;
    sMyIIC.SDA_SetPinDir = EEPROM_SDA_SetPinDir;
}


