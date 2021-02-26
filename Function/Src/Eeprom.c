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

MyIICStruct sMyIIC;

#define EEPROM_WADDR   0XA0
#define EEPROM_RADDR   0XA1
#define EEPROM_PAGE_SIZE  32

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

/***********************************************************************
 * DESCRIPTION: This function can not write eeprom across page
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void EEPROM_Page_Write(UINT16 Writeaddr,UINT8 *Str,UINT16 Len)
{
    myiic_start(&sMyIIC);    
    myiic_WriteByte(&sMyIIC,EEPROM_WADDR);  
    myiic_waitack(&sMyIIC);  
    myiic_WriteByte(&sMyIIC,Writeaddr>>8);  
    myiic_waitack(&sMyIIC);
    myiic_WriteByte(&sMyIIC,Writeaddr);  
    myiic_waitack(&sMyIIC);
 
	while(Len--)
	{
        myiic_WriteByte(&sMyIIC,*Str);  
        myiic_waitack(&sMyIIC);
		Str++;		
	}
	myiic_stop(&sMyIIC);
	delay_ms(10);  
    
#if 0   
    while(Len--)
    {
        At24c02_Write_Byte(Writeaddr,*Str);
        Writeaddr++;
        Str++;
    }
#endif
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
    myiic_start(&sMyIIC); 
	myiic_WriteByte(&sMyIIC,EEPROM_WADDR);  
    myiic_waitack(&sMyIIC);    
    myiic_WriteByte(&sMyIIC,Readaddr>>8);  
    myiic_waitack(&sMyIIC);
    myiic_WriteByte(&sMyIIC,Readaddr);  
    myiic_waitack(&sMyIIC);
    myiic_start(&sMyIIC); 
    myiic_WriteByte(&sMyIIC,EEPROM_RADDR);
    myiic_waitack(&sMyIIC);
    
	while(Len)
	{
		*Str = myiic_ReadByte(&sMyIIC);
		if(Len==1) myiic_noack(&sMyIIC);
		else myiic_ack(&sMyIIC);
		Str++;
		Len--;
	}
	myiic_stop(&sMyIIC);

#if 0
    while(Len)
    {
        *Str++ = At24c02_Read_Byte(Readaddr++);
        Len--;
    }
#endif
    
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
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, PinState);
}

// Eeprom module: SCL
void EEPROM_SCL_WritePin(uint8_t PinState)
{
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, PinState);
}

// Eeprom module: SDA Read
uint8_t EEPROM_SDA_ReadPin(void)
{
    // return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
}