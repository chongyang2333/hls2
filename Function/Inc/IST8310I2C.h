/*******************************************************************
 *
 * FILE NAME:  IST8310I2C.h
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

#ifndef _IST8310I2C_H_
#define _IST8310I2C_H_

#include "UserDataTypes.h"
#include "gd32f4xx.h"

extern PUBLIC void IST8310I2C_Init(void);
//extern PUBLIC void IST8310I2C_Serial_Write(UINT16 Writeaddr,UINT8 *Str,UINT16 Len);
extern PUBLIC void IST8310I2C_Serial_Read(UINT16 Readaddr,UINT8 *Str,UINT16 Len);
extern PUBLIC void IST8310_Byte_Write(UINT16 Writeaddr,UINT8 Writedata);
void IST8310_SDA_WritePin(uint8_t PinState);
void IST8310_SCL_WritePin(uint8_t PinState);
uint8_t IST8310_SDA_ReadPin(void);
void IST8310_SDA_SetPinDir(uint8_t PinState);
void IST8310_SCL_SetPinDir(uint8_t PinState);

extern UINT8 IST8310I2C_WADDR;   
extern UINT8 IST8310I2C_RADDR; 
extern UINT8 alarm_level;
extern UINT8 alarm_levelBak; 
#endif
