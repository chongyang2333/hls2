/*******************************************************************
 *
 * FILE NAME:  Eeprom.h
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

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "UserDataTypes.h"
#include "stm32f7xx_hal.h"

extern PUBLIC void EepromInit(void);
extern PUBLIC void EEPROM_Serial_Write(UINT16 Writeaddr,UINT8 *Str,UINT16 Len);
extern PUBLIC void EEPROM_Serial_Read(UINT16 Readaddr,UINT8 *Str,UINT16 Len);
void EEPROM_SDA_WritePin(uint8_t PinState);
void EEPROM_SCL_WritePin(uint8_t PinState);
uint8_t EEPROM_SDA_ReadPin(void);
#endif
