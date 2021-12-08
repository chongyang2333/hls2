/*******************************************************************
 *
 * FILE NAME:  Gyro.h
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

#ifndef _ist1830_H_
#define _ist1830_H_

#include "UserDataTypes.h"

#define L_ist8310_Adder				0x0F
#define R_ist8310_Adder				0x0E


#define Who_Am_I 						0x00
#define STATUS_1						0x02
#define MAG_X_L 						0x03
#define MAG_X_H							0x04
#define MAG_Y_L 						0x05
#define MAG_Y_H 						0x06
#define MAG_Z_L 						0x07
#define MAG_Z_H							0x08
#define STATUS_2						0x09
#define CONTROL_1						0x0A
#define CONTROL_2						0x0B
#define TEMPERATURE_L				0x1C
#define TEMPERATURE_H				0x1D

#endif





