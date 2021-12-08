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

#ifndef _ISTMAGIC_H_
#define _ISTMAGIC_H_

#include "UserDataTypes.h"

#define IST8310_REG_WIA				0x00	//Who I am
#define IST8310_REG_STAT1			0x02	//Status 1 register
#define IST8310_REG_DATAX			0x03	//Output Value x
#define IST8310_REG_DATAY			0x05	//Output Value y
#define IST8310_REG_DATAZ			0x07	//Output Value z
#define IST8310_REG_STAT2			0x09	//Status 2 register
#define IST8310_REG_CNTRL1			0x0A	//Control setting register 1
#define IST8310_REG_CNTRL2			0x0B	//Control setting register 2
#define IST8310_REG_SELF_TEST		0x0C	//Self Test
#define IST8310_REG_CNTRL3			0x0D	//Control setting register 3
#define IST8310_REG_TEMP_L			0x1C	//Low byte of Temperature
#define IST8310_REG_TEMP_H			0x1D	//High byte of Temperature
#define IST8310_REG_TCCNTL       	0x40    //Temperature Compensation Control register
#define IST8310_REG_AVGCNTL         0x41    //Average Control register
#define IST8310_REG_PULSE_DURATION  0x42    //Pulse Duration Control

extern PUBLIC void ISTMagic_init(void);
extern PUBLIC void MagXYZ_Exec(void);
#endif
