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

#ifndef _GYRO_APP_H_
#define _GYRO_APP_H_

#include "UserDataTypes.h"

#define I2C2_MPU6050					0xd0


/***********???MPU6050???????****************/
#define	SMPLRT_DIV					0x19	//??????,???:0x07(125Hz)
#define	CONFIG						0x1A	//??????,???:0x06(5Hz)
#define	GYRO_CONFIG					0x1B	//??????????,???:0x18(???,2000deg/s)
#define	ACCEL_CONFIG				0x1C	//?????????????????,???:0x01(???,2G,5Hz)
#define	ACCEL_XOUT_H				0x3B
#define	ACCEL_XOUT_L				0x3C
#define	ACCEL_YOUT_H				0x3D
#define	ACCEL_YOUT_L				0x3E
#define	ACCEL_ZOUT_H				0x3F
#define	ACCEL_ZOUT_L				0x40
#define	TEMP_OUT_H					0x41
#define	TEMP_OUT_L					0x42
#define	GYRO_XOUT_H					0x43
#define	GYRO_XOUT_L					0x44	
#define	GYRO_YOUT_H					0x45
#define	GYRO_YOUT_L					0x46
#define	GYRO_ZOUT_H					0x47
#define	GYRO_ZOUT_L					0x48
#define	PWR_MGMT_1					0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I					0x75	//IIC地址寄存器(默认数值0x68，只读)

extern PUBLIC void GyroInit(void);
extern PUBLIC void GyroExec(void);
extern PUBLIC void GetGyroData(INT16 *pData);

#endif
