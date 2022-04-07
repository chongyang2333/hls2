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

#ifdef MPU6050
/***********MPU6050****************/
#define I2C2_MPU6050				0xd0
#define	SMPLRT_DIV					0x19	
#define	CONFIG						  0x1A	
#define	GYRO_CONFIG					0x1B	
#define	ACCEL_CONFIG				0x1C	
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
#define	PWR_MGMT_1					0x6B	
#define	WHO_AM_I					  0x75	
#else
/***********ICM40609****************/
#define I2C2_ICM40609                   0xD0
#define ICM40609_DEVICE_CONFIG          0x11
#define ICM40609_ACCEL_XOUT_H           0x1F     // accelerometer Data
#define ICM40609_ACCEL_XOUT_L           0x20
#define ICM40609_ACCEL_YOUT_H           0x21
#define ICM40609_ACCEL_YOUT_L           0x22
#define ICM40609_ACCEL_ZOUT_H           0x23
#define ICM40609_ACCEL_ZOUT_L           0x24
#define ICM40609_GYRO_XOUT_H            0x25     // gyro Data
#define ICM40609_GYRO_XOUT_L            0x26
#define ICM40609_GYRO_YOUT_H            0x27
#define ICM40609_GYRO_YOUT_L            0x28
#define ICM40609_GYRO_ZOUT_H            0x29
#define ICM40609_GYRO_ZOUT_L            0x2A
#define ICM40609_LowV_CLK_SEL           0x4d     // clock select
#define ICM40609_PWR_MGMT0              0x4e
#define ICM40609_GYRO_CONFIG0           0x4f
#define ICM40609_ACCEL_CONFIG0          0x50
#define ICM40609_GYRO_CONFIG1           0x51     
#define ICM40609_GYRO_ACCEL_CONFIG0     0x52
#define ICM40609_ACCEL_CONFIG1          0x53     
#define ICM40609_WHO_AM_I               0x75     // default value ID = 0x3B;
#define ICM40609_REG_BANK_SEL           0x76     // REG_BANK_SEL
//REG_BANK_1
#define ICM40609_SENSOR_CONFIG0         0x03
#define ICM40609_XG_ST_DATA             0x5f     // X-gyro self-test data
#define ICM40609_YG_ST_DATA             0x60     // Y-gyro self-test data
#define ICM40609_ZG_ST_DATA             0x61     // Z-gyro self-test data
#endif

extern PUBLIC void GyroInit(void);
extern PUBLIC void GyroExec(void);
extern PUBLIC void GetGyroData(INT16 *pData);

#endif
