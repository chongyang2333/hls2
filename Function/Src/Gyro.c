/*******************************************************************
 *
 * FILE NAME:  Gyro.c
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

#include "Gyro.h"
#include "i2c.h"
#include "CanApp.h"
#include "delay.h"

PRIVATE void I2C_Gyro_ByteWrite(UINT8 WriteAddr, UINT8 WriteData);
PRIVATE void I2C_Gyro_BufferRead(UINT8* pBuffer, UINT8 ReadAddr, UINT8 NumByteToRead);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void GyroInit(void)
{
    MX_I2C2_Init();
	
#ifdef MPU6050
//	delay_ms(20);//100
//	I2C_Gyro_ByteWrite(PWR_MGMT_1,0x00);
//	delay_ms(10);
//	I2C_Gyro_ByteWrite(SMPLRT_DIV,0x07);
//	delay_ms(10);
//	I2C_Gyro_ByteWrite(CONFIG,0x06);
//	delay_ms(10);
// 	I2C_Gyro_ByteWrite(GYRO_CONFIG,0x08);
//	delay_ms(10);
// 	I2C_Gyro_ByteWrite(ACCEL_CONFIG,0x01);
//	delay_ms(40);//500
#else
    delay_ms(20);
    I2C_Gyro_ByteWrite(ICM40609_DEVICE_CONFIG,0x01);   //reset
    delay_ms(10);
    I2C_Gyro_ByteWrite(ICM40609_DEVICE_CONFIG,0x00);   //normol
    delay_ms(10);
    I2C_Gyro_ByteWrite(ICM40609_ACCEL_CONFIG0,0x66);   //set ACCEL  rate 1000HZ and scale 2g
    delay_ms(10);
    I2C_Gyro_ByteWrite(ICM40609_GYRO_CONFIG0,0x46);    //set GYRO  rate 1000HZ and scale 500deg
    delay_ms(10);
    I2C_Gyro_ByteWrite(ICM40609_PWR_MGMT0,0x0F);       //set GYRO and ACCEL work in LN mode
    delay_ms(10);
    I2C_Gyro_ByteWrite(ICM40609_GYRO_ACCEL_CONFIG0,0x77); //set bandwidth
    delay_ms(10);
    I2C_Gyro_ByteWrite(ICM40609_LowV_CLK_SEL,0x01); //set CLKSEL
    delay_ms(40);
#endif
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
INT16 gyro[3], accel[3];
PUBLIC void GyroExec(void)
{
#ifdef MPU6050	
	UINT8 DataBuffer[14] = {0};	
/****************************加速度****************************************/
	I2C_Gyro_BufferRead(DataBuffer, ACCEL_XOUT_H, 14);
	accel[0]  = (short)((DataBuffer[0]<<8)+DataBuffer[1]);	  //读取X轴加速度
	
//	I2C_Gyro_BufferRead(DataBuffer, ACCEL_YOUT_H, 2);
	accel[1]  = -(short)((DataBuffer[2]<<8)+DataBuffer[3]);	  //读取Y轴加速度
	
//	I2C_Gyro_BufferRead(DataBuffer, ACCEL_ZOUT_H, 2);
	accel[2]  = -(short)((DataBuffer[4]<<8)+DataBuffer[5]);	  //读取Z轴加速度
	
/****************************角速度****************************************/
//	I2C_Gyro_BufferRead(DataBuffer, GYRO_XOUT_H, 2);
	gyro[0] = (short)((DataBuffer[8]<<8)+DataBuffer[9]);	      //读取X轴角速度
	
//	I2C_Gyro_BufferRead(DataBuffer, GYRO_YOUT_H, 2);
	gyro[1] = -(short)((DataBuffer[10]<<8)+DataBuffer[11]);	      //读取Y轴角速度
	
//	I2C_Gyro_BufferRead(DataBuffer, GYRO_ZOUT_H, 2);
	gyro[2] = -(short)((DataBuffer[12]<<8)+DataBuffer[13]);	      //读取Z轴角速度
	
#else	
    UINT8 DataBuffer[12] = {0};
    /****************************加速度****************************************/
    I2C_Gyro_BufferRead(DataBuffer, ICM40609_ACCEL_XOUT_H, 12);
    accel[0]  = 2*(short)((DataBuffer[0]<<8)+DataBuffer[1]);      //读取X轴加速度

//  I2C_Gyro_BufferRead(DataBuffer, ACCEL_YOUT_H, 2);
    accel[1]  = 2*(short)((DataBuffer[2]<<8)+DataBuffer[3]);     //读取Y轴加速度

//  I2C_Gyro_BufferRead(DataBuffer, ACCEL_ZOUT_H, 2);
    accel[2]  = 2*(short)((DataBuffer[4]<<8)+DataBuffer[5]);     //读取Z轴加速度

    /****************************角速度****************************************/
//  I2C_Gyro_BufferRead(DataBuffer, GYRO_XOUT_H, 2);
    gyro[0] = (short)((DataBuffer[6]<<8)+DataBuffer[7]);          //读取X轴角速度

//  I2C_Gyro_BufferRead(DataBuffer, GYRO_YOUT_H, 2);
    gyro[1] = (short)((DataBuffer[8]<<8)+DataBuffer[9]);         //读取Y轴角速度

//  I2C_Gyro_BufferRead(DataBuffer, GYRO_ZOUT_H, 2);
    gyro[2] = (short)((DataBuffer[10]<<8)+DataBuffer[11]);       //读取Z轴角速度
#endif
    CanSendGyro(gyro, accel);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void GetGyroData(INT16 *pData)
{
    pData[0] = gyro[0];
    pData[1] = gyro[1];
    pData[2] = gyro[2];

    pData[3] = accel[0];
    pData[4] = accel[1];
    pData[5] = accel[2];
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void I2C_Gyro_ByteWrite(UINT8 WriteAddr, UINT8 WriteData)
{
    i2c_mem_write(I2C1, I2C2_ICM40609, WriteAddr, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 5000);
}

PRIVATE void I2C_Gyro_ByteWrite_Weak(UINT8 WriteAddr, UINT8 WriteData)
{
    // HAL_I2C_Mem_Write(&hi2c2, I2C2_MPU6050,WriteAddr, I2C_MEMADD_SIZE_8BIT, &WriteData,1,500);
    // while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void I2C_Gyro_BufferRead(UINT8* pBuffer, UINT8 ReadAddr, UINT8 NumByteToRead)
{
    i2c_mem_read(I2C1, I2C2_ICM40609, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead, 5000);
}

PRIVATE void I2C_Gyro_BufferRead_Weak(UINT8* pBuffer, UINT8 ReadAddr, UINT8 NumByteToRead)
{
    // HAL_I2C_Mem_Read(&hi2c2, I2C2_MPU6050, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead,1000);
    // while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
}

