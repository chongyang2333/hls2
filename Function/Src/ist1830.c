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

#include "ist1830.h"
#include "magfieldsensor_iic.h"
#include "CanApp.h"
#include "delay.h"
#include "gpio.h"
#include "gd_hal.h"



// Battery module: SDA
void Mag_SDA_WritePin(uint8_t PinState)
{
    gpio_bit_write(MAG_SDA_PORT, MAG_SDA_GPIO, (bit_status)PinState);
}

// Battery module: SCL
void Mag_SCL_WritePin(uint8_t PinState)
{
    gpio_bit_write(MAG_SCL_PORT, MAG_SCL_GPIO, (bit_status)PinState);
}

// Battery module: SDA Read
uint8_t Mag_SDA_ReadPin(void)
{
    return HAL_GPIO_ReadPin(MAG_SDA_PORT, MAG_SDA_GPIO);
}

// Battery module: SCL Read
uint8_t Mag_SCL_ReadPin(void)
{
    return HAL_GPIO_ReadPin(MAG_SCL_PORT, MAG_SCL_GPIO);
}


MagfieldsensorIICStruct MagSensor_I2c={
.SDA_WritePin=Mag_SDA_WritePin,
.SCL_WritePin=Mag_SCL_WritePin,
.SDA_ReadPinState=Mag_SDA_ReadPin,
.SCL_ReadPinState=Mag_SCL_ReadPin
	
};


void ist8310_Read_Len(uint8_t ReadAddr,uint8_t *data,uint8_t len)
{
		


}


void ist8310_Write_Len(uint8_t WriteAddr,uint8_t *data,uint8_t len)
{



}


void ist8310_Read_Byte(uint8_t ReadAddr,uint8_t *data)
{



}

void ist8310_Read_byte(uint8_t ReadAddr,uint8_t *data)
{



}






void ist8310_init(void)
{
		Mag_I2C_GPIO_Init();		
		

}

