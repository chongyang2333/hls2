/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "gd_hal.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

#define __I2C_SCL_PORT  GPIOA
#define __I2C_SCL_GPIO  GPIO_PIN_8
#define __I2C_SDA_PORT  GPIOC
#define __I2C_SDA_GPIO  GPIO_PIN_9

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB12   ------> USB_OTG_HS_ID
     PB14   ------> USB_OTG_HS_DM
     PB15   ------> USB_OTG_HS_DP
     PD0   ------> CAN1_RX
     PD1   ------> CAN1_TX
*/
UINT8 MX_GPIO_Init(void)
{
    UINT8 TmpChar = 0;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);

     /*Configure Left HALL GPIO pins PD7::HA  PB6:HB  PB7:HC */
    gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_7);
    gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_6);
    gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_7);
     /*Configure Right HALL GPIO pins :PC12:HA  PC10:HB  PC11:HC */
    gpio_mode_set(GPIOC,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_12);
    gpio_mode_set(GPIOC,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_10);
    gpio_mode_set(GPIOC,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_11);

    
     /*Configure GPIO pins : PE5 :DRVE_PW_EN(12V)*/
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_5);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);

    DrvPwDisable();
   
//     /*Configure GPIO pins : PC3 :MOTOR POWER ENABLE, PD15:MOTOR POWER BUFF ENABLE*/
    gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_5);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);
    VbusBufferDisable();
    
    gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_3);
    gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3);

    VbusDisable();

     /*Configure GPIO pins : PD3 :PRE_PWR_EN, 5v/5A power en */
     gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_3);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3);
     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    
//     /*Configure GPIO pins : PD4 :SYS_12V_EN (12V)*/
        gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_4);
        gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_4);
        
     /*Configure GPIO pins : PD6 :SYS_24V_EN (24V)*/
     gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_6);
     gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_6);
     PadPowerOn();

//     /*Configure GPIO pins : PA0 : RST_ACS711 */
     gpio_mode_set(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_0);
     gpio_output_options_set(GPIOA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
 
     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
    
//     /*Configure GPIO pins : PE3 :BEMF DISCHARGE */
       
       gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_3);
       gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3);
       
//     // disable BEMF discharge by default
     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

//     /*Configure GPIO pins : PE0 PE1 :led1,2 */
       gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_0);
       gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
       gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_1);
       gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_1);
       
//     /*Configure GPIO pins : PC13 :CAN_LED  PC14: STM32 heartbeat led*/
       gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_13);
       gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
       gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_14);
       gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);
    
//     /*Configure GPIO pins : PE4 : DC Voltage State */
       gpio_mode_set(GPIOE,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_4);
    
//     /*Configure GPIO pins : PB2:L_IU_FO, PE7 : L_IV_FO */
       gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_2);
       gpio_mode_set(GPIOE,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_7);
     
//     /*Configure GPIO pins : PA10 : SAFE_IN */
       gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_10);
       
//     /*Configure GPIO pins : PE14 : R_IU_FO */
       gpio_mode_set(GPIOE,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_14);
    
//     /*Configure GPIO pins : PE15 : R_IV_FO */
       gpio_mode_set(GPIOE,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_15);
 
//     /*Configure GPIO pins : PD2 : IBUS_FO */
       gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_2);
     
//     /*Configure GPIO pins : PB12: CHARGE_FO */
       gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_12);
  
//      /*Configure GPIO pins : PB9 PB8: EEPROM SDA SCL */
       gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_8);
       gpio_output_options_set(GPIOB,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
       gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_9);
       gpio_output_options_set(GPIOB,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

//     /*Configure GPIO pins : PB15,PA6: KEY_IN_DET, CHARGE_IN_DET */
        gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_15); 
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_6);  
    
//     /*Configure GPIO pins : PB14 :WAKE_IO*/
       gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_14);
       gpio_output_options_set(GPIOB,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);
        McuPowerOn();
    
//     /*Configure GPIO pins : PC15 : Tlc59108 RSTn*/
       gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_15);
       gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

// 	/************************超声雷达，暂时没用**********************************/
// 	GPIO_InitStruct.Pin = GPIO_PIN_14;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
// 	GPIO_InitStruct.Pin = GPIO_PIN_6;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
// 	GPIO_InitStruct.Pin = GPIO_PIN_5;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct); 
//     HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
// 	/**************************************************************/
    
//     /*Configure GPIO pins : PA12: 3399_HEART En */
       gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_12);
    
//     /*Configure GPIO pins : PD14:S1 PD15:S2：ApplicationMode*/
       gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_14);
       gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_15);
       TmpChar = ReadApplicationMode();
    
// //    /*Configure GPIO pin : PF4 */
// //    GPIO_InitStruct.Pin = GPIO_PIN_4;
// //    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
// //    GPIO_InitStruct.Pull = GPIO_NOPULL;
// //    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

// //    /* EXTI interrupt init*/
// //    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);   
    
//     /*Configure GPIO pins : PB13 :CHARGE_EN */
       gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_13);
       gpio_output_options_set(GPIOB,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
       DisableCharge(TmpChar);
    
    return TmpChar;
}


void battery_I2C_GPIO_Init(void)
{  
    /* enable gpio clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* configure I2C GPIO */
    gpio_output_options_set(__I2C_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C_SCL_GPIO);
    gpio_mode_set(__I2C_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, __I2C_SCL_GPIO);
    
    gpio_output_options_set(__I2C_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C_SDA_GPIO);
    gpio_mode_set(__I2C_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, __I2C_SDA_GPIO);

    gpio_bit_write(__I2C_SCL_PORT, __I2C_SCL_GPIO, SET);
    gpio_bit_write(__I2C_SDA_PORT, __I2C_SDA_GPIO, SET);
}

// Battery module: SDA
void Battery_SDA_WritePin(uint8_t PinState)
{
    gpio_bit_write(__I2C_SDA_PORT, __I2C_SDA_GPIO, (bit_status)PinState);
}

// Battery module: SCL
void Battery_SCL_WritePin(uint8_t PinState)
{
    gpio_bit_write(__I2C_SCL_PORT, __I2C_SCL_GPIO, (bit_status)PinState);
}

// Battery module: SDA Read
uint8_t Battery_SDA_ReadPin(void)
{
    return HAL_GPIO_ReadPin(__I2C_SDA_PORT, __I2C_SDA_GPIO);
}

// Battery module: SCL Read
uint8_t Battery_SCL_ReadPin(void)
{
    return HAL_GPIO_ReadPin(__I2C_SCL_PORT, __I2C_SCL_GPIO);
}

// Enable Back Electromotive Force Discharge 
void BEMF_DischargeOn(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);    
}

// Disable Back Electromotive Force Discharge 
void BEMF_DischargeOff(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);   
}

// Enable Vbus Buffer Voltage
void VbusEnable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);    
}

// Disable Vbus Buffer Voltage
void VbusDisable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  
}

// Read Vbus Enable State
UINT8 ReadVbusEnableState(void)
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);    
}

// Enable DC Voltage Buffer
void VbusBufferEnable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);    
}

// Disable DC Voltage Buffer
void VbusBufferDisable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);  
}

// Enable DC Voltage Buffer
void DrvPwEnable(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);    
}

// Disable DC Voltage Buffer
void DrvPwDisable(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);  
}

// turn off lidar power 
void LidarPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
}

// turn on lidar power 
void LidarPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
}

// Read Lidar power State
uint8_t ReadLidarPowerState(void)
{
		uint8_t state;
	
		state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
		state = !state;
		
		return state;
}

// turn off mcu power 
void McuPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

// turn on mcu power 
void McuPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

PRIVATE UINT8 ChargeMosState = 0;
void EnableCharge(UINT8 ApplicationMode)
{
    ChargeMosState = 1;
    if (!ApplicationMode)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

void DisableCharge(UINT8 ApplicationMode)
{
    ChargeMosState = 0;
    if (!ApplicationMode)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}


PUBLIC UINT8 ReadChargeMosState(void)
{
    return ChargeMosState;
}


// Read Key State
//����:1 , �ɿ�:0
uint8_t ReadKeyInPinState(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
}

// Read Charge State
uint8_t ReadChargeInPinState(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
}

// turn on pad power 
void PadPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
}

// turn off pad power 
void PadPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

// turn on Disinfection Module power 
void DisinfectionModulePowerOn(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

// turn off Disinfection Module power 
void DisinfectionModulePowerOff(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

// Read pad power State
uint8_t ReadPadPowerState(void)
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);
}

UINT16 VeneerAgingTestState(void)
{
	return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) + HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);
}

// Read Application Mode
PUBLIC UINT8 ReadApplicationMode(void)
{
    UINT8 TmpChar = 0;
    
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14))
    {
        TmpChar |= 0x01 << 0;
    }
    
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15))
    {
        TmpChar |= 0x01 << 1;
    }    
    
    return TmpChar;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
