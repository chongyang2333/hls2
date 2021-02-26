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
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

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
//     GPIO_InitTypeDef GPIO_InitStruct;
    UINT8 TmpChar = 0;

//     /* GPIO Ports Clock Enable */  
//     __HAL_RCC_GPIOA_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();
//     __HAL_RCC_GPIOC_CLK_ENABLE(); 
//     __HAL_RCC_GPIOD_CLK_ENABLE();
//     __HAL_RCC_GPIOE_CLK_ENABLE();
//     __HAL_RCC_GPIOF_CLK_ENABLE();
//     __HAL_RCC_GPIOG_CLK_ENABLE();
//     __HAL_RCC_GPIOH_CLK_ENABLE();

//     /*Configure Right HALL GPIO pins :PF9:HA  PF7:HB  PF8:HC */
//     GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
//     /*Configure Left HALL GPIO pins :PC12:HA  PC10:HB  PC11:HC */
//     GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_10;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//     /*Configure GPIO pins : PF5 :DRVE_PW_EN(12V)*/
//     GPIO_InitStruct.Pin = GPIO_PIN_5;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 
//     DrvPwDisable();
   
//     /*Configure GPIO pins : PG0 :MOTOR POWER ENABLE, PD15:MOTOR POWER BUFF ENABLE*/
//     GPIO_InitStruct.Pin = GPIO_PIN_15;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//     VbusBufferDisable();

//     GPIO_InitStruct.Pin = GPIO_PIN_0;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//     VbusDisable();

//     /*Configure GPIO pins : PG11 :PRE_PWR_EN, 5v/5A power en */
//     GPIO_InitStruct.Pin = GPIO_PIN_11;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//     HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);
    
//     /*Configure GPIO pins : PG12 :SYS_12V_EN (12V)*/
//     GPIO_InitStruct.Pin = GPIO_PIN_12;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct); 
    
//     /*Configure GPIO pins : PG14 :SYS_24V_EN (24V)*/
//     GPIO_InitStruct.Pin = GPIO_PIN_14;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct); 
//     PadPowerOn();

//     /*Configure GPIO pins : PF15 : RST_ACS711 */
//     GPIO_InitStruct.Pin = GPIO_PIN_15;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 
 
//     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
    
//     /*Configure GPIO pins : PF2 :BEMF DISCHARGE */
//     GPIO_InitStruct.Pin = GPIO_PIN_2;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//     // disable BEMF discharge by default
//     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

//     /*Configure GPIO pins : PE0 PE1 :led1,2 */
//     GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); 

//     /*Configure GPIO pins : PC13 :CAN_LED  PC14: STM32 heartbeat led*/
//     GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  
    
//     /*Configure GPIO pins : PF4 : DC Voltage State */
//     GPIO_InitStruct.Pin = GPIO_PIN_4;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);  
    
//     /*Configure GPIO pins : PF13:L_IU_FO, PF14 : L_IV_FO */
//     GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);  
    
//     /*Configure GPIO pins : PG1 : R_IU_FO */
//     GPIO_InitStruct.Pin = GPIO_PIN_1;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);  
    
//     /*Configure GPIO pins : PE7 : R_IV_FO */
//     GPIO_InitStruct.Pin = GPIO_PIN_7;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);  
 
//     /*Configure GPIO pins : PF10 : IBUS_FO */
//     GPIO_InitStruct.Pin = GPIO_PIN_10;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);  
     
//     /*Configure GPIO pins : PB10: CHARGE_FO */
//     GPIO_InitStruct.Pin = GPIO_PIN_10;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  
//      /*Configure GPIO pins : PB7 PB6: EEPROM SDA SCL */
//     GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
    
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

//     /*Configure GPIO pins : PF11,PF12: KEY_IN_DET, CHARGE_IN_DET */
//     GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 
 
    
//     /*Configure GPIO pins : PB2 :WAKE_IO*/
//     GPIO_InitStruct.Pin = GPIO_PIN_2;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
//     McuPowerOn();
    
//     /*Configure GPIO pins : PC15 : Tlc59108 RSTn*/
//     GPIO_InitStruct.Pin = GPIO_PIN_15;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
//     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

// 	/*Configure GPIO pins : PF15 : RST_ACS711 */
//     GPIO_InitStruct.Pin = GPIO_PIN_15;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); 

// 	/**********************************************************/
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
    
//     /*Configure GPIO pins : PB14: 消毒模块 En */
//     GPIO_InitStruct.Pin = GPIO_PIN_14;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
//     DisinfectionModulePowerOff();
    
//     /*Configure GPIO pins : PG2:S1 PG4:S2：ApplicationMode*/
//     GPIO_InitStruct.Pin = GPIO_PIN_2 |GPIO_PIN_4;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//     TmpChar = ReadApplicationMode();
    
// //    /*Configure GPIO pin : PF4 */
// //    GPIO_InitStruct.Pin = GPIO_PIN_4;
// //    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
// //    GPIO_InitStruct.Pull = GPIO_NOPULL;
// //    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

// //    /* EXTI interrupt init*/
// //    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);   
    
//     /*Configure GPIO pins : PB12 :CHARGE_EN */
//     GPIO_InitStruct.Pin = GPIO_PIN_12;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
// 	DisableCharge(TmpChar);
    
    return TmpChar;
}


void battery_I2C_GPIO_Init(void)
{  
    
    // GPIO_InitTypeDef GPIO_InitStruct;
    
    // /*Configure GPIO pins : PA8 : BATTERY SCL*/
    // GPIO_InitStruct.Pin = GPIO_PIN_8;
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
    // /*Configure GPIO pins :PC9: BATTERY SDA*/   
    // GPIO_InitStruct.Pin = GPIO_PIN_9;
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   
}

// Battery module: SDA
void Battery_SDA_WritePin(uint8_t PinState)
{
		// if(PinState)
		// 		GPIOC->BSRR = (1<<9);
		// else
		// 		GPIOC->BSRR = (1<<25);
}

// Battery module: SCL
void Battery_SCL_WritePin(uint8_t PinState)
{
		// if(PinState)
		// 		GPIOA->BSRR = (1<<8);
		// else
		// 		GPIOA->BSRR = (1<<24);
}

// Battery module: SDA Read
uint8_t Battery_SDA_ReadPin(void)
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
}

// Battery module: SCL Read
uint8_t Battery_SCL_ReadPin(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
}

// Enable Back Electromotive Force Discharge 
void BEMF_DischargeOn(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);    
}

// Disable Back Electromotive Force Discharge 
void BEMF_DischargeOff(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);   
}

// Enable Vbus Buffer Voltage
void VbusEnable(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);    
}

// Disable Vbus Buffer Voltage
void VbusDisable(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);  
}

// Read Vbus Enable State
UINT8 ReadVbusEnableState(void)
{
    return HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0);    
}

// Enable DC Voltage Buffer
void VbusBufferEnable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);    
}

// Disable DC Voltage Buffer
void VbusBufferDisable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);  
}

// Enable DC Voltage Buffer
void DrvPwEnable(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);    
}

// Disable DC Voltage Buffer
void DrvPwDisable(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET);  
}

// turn off lidar power 
void LidarPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
}

// turn on lidar power 
void LidarPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
}

// Read Lidar power State
uint8_t ReadLidarPowerState(void)
{
		uint8_t state;
	
		state = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12);
		state = !state;
		
		return state;
}

// turn off mcu power 
void McuPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

// turn on mcu power 
void McuPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

PRIVATE UINT8 ChargeMosState = 0;
void EnableCharge(UINT8 ApplicationMode)
{
    ChargeMosState = 1;
    if (!ApplicationMode)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    }
}

void DisableCharge(UINT8 ApplicationMode)
{
    ChargeMosState = 0;
    if (!ApplicationMode)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
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
    return HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);
}

// Read Charge State
uint8_t ReadChargeInPinState(void)
{
    return HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
}

// turn on pad power 
void PadPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
}

// turn off pad power 
void PadPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
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
    return HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14);
}

UINT16 VeneerAgingTestState(void)
{
	return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) + HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);
}

// Read Application Mode
PUBLIC UINT8 ReadApplicationMode(void)
{
    UINT8 TmpChar = 0;
    
    if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2))
    {
        TmpChar |= 0x01 << 0;
    }
    
    if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4))
    {
        TmpChar |= 0x01 << 1;
    }    
    
    return TmpChar;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
