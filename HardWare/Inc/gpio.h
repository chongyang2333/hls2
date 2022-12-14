/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f4xx.h"
#include "Param.h"
#include "PowerManager.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


#define MUSIC_ENABLE_PORT   GPIOB
#define MUSIC_ENABLE_PIN    GPIO_PIN_14
#define MUSIC_MUTE_PORT     GPIOB
#define MUSIC_MUTE_PIN      GPIO_PIN_15


UINT8 MX_GPIO_Init(void);

void battery_I2C_GPIO_Init(void);
void Battery_SDA_WritePin(uint8_t PinState);
void Battery_SCL_WritePin(uint8_t PinState);
uint8_t Battery_SDA_ReadPin(void);
uint8_t Battery_SCL_ReadPin(void);

void BEMF_DischargeOn(void);
void BEMF_DischargeOff(void);

void VbusEnable(void);
void VbusDisable(void);
UINT8 ReadVbusEnableState(void);

void VbusBufferEnable(void);
void VbusBufferDisable(void);

void DrvPwEnable(void);
void DrvPwDisable(void);

void LidarPowerOn(void);
void LidarPowerOff(void);
uint8_t ReadLidarPowerState(void);

void McuPowerOff(void);
void McuPowerOn(void);

void EnableCharge(UINT8 ApplicationMode);
void DisableCharge(UINT8 ApplicationMode);
PUBLIC UINT8 ReadChargeMosState(void);

uint8_t ReadKeyInPinState(void);
uint8_t ReadChargeInPinState(void);

void PadPowerOn(void);
void PadPowerOff(void);
uint8_t ReadPadPowerState(void);

UINT16 VeneerAgingTestState(void);

void DisinfectionModulePowerOn(void);
void DisinfectionModulePowerOff(void);

PUBLIC UINT8 ReadApplicationMode(void);

void MusicPwEnable(void);
void MusicPwDisable(void);

void MuteEnable(void);
void MuteDisable(void);
void ExtVEnable(void);
void ExtVDisable(void);
void EXOLidarPowerOff(void);
void EXOLidarPowerOn(void);
void LOGOEnable(void);
void LOGODisable(void);
#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
