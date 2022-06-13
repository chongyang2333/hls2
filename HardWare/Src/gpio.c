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

/***********************************************************************
 * DESCRIPTION:GPIO定义初始化
 *
 * RETURNS:
 *
***********************************************************************/
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

		/* PB12 电机老化 */
    gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_12);

    /*Configure GPIO pins : PE5 :DRVE_PW_EN(12V)*/
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_5);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);
    DrvPwDisable();
   
    /*Configure GPIO pins:PD10:MOTOR POWER BUFF ENABLE*/
    gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_10);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);
    VbusBufferDisable();
		
    //PE15:MOTOR POWER ENABLE
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_15);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
    VbusDisable();
		
	  /*Configure GPIO pins : PA9 : RST_ACS711 */
    gpio_mode_set(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_9);
    gpio_output_options_set(GPIOA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		
    /*Configure GPIO pins : PE0 PE1 :led1,2 */
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_0);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_1);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_1);		
		
    /*Configure GPIO pins : PC13 :CAN_LED  PC14: STM32 heartbeat led*/
    gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_13);
    gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_14);
    gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);

    /*Configure GPIO pins : PD3 : EXOLIDARPW */
    gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_3);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	
    /*Configure GPIO pins : PE4 : DC Voltage State */
    gpio_mode_set(GPIOE,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_4);
		
		/*DIAGNOSTIC_PWM  PA10*/
    gpio_mode_set(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_10);
    gpio_output_options_set(GPIOA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		
		/*Configure GPIO pins  PE14 : SAFE_IN1*/
    gpio_mode_set(GPIOE,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_14);

    /*Configure GPIO pins : PD2 : IBUS_FO */
    gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_2);

    /*Configure GPIO pins : PB9 PB8: EEPROM SDA SCL */
    gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_8);
    gpio_output_options_set(GPIOB,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_9);
    gpio_output_options_set(GPIOB,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);		
		
    /*Configure GPIO pins : PE2 PE6: SDA2 SCL2 */
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_2);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,GPIO_PIN_2);
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_6);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,GPIO_PIN_6);    
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
		 
    //PB3 : LOGO 使能
    gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_3);
    gpio_output_options_set(GPIOB,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		
		//PD15 : LOGO PWM
    gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_15);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			 
		/*Configure GPIO pins : PB15 : MUSIC ENABLE*/
    gpio_mode_set(MUSIC_ENABLE_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,MUSIC_ENABLE_PIN);
    gpio_output_options_set(MUSIC_ENABLE_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,MUSIC_ENABLE_PIN);
    MusicPwDisable();

    /*Configure GPIO pins : PB14 : MUTE*/
    gpio_mode_set(MUSIC_MUTE_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,MUSIC_MUTE_PIN);
    gpio_output_options_set(MUSIC_MUTE_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,MUSIC_MUTE_PIN);
    MuteEnable(); 
			 		 
    /*Configure GPIO pins : PB13 : 外部3.3V*/
    gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_13);
    gpio_output_options_set(GPIOB,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);			 
			 
    /*PD9 : HEAD-FO*/
    gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_9);			 
		 
    /*Configure GPIO pins : PA15 : RK3399 WEAK IO*/
    gpio_mode_set(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_15);
    gpio_output_options_set(GPIOA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);		 
		 
    /* PE7 ：RK3399_SYNC */
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_7);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_7);
		 
      
#ifdef USING_ENCODER_EXTI
    /*Configure GPIO pin : PA0-->LEFT MOTOR PWMOUT */

    gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLUP,GPIO_PIN_0);
    nvic_irq_enable(EXTI0_IRQn,0,0);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA,EXTI_SOURCE_PIN0);
    exti_init(EXTI_0,EXTI_INTERRUPT,EXTI_TRIG_RISING);
    LL_EXTI_DisableRisingTrig_0_31(EXTI_0);
    exti_interrupt_flag_clear(EXTI_0);

    /*Configure GPIO pin : PA3-->RIGHT MOTOR PWMOUT */
    gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLUP,GPIO_PIN_3);
    nvic_irq_enable(EXTI3_IRQn,0,0);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA,EXTI_SOURCE_PIN3);
    exti_init(EXTI_3,EXTI_INTERRUPT,EXTI_TRIG_RISING);
    LL_EXTI_DisableRisingTrig_0_31(EXTI_3);
    exti_interrupt_flag_clear(EXTI_3);
#endif

    TmpChar = ReadApplicationMode();
    DisableCharge(TmpChar);

    return TmpChar;
}

/***********************************************************************
 * DESCRIPTION:电池IIC初始化
 *
 * RETURNS:
 *
***********************************************************************/
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

/***********************************************************************
 * DESCRIPTION: Battery IIC module: SDA
 *
 * RETURNS:
 *
***********************************************************************/
void Battery_SDA_WritePin(uint8_t PinState)
{
    gpio_bit_write(__I2C_SDA_PORT, __I2C_SDA_GPIO, (bit_status)PinState);
}

/***********************************************************************
 * DESCRIPTION: Battery IIC module: SCL
 *
 * RETURNS:
 *
***********************************************************************/
void Battery_SCL_WritePin(uint8_t PinState)
{
    gpio_bit_write(__I2C_SCL_PORT, __I2C_SCL_GPIO, (bit_status)PinState);
}

/***********************************************************************
 * DESCRIPTION: Battery IIC module: SDA READ
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t Battery_SDA_ReadPin(void)
{
    return HAL_GPIO_ReadPin(__I2C_SDA_PORT, __I2C_SDA_GPIO);
}

/***********************************************************************
 * DESCRIPTION: Battery IIC module: SCL READ
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t Battery_SCL_ReadPin(void)
{
    return HAL_GPIO_ReadPin(__I2C_SCL_PORT, __I2C_SCL_GPIO);
}

/***********************************************************************
 * DESCRIPTION: Enable Vbus Buffer Voltage
 *
 * RETURNS:
 *
***********************************************************************/
void VbusEnable(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
}

/***********************************************************************
 * DESCRIPTION: Disable Vbus Buffer Voltage
 *
 * RETURNS:
 *
***********************************************************************/
void VbusDisable(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
}

/***********************************************************************
 * DESCRIPTION: Read Vbus Enable State
 *
 * RETURNS:
 *
***********************************************************************/
UINT8 ReadVbusEnableState(void)
{
    return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
}

/***********************************************************************
 * DESCRIPTION: Enable DC Voltage Buffer
 *
 * RETURNS:
 *
***********************************************************************/
void VbusBufferEnable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);    
}


/***********************************************************************
 * DESCRIPTION: Disable DC Voltage Buffer
 *
 * RETURNS:
 *
***********************************************************************/
void VbusBufferDisable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  
}

/***********************************************************************
 * DESCRIPTION: Enable DC Voltage Buffer
 *
 * RETURNS:
 *
***********************************************************************/
void DrvPwEnable(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);    
}

/***********************************************************************
 * DESCRIPTION: Disable DC Voltage Buffer
 *
 * RETURNS:
 *
***********************************************************************/
void DrvPwDisable(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);  
}

/***********************************************************************
 * DESCRIPTION: turn off EXO lidar power
 *
 * RETURNS:
 *
***********************************************************************/
void EXOLidarPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
}

/***********************************************************************
 * DESCRIPTION: turn on EXOlidar power
 *
 * RETURNS:
 *
***********************************************************************/
void EXOLidarPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET );
}

/***********************************************************************
 * DESCRIPTION: turn off  lidar power
 *
 * RETURNS:
 *
***********************************************************************/
void LidarPowerOff(void)
{
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
}

/***********************************************************************
 * DESCRIPTION: turn on lidar power 
 *
 * RETURNS:
 *
***********************************************************************/
void LidarPowerOn(void)
{
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET );
}

/***********************************************************************
 * DESCRIPTION: Read Lidar power State
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t ReadLidarPowerState(void)
{
    uint8_t state;

    state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);
    state = !state;

    return state;
}

/***********************************************************************
 * DESCRIPTION: turn off mcu power
 *
 * RETURNS:
 *
***********************************************************************/
void McuPowerOff(void)
{
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

/***********************************************************************
 * DESCRIPTION: turn on mcu power
 *
 * RETURNS:
 *
***********************************************************************/
void McuPowerOn(void)
{
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

/***********************************************************************
 * DESCRIPTION: EnableCharge
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 ChargeMosState = 0;
void EnableCharge(UINT8 ApplicationMode)
{
    ChargeMosState = 1;
//    if (!ApplicationMode)
//    {
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
//    }
//    else
//    {
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//    }
}

/***********************************************************************
 * DESCRIPTION: DisableCharge
 *
 * RETURNS:
 *
***********************************************************************/
void DisableCharge(UINT8 ApplicationMode)
{
    ChargeMosState = 0;
//    if (!ApplicationMode)
//    {
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//    }
//    else
//    {
//   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
//    }
}

/***********************************************************************
 * DESCRIPTION: ReadChargeMosState
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT8 ReadChargeMosState(void)
{
    return ChargeMosState;
}


/***********************************************************************
 * DESCRIPTION: Read Key State
 *
 * RETURNS:1按下 0释放
 *
***********************************************************************/
uint8_t ReadKeyInPinState(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
}

/***********************************************************************
 * DESCRIPTION: Read Charge State
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t ReadChargeInPinState(void)
{
    //return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
}


/***********************************************************************
* DESCRIPTION: 打开上位机电源
 *
 * RETURNS:
 *
***********************************************************************/
void PadPowerOn(void)
{
   // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
}

/***********************************************************************
* DESCRIPTION: 关闭上位机电源
 *
 * RETURNS:
 *
***********************************************************************/
void PadPowerOff(void)
{
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
}

/***********************************************************************
* DESCRIPTION: 打开消毒模块电源
 *
 * RETURNS:
 *
***********************************************************************/
void DisinfectionModulePowerOn(void)
{
   // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

/***********************************************************************
* DESCRIPTION: 关闭消毒模块电源
 *
 * RETURNS:
 *
***********************************************************************/
void DisinfectionModulePowerOff(void)
{
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

/***********************************************************************
* DESCRIPTION: 读取上位机电源状态
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t ReadPadPowerState(void)
{
    //return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
}

/***********************************************************************
* DESCRIPTION: ReadRk3399HeartState
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t ReadRk3399HeartState(void)
{
    return 0;
}

/***********************************************************************
* DESCRIPTION: 电机老化开关状态
 *
 * RETURNS:
 *
***********************************************************************/
UINT16 VeneerAgingTestState(void)
{
//	return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) + HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);
    return 0;
}

/***********************************************************************
* DESCRIPTION: Enable Music Power
 *
 * RETURNS:
 *
***********************************************************************/
void MusicPwEnable(void)
{
    HAL_GPIO_WritePin(MUSIC_ENABLE_PORT, MUSIC_ENABLE_PIN,GPIO_PIN_SET );
}

/***********************************************************************
* DESCRIPTION: Disable Music Power
 *
 * RETURNS:
 *
***********************************************************************/
void MusicPwDisable(void)
{
    HAL_GPIO_WritePin(MUSIC_ENABLE_PORT, MUSIC_ENABLE_PIN, GPIO_PIN_RESET);
}

/***********************************************************************
* DESCRIPTION: Enable EXT3.3V Power 低电平有效
 *
 * RETURNS:
 *
***********************************************************************/
void ExtVEnable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET );
}

/***********************************************************************
* DESCRIPTION: Disable EXT3.3 Power 高电平有效
 *
 * RETURNS:
 *
***********************************************************************/
void ExtVDisable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}

/***********************************************************************
* DESCRIPTION: Enable MUTE
 *
 * RETURNS:
 *
***********************************************************************/
void MuteEnable(void)
{
    HAL_GPIO_WritePin(MUSIC_MUTE_PORT, MUSIC_MUTE_PIN, GPIO_PIN_RESET );
}

/***********************************************************************
* DESCRIPTION: Disable MUTE
 *
 * RETURNS:
 *
***********************************************************************/
void MuteDisable(void)
{
    HAL_GPIO_WritePin(MUSIC_MUTE_PORT, MUSIC_MUTE_PIN, GPIO_PIN_SET );
}

/***********************************************************************
* DESCRIPTION: Enable LOGO PW
 *
 * RETURNS:
 *
***********************************************************************/
void LOGOEnable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET );
}

/***********************************************************************
* DESCRIPTION: Disable LOGO PW
 *
 * RETURNS:
 *
***********************************************************************/
void LOGODisable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET );
}

/***********************************************************************
* DESCRIPTION: Read Application Mode
 *
 * RETURNS:
 *
***********************************************************************/
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
