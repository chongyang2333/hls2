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
    
    // pcb 将此引脚接到了12V
    gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_12);
    gpio_output_options_set(GPIOB,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,GPIO_PIN_12);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
    
     /*Configure GPIO pins : PE5 :DRVE_PW_EN(12V)*/
    gpio_mode_set(GPIOE,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_5);
    gpio_output_options_set(GPIOE,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);

    DrvPwDisable();
   
//     /*Configure GPIO pins : PC3 :MOTOR POWER ENABLE, PD10:MOTOR POWER BUFF ENABLE*/
    gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_10);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);
    VbusBufferDisable();
    
    gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_3);
    gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3);

    VbusDisable();
    
    /*Configure GPIO pins : PD4 :SYS_12V_EN (12V)*/
    gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_4);
    gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_4);
        
     /*Configure GPIO pins : PD9 :SYS_24V_EN (24V)*/
     gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_9);
     gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
     PadPowerOn();

//     /*Configure GPIO pins : PC2 : RST_ACS711 */
     gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_2);
     gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_2);
 
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    
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
     
//     /*Configure GPIO pins : PA6: CHARGE_FO */
       gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_6);
  
//      /*Configure GPIO pins : PB9 PB8: EEPROM SDA SCL */
       gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_8);
       gpio_output_options_set(GPIOB,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,GPIO_PIN_8);
       gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_9);
       gpio_output_options_set(GPIOB,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,GPIO_PIN_9);
    
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

//     /*Configure GPIO pins : PA12,PB3: KEY_IN_DET, CHARGE_IN_DET */
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_12); 
        gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_3);  
    
//     /*Configure GPIO pins : PA11 :WAKE_IO*/
       gpio_mode_set(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_11);
       gpio_output_options_set(GPIOA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
       McuPowerOn();
    
//     /*Configure GPIO pins : PB15 : MUSIC ENABLE*/
       gpio_mode_set(MUSIC_ENABLE_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,MUSIC_ENABLE_PIN);
       gpio_output_options_set(MUSIC_ENABLE_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,MUSIC_ENABLE_PIN);
       MusicPwDisable();

//     /*Configure GPIO pins : PB14 : MUTE*/
       gpio_mode_set(MUSIC_MUTE_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,MUSIC_MUTE_PIN);
       gpio_output_options_set(MUSIC_MUTE_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,MUSIC_MUTE_PIN);
       MuteEnable(); // 静音使能
       
       
//     /*Configure GPIO pins : PC15 : Tlc59108 RSTn*/
       gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_PIN_15);
       gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    
//     /*Configure GPIO pins : PD14:S1 PD15:S2：ApplicationMode*/
       gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_14);
       gpio_mode_set(GPIOD,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO_PIN_15);
       TmpChar = ReadApplicationMode();
 
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
    
// //    /*Configure GPIO pin : PF4 */
// //    GPIO_InitStruct.Pin = GPIO_PIN_4;
// //    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
// //    GPIO_InitStruct.Pull = GPIO_NOPULL;
// //    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

// //    /* EXTI interrupt init*/
// //    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);   
    
//     /*Configure GPIO pins : Pd11 :CHARGE_EN */
       gpio_mode_set(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_11);
       gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
       DisableCharge(TmpChar);
    
    return TmpChar;
}


void battery_I2C_GPIO_Init(void)
{  
    /* enable gpio clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* configure I2C GPIO */
    gpio_output_options_set(__I2C_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, __I2C_SCL_GPIO);
    gpio_mode_set(__I2C_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, __I2C_SCL_GPIO);
    
    gpio_output_options_set(__I2C_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, __I2C_SDA_GPIO);
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
    return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);    
}

// Enable DC Voltage Buffer
void VbusBufferEnable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);    
}

// Disable DC Voltage Buffer
void VbusBufferDisable(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

// turn on mcu power 
void McuPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

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
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//    }
}

void DisableCharge(UINT8 ApplicationMode)
{
    ChargeMosState = 0;
//    if (!ApplicationMode)
//    {
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//    }
//    else
//    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
//    }
}


PUBLIC UINT8 ReadChargeMosState(void)
{
    return ChargeMosState;
}


// Read Key State
//����:1 , �ɿ�:0
uint8_t ReadKeyInPinState(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
}

// Read Charge State
uint8_t ReadChargeInPinState(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
}

// turn on pad power 
void PadPowerOn(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
}

// turn off pad power 
void PadPowerOff(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
}

// turn on Disinfection Module power  没用
void DisinfectionModulePowerOn(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

// turn off Disinfection Module power  没用
void DisinfectionModulePowerOff(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

// Read pad power State
uint8_t ReadPadPowerState(void)
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
}

UINT16 VeneerAgingTestState(void)
{
//	return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) + HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);
}

// Enable Music Power 
void MusicPwEnable(void)
{
    HAL_GPIO_WritePin(MUSIC_ENABLE_PORT, MUSIC_ENABLE_PIN,GPIO_PIN_RESET );  
}

// Disable MUSIC Power 
void MusicPwDisable(void)
{
    HAL_GPIO_WritePin(MUSIC_ENABLE_PORT, MUSIC_ENABLE_PIN, GPIO_PIN_SET);  
}

// Enable MUTE
void MuteEnable(void)
{
    HAL_GPIO_WritePin(MUSIC_MUTE_PORT, MUSIC_MUTE_PIN, GPIO_PIN_SET );  
}

// Disable MUTE
void MuteDisable(void)
{
    HAL_GPIO_WritePin(MUSIC_MUTE_PORT, MUSIC_MUTE_PIN, GPIO_PIN_RESET );  
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
