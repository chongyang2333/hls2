/*******************************************************************
 *
 * FILE NAME:  Rgb.c
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

#include "Rgb.h"
#include "tim.h"
#include "Param.h"
#include "gd_hal.h"

PRIVATE UINT16 RgbMode = 0;

PRIVATE void RgbPwmUpdate(UINT8 PwmValue);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void RgbInit(void)
{
    /* Initialize RGB pwm module */
//    MX_TIM4_Init();
}


/***********************************************************************
 * DESCRIPTION: 40HZ
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void RgbExec(void)
{
    static UINT16 Cnt = 0;
    
    uint16_t max_cnt = 0;
    
    GPIO_PinState PinState = GPIO_PIN_SET;
    
    if (gParam[0].ErrorRegister0x230D || gParam[1].ErrorRegister0x230D)
    {
        max_cnt = 10; // 故障时快速闪烁
    }
    else
    {
        max_cnt = 50; // 正常呼吸 
    }
    
    Cnt++;
    Cnt = Cnt%(max_cnt*2);
    
    if(Cnt < max_cnt)
    {
        PinState = GPIO_PIN_SET;
    }
    else
    {
        PinState = GPIO_PIN_RESET;
    }
    
    //modify by hyr LED1 PE0
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, PinState); // STM32 HEART BEAT
    //modify by hyr 
//    if(gParam[0].ErrorRegister0x230D)
//    {
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, PinState);
//    }
//    else
//    {
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
//    }
//
//    if(gParam[1].ErrorRegister0x230D)
//    {
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, PinState);
//    }
//    else
//    {
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
//    }
    
   
    return;
    
    //led control
    switch(RgbMode)
    {
        case 0:  //led off
             RgbPwmUpdate(0);
        break;
        
        case 1:  //led on
            RgbPwmUpdate(50);
        break;
        
        case 2:  //led breathe
            {
                if(Cnt <= 50)
                    RgbPwmUpdate(Cnt+ 5);
                else if(Cnt > 50)
                    RgbPwmUpdate(100-Cnt+ 5);
            }
        break;
            
    }

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void RgbSetMode(UINT16 mode)
{
    RgbMode = mode;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void RgbPwmUpdate(UINT8 PwmValue)
{
    // TIM4->CCR1 = PwmValue;
	// TIM4->CCR2 = PwmValue;
	// TIM4->CCR3 = PwmValue;
}
