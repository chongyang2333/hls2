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
    GPIO_PinState PinState = GPIO_PIN_SET;

    Cnt++;
    Cnt = Cnt%100;

    if(Cnt < 50)
    {
        PinState = GPIO_PIN_SET;
    }
    else
    {
        PinState = GPIO_PIN_RESET;
    }


    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, PinState); // STM32 HEART BEAT

    if(gParam[0].ErrorRegister0x230D)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, PinState);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
    }

    if(gParam[1].ErrorRegister0x230D)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, PinState);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    }


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
