/*******************************************************************
 * 
 * FILE NAME:  HardApi.c
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

/*------------------------- Include files -------------------------*/
#include "HardApi.h"
#include "ControlRun.h"
#include "adc.h"
#include "tim.h"

PRIVATE INT16 ADC2_JDR1_Offset = 2160;
PRIVATE INT16 ADC2_JDR2_Offset = 2070;
PRIVATE INT16 ADC3_JDR1_Offset = 2090;
PRIVATE INT16 ADC3_JDR2_Offset = 2155;

PRIVATE INT16 ADC1_JDR4_Offset = 2048;

#define ADC2_JDR1_GAIN  0.01464844f    // (+/-)30A/2048
#define ADC2_JDR2_GAIN  0.01464844f    // (+/-)30A/2048
#define ADC3_JDR1_GAIN  0.01464844f    // (+/-)30A/2048
#define ADC3_JDR2_GAIN  0.01464844f    // (+/-)30A/2048

#define ADC1_JDR4_GAIN  0.01464844f    // (+/-)30A/2048

#define ADC1_JDR1_GAIN  0.01859225f    // Dc Voltage coff
#define ADC3_JDR4_GAIN  0.01859225f    // Charge Voltage coff

extern PUBLIC UINT8 ApplicationMode;
PRIVATE void AdcSumPort(uint16_t * sum);

/***********************************************************************
 * DESCRIPTION: Initialize time stamp timer.
 *        Can be used to calculate the execution time of a piece of code
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void TimeStampTimerInit(void)
{
    MX_TIM5_Init();
}

/***********************************************************************
 * DESCRIPTION: Read time stamp value
 *
 * RETURNS: Return current time stam value
 *
***********************************************************************/
PUBLIC UINT32 ReadTimeStampTimer(void)
{
//    return TIM5->CNT;
}

/***********************************************************************
 * DESCRIPTION: Initialize TIM1 and TIM8
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PwmInit(void)
{
    MX_TIM1_Init(); // Axis_Left PWM Init
    MX_TIM8_Init(); // Axis_Right PWM Init
    
    PwmDisable(AXIS_LEFT);
    PwmDisable(AXIS_RIGHT);
}

/***********************************************************************
 * DESCRIPTION: This function start TIM1 or TIM8 pwm
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PwmEnable(UINT16 AxisID)
{
    // if(AXIS_LEFT == AxisID)
    // {
    //     TIM1->CCER = 0x555;
    // }
    // else if(AXIS_RIGHT == AxisID)
    // {
    //     TIM8->CCER = 0x555;
    // }
}

/***********************************************************************
 * DESCRIPTION: This function stop TIM1 or TIM8 pwm
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PwmDisable(UINT16 AxisID)
{
    // if(AXIS_LEFT == AxisID)
    // {
    //     TIM1->CCER = 0x0;
    // }
    // else if(AXIS_RIGHT == AxisID)
    // {
    //     TIM8->CCER = 0x0;
    // }
}

/***********************************************************************
 * DESCRIPTION: This function update TIM1 or TIM8 compare value
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PwmUpdate(UINT16 AxisID, UINT16 PowerFlag, UINT16 Ta, UINT16 Tb, UINT16 Tc)
{
    if(POWER_OFF == PowerFlag)
    {
        Ta = Tb = Tc = 0;
    }
    
    // if(AXIS_LEFT == AxisID)
    // {
    //     TIM1->CCR1 = Ta;
    //     TIM1->CCR2 = Tb;
    //     TIM1->CCR3 = Tc;
    // }
    // else if(AXIS_RIGHT == AxisID)
    // {
    //     TIM8->CCR1 = Ta;
    //     TIM8->CCR2 = Tb;
    //     TIM8->CCR3 = Tc;
    // }
}

/***********************************************************************
 * DESCRIPTION: Initialize all adc channel
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AdcInit(void)
{
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
    
    delay_ms(50);
    AdcOffsetCal();  // todo
}

/***********************************************************************
 * DESCRIPTION: Start adc sample
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AdcSampleStart(void)
{
    // ADC3->CR2 |= (1<<22);
	// ADC2->CR2 |= (1<<22);
	// ADC1->CR2 |= (1<<22);  
}

/***********************************************************************
 * DESCRIPTION: wait for ADC to complete, then acknowledge flag
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AdcSampleClearFlag(void)
{
	// while(!(ADC3->SR&(1<<2)));
	// ADC3->SR&=~(1<<2);
	// while(!(ADC2->SR&(1<<2)));
	// ADC2->SR&=~(1<<2);
	// while(!(ADC1->SR&(1<<2)));
	// ADC1->SR&=~(1<<2);
    
//    LeftMotorAdc = ADC3->JDR3;
//    RightMotorAdc = ADC2->JDR3;
}

/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void GetMotorAdc(UINT16 *leftAdc, UINT16 *rightAdc)
{
    // *leftAdc = ADC3->JDR3;
    // *rightAdc = ADC2->JDR3;
}
  
/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void GetMosAdc(UINT16 *leftMosAdc, UINT16 *rightMosAdc)
{
    // *leftMosAdc = ADC1->JDR2;
    // *rightMosAdc = ADC1->JDR3;
}

/***********************************************************************
 * DESCRIPTION: 
 *	     
* RETURNS: ADC通道和接口，服务AdcOffsetCal函数
 *
***********************************************************************/
PRIVATE void AdcSumPort(uint16_t * sum)
{
//        sum[0] += ADC2->JDR1;
//        sum[1] += ADC2->JDR2;
//        sum[2] += ADC3->JDR1;
//        sum[3] += ADC3->JDR2;
}
/***********************************************************************
 * DESCRIPTION: 
 *	     
 * RETURNS:
 *
***********************************************************************/
PRIVATE INT16 Left_AdcInitState = 0;
PRIVATE INT16 Right_AdcInitState = 0;
PUBLIC void AdcOffsetCal(void)
{
    UINT16 sum[6]={0};
    
    for(int i=0;i<16;i++)
    {
        AdcSampleStart();
        AdcSampleClearFlag();
        
        AdcSumPort(sum);
        
        delay_ms(2);
    }
    
    ADC2_JDR1_Offset = sum[0]/16;
    ADC2_JDR2_Offset = sum[1]/16;
    
    if((ADC2_JDR1_Offset > 2248) || (ADC2_JDR1_Offset <1848)
        || (ADC2_JDR2_Offset > 2248) || (ADC2_JDR2_Offset <1848))
    {
        Left_AdcInitState = 1;
    }
    
    ADC3_JDR1_Offset = sum[2]/16;
    ADC3_JDR2_Offset = sum[3]/16;
     
    if((ADC3_JDR1_Offset > 2248) || (ADC3_JDR1_Offset <1848)
        || (ADC3_JDR2_Offset > 2248) || (ADC3_JDR2_Offset <1848))
    {
        Right_AdcInitState = 1;
    }
    
//    ADC2_JDR4_Offset = sum[4]/16;
//    ADC1_JDR4_Offset = sum[5]/16;
    
}

/***********************************************************************
 * DESCRIPTION: 
 *	     
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT16 GetAdcInitState(UINT16 AxisID)
{
    if(AXIS_LEFT == AxisID)
    {
        return Left_AdcInitState;
    }
    else if(AXIS_RIGHT == AxisID)
    {
        return Right_AdcInitState;
    }
    
    return 0;
}

/***********************************************************************
 * DESCRIPTION: Get phase current
 *	     
 * RETURNS: Return Ia, Ib
 *
***********************************************************************/
PUBLIC void GetPhaseCurrent(UINT16 AxisID, float *Ia, float *Ib)
{ 
    if(AXIS_LEFT == AxisID)
    {
        // *Ia = ((INT16)ADC3->JDR1 - ADC3_JDR1_Offset)*ADC3_JDR1_GAIN;
        // *Ib = ((INT16)ADC3->JDR2 - ADC3_JDR2_Offset)*ADC3_JDR2_GAIN;
    }
    else if(AXIS_RIGHT == AxisID)
    { 
        // *Ia = ((INT16)ADC2->JDR1 - ADC2_JDR1_Offset)*ADC2_JDR1_GAIN;
        // *Ib = ((INT16)ADC2->JDR2 - ADC2_JDR2_Offset)*ADC2_JDR2_GAIN;
    }
    
}

/***********************************************************************
 * DESCRIPTION: Get Charge current
 *	     
 * RETURNS: 
 *
***********************************************************************/
PUBLIC float GetChargeCurrent(void)
{ 
    float Res = 0;
    // Res = ((INT16)ADC1->JDR4 - ADC1_JDR4_Offset)*ADC1_JDR4_GAIN;
    return Res;
}

/***********************************************************************
 * DESCRIPTION: Get battery current
 *	     
 * RETURNS: 
 *
***********************************************************************/
PUBLIC float GetBatteryCurrent(void)
{ 
    return 0.0f;
}

/***********************************************************************
 * DESCRIPTION: Get battery Voltage
 *	     
 * RETURNS: 
 *
***********************************************************************/
PUBLIC float GetBatteryVoltage(void)
{ 
    float Vbattery_tmp; 
   
    // Vbattery_tmp = ADC1->JDR1*ADC1_JDR1_GAIN;
    return Vbattery_tmp;
}

/***********************************************************************
 * DESCRIPTION: Get MT bus voltage
 *	     
 * RETURNS: Return Vbus
 *
***********************************************************************/
PRIVATE REAL32 Vbus_local_voltage = 0.0f;
PUBLIC void GetDcVoltage(float *Vbus)
{ 
    float Vbus_tmp = 0.0f;
    
    if (!ApplicationMode)
    {
        /*<= B04, Use ADC1->JDR1, PA4*/
        // Vbus_tmp = ADC1->JDR1*ADC1_JDR1_GAIN;
    }
    else
    {
        /*>= A06, Use ADC2->JDR4, PA3*/
        // Vbus_tmp = ADC2->JDR4*ADC2_JDR4_GAIN;
    }
    
    Vbus_local_voltage = Vbus_tmp;
    UTILS_LP_FAST(*Vbus, Vbus_tmp, 0.01f);
}

/***********************************************************************
 * DESCRIPTION: Get MT bus voltage No filter
 *	     
 * RETURNS: Return Vbus
 *
***********************************************************************/
PUBLIC REAL32 GetDcVoltageNoFilter(void)
{ 
    return Vbus_local_voltage;
}

/***********************************************************************
 * DESCRIPTION: Get Charge voltage
 *	     
 * RETURNS: Return 
 *
***********************************************************************/
PUBLIC float GetChargeVoltage(void)
{ 
    float Vtmp;
    // Vtmp = ADC3->JDR4*ADC3_JDR4_GAIN;
    
    return Vtmp;
}

/***********************************************************************
 * DESCRIPTION: 
 *	     
 * RETURNS:
 *
***********************************************************************/
PUBLIC void IncEncoderInit(void)
{


}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT32 GetIncEncoderPulse(UINT16 AxisID)
{
    UINT32 res = 0;
    if(AXIS_LEFT == AxisID)
    {
        // res = TIM2->CNT;
    }
    else if(AXIS_RIGHT == AxisID)
    {
        // res = TIM4->CNT;
    }
    
    return res;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ClearIncEncoderPulse(UINT16 AxisID)
{
    if(AXIS_LEFT == AxisID)
    {
        // TIM3->CNT = 0;
    }
    else if(AXIS_RIGHT == AxisID)
    {
        // TIM2->CNT = 0;
    }
}

/***********************************************************************
 * DESCRIPTION:Get hall state.
 *
 * RETURNS:
 *
***********************************************************************/
const UINT16 bldc_hall_tab[20][8]={ {0x7F, 0, 0, 0, 0, 0, 0, 0xFF},
                                    {0x7F, 6, 4, 5, 2, 1, 3, 0xFF}, //ZL 5.5" 1024PRD(ZLLG55ASM150-PD), gMachineInfo.motorVersion = 1
                                    {0x7F, 4, 6, 5, 2, 3, 1, 0xFF}, //WeiYi 5.5" 1024PRD, gMachineInfo.motorVersion = 2
                                    {0x7F, 2, 6, 1, 4, 3, 5, 0xFF}, //ZL 6.5" 1024PRD(ZLLG65ASM250-L), gMachineInfo.motorVersion = 3
                                    {0x7F, 0, 0, 0, 0, 0, 0, 0xFF}, //ZL 6.5" 2048PRD(), gMachineInfo.motorVersion = 4
                                    {0x7F, 0, 0, 0, 0, 0, 0, 0xFF}, //ZL 6.5" 4096PRD(), gMachineInfo.motorVersion = 5
                                    {0x7F, 6, 2, 1, 4, 5, 3, 0xFF}, //DongXingChang.Tech 6.5" 1024PRD(FDK10529B024-1), gMachineInfo.motorVersion = 6
                                    {0x7F, 2, 4, 3, 6, 1, 5, 0xFF}  //YaTeng.Tech 5.5" 1024PRD), gMachineInfo.motorVersion = 7
                                  };
PUBLIC UINT16 GetHallState(UINT16 AxisID, UINT32 MotorVersion)
{
	UINT16 hall_val=0;
    UINT16 hallA,hallB,hallC;
 
    if(AXIS_LEFT == AxisID)
    {
        hallA = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9);
        hallB = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7);
        hallC = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8);
    }
    else if(AXIS_RIGHT == AxisID)
    {
        hallA = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
        hallB = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
        hallC = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
    }

	if(hallA==1)
		hall_val|=4;
	if(hallB==1)
		hall_val|=1;
	if(hallC==1)
		hall_val|=2;
    
    return bldc_hall_tab[MotorVersion][hall_val];

}

/***********************************************************************
 * DESCRIPTION:Get hardware overcurrent state.
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT16 GetHardOverCurState(UINT16 AxisID)
{
    UINT16 res = !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10) 
               + !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13) 
               + !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14)
               + !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1) 
               + !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
    
    return res;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ResetACS711(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
    delay_us(300);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
}
