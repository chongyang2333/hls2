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
#include "gd_hal.h"
#include "delay.h"

PRIVATE INT16 ADC2_JDR0_Offset = 2160;
PRIVATE INT16 ADC2_JDR1_Offset = 2070;
PRIVATE INT16 ADC1_JDR0_Offset = 2090;
PRIVATE INT16 ADC1_JDR1_Offset = 2155;

PRIVATE INT16 ADC0_JDR3_Offset = 2048;

#define ADC2_JDR0_GAIN  0.016833f//0.01464844f    // (+/-)34.5A/2048
#define ADC2_JDR1_GAIN  0.016833f//0.01464844f    // (+/-)34.5A/2048
#define ADC1_JDR0_GAIN  0.016833f//0.01464844f    // (+/-)34.5A/2048
#define ADC1_JDR1_GAIN  0.016833f//0.01464844f    // (+/-)34.5A/2048


#define ADC0_JDR0_GAIN1  0.02306f    // Dc Voltage coff MT_BUS
#define ADC0_JDR1_GAIN  0.02306f    // Dc Voltage coff MAIN_V/BATTERY_V
#define ADC0_JDR2_GAIN  0.02306f    // Dc Voltage coff CHARGE_V
#define ADC0_JDR3_GAIN  0.02306f    //0.01592221f    // (+/-)30A/2048   CHARGE_I


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
    MX_TIM1_Init();
}

/***********************************************************************
 * DESCRIPTION: Read time stamp value
 *
 * RETURNS: Return current time stam value
 *
***********************************************************************/
uint32_t TEST_V32 = 0;
PUBLIC UINT32 ReadTimeStampTimer(void)
{
    TEST_V32 = TIMER_CNT(TIMER1);
    return TIMER_CNT(TIMER1);
	
	//return TIMER1->TIMER_CNT;
}

/***********************************************************************
 * DESCRIPTION: Initialize TIM1 and TIM8
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PwmInit(void)
{
    MX_TIM0_Init(); // Axis_Left PWM Init
    MX_TIM7_Init(); // Axis_Right PWM Init
    
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
    if(AXIS_LEFT == AxisID)
     {
        TIMER_CHCTL2(TIMER0) = 0x555;
     }
     else if(AXIS_RIGHT == AxisID)
     {
        TIMER_CHCTL2(TIMER7) = 0x555;
     }
}

/***********************************************************************
 * DESCRIPTION: This function stop TIM1 or TIM8 pwm
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PwmDisable(UINT16 AxisID)
{
     if(AXIS_LEFT == AxisID)
     {
        TIMER_CHCTL2(TIMER0) = 0;
    }
    else if(AXIS_RIGHT == AxisID)
    {
         TIMER_CHCTL2(TIMER7) = 0;
    }
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
    
     if(AXIS_LEFT == AxisID)
     {
				TIMER_CH0CV(TIMER0) = Ta;
				TIMER_CH1CV(TIMER0) = Tb;
				TIMER_CH2CV(TIMER0) = Tc;
     }
     else if(AXIS_RIGHT == AxisID)
     {
                TIMER_CH0CV(TIMER7) = Ta;
                TIMER_CH1CV(TIMER7) = Tb;
                TIMER_CH2CV(TIMER7) = Tc;
    }
}

/***********************************************************************
 * DESCRIPTION: Initialize all adc channel
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AdcInit(void)
{
    gpio_adc_config();
	adc_config();
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
	adc_software_trigger_enable(ADC0,ADC_INSERTED_CHANNEL);
	adc_software_trigger_enable(ADC1,ADC_INSERTED_CHANNEL);
	adc_software_trigger_enable(ADC2,ADC_INSERTED_CHANNEL);
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
	
	if((ADC_STAT(ADC0) & ADC_STAT_STIC))
			ADC_STAT(ADC0)&=(~ADC_STAT_STIC);
	if((ADC_STAT(ADC1) & ADC_STAT_STIC))
		ADC_STAT(ADC1)&=(~ADC_STAT_STIC);
	if((ADC_STAT(ADC2) & ADC_STAT_STIC))
		ADC_STAT(ADC2)&=(~ADC_STAT_STIC);
}

/***********************************************************************
 * DESCRIPTION:  get motor temprature
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void GetMotorAdc(UINT16 *leftAdc, UINT16 *rightAdc)
{
     *leftAdc = ADC_IDATA2(ADC2);  //ADC_IDATA1
     *rightAdc =ADC_IDATA2(ADC1);
}
  
/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void GetMosAdc(UINT16 *leftMosAdc, UINT16 *rightMosAdc)
{
     //*leftMosAdc = ADC1->JDR2;
     //*rightMosAdc = ADC1->JDR3;
	
	*leftMosAdc = 3828; //没有ADC，给个假值
	*rightMosAdc = 3828;
}

/***********************************************************************
 * DESCRIPTION: 
 *	     
* RETURNS: ADC通道和接口，服务AdcOffsetCal函数
 *
***********************************************************************/
PRIVATE void AdcSumPort(uint16_t * sum)
{
        sum[0] += ADC_IDATA0(ADC2);
        sum[1] += ADC_IDATA1(ADC2);
        sum[2] += ADC_IDATA0(ADC1);
        sum[3] += ADC_IDATA1(ADC1);
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
        delay_ms(1);
        AdcSumPort(sum);
        
        delay_ms(1);
    }
    
    ADC2_JDR0_Offset = sum[0]/16;
    ADC2_JDR1_Offset = sum[1]/16;
    
    if((ADC2_JDR0_Offset > 2248) || (ADC2_JDR0_Offset <1848)
        || (ADC2_JDR1_Offset > 2248) || (ADC2_JDR1_Offset <1848))
    {
        Left_AdcInitState = 1;
    }
    
    ADC1_JDR0_Offset = sum[2]/16;
    ADC1_JDR1_Offset = sum[3]/16;
     
    if((ADC1_JDR0_Offset > 2248) || (ADC1_JDR0_Offset <1848)
        || (ADC1_JDR1_Offset > 2248) || (ADC1_JDR1_Offset <1848))
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
    if(AXIS_LEFT == AxisID) //ADC_IDATA3(ADC2);
    {
         *Ia = -((INT16)ADC_IDATA0(ADC2) - ADC2_JDR0_Offset)*ADC2_JDR0_GAIN;
         *Ib = -((INT16)ADC_IDATA1(ADC2) - ADC2_JDR1_Offset)*ADC2_JDR1_GAIN;
    }
    else if(AXIS_RIGHT == AxisID)
    { 
         *Ia = -((INT16)ADC_IDATA0(ADC1) - ADC1_JDR0_Offset)*ADC1_JDR0_GAIN;
         *Ib = -((INT16)ADC_IDATA1(ADC1) - ADC1_JDR1_Offset)*ADC1_JDR1_GAIN;
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
     Res = ((INT16)ADC_IDATA3(ADC0) - ADC0_JDR3_Offset)*ADC0_JDR3_GAIN;
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
   
     Vbattery_tmp = ADC_IDATA1(ADC0)*ADC0_JDR1_GAIN;
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
    
//    if (!ApplicationMode)
//    {
//        /*<= B04, Use ADC1->JDR1, PA4*/
//         Vbus_tmp = ADC_IDATA0(ADC0)*ADC0_JDR0_GAIN;
//    }
//    else
//    {
        /*>= A06, Use ADC2->JDR4, PA3*/
         Vbus_tmp = ADC_IDATA0(ADC0)*ADC0_JDR0_GAIN1;
//    }
    
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
     Vtmp = ADC_IDATA2(ADC0)*ADC0_JDR2_GAIN;
    
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
         res = TIMER_CNT(TIMER3);
    }
    else if(AXIS_RIGHT == AxisID)
    {
         res = TIMER_CNT(TIMER2);
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
         TIMER_CNT(TIMER3) = 0;
    }
    else if(AXIS_RIGHT == AxisID)
    {
         TIMER_CNT(TIMER2) = 0;
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
                                    {0x7F, 6, 4, 5, 2, 1, 3, 0xFF}, //ZL 5.5" 4096PRD, gMachineInfo.motorVersion = 2
                                    {0x7F, 2, 6, 1, 4, 3, 5, 0xFF}, //ZL 6.5" 1024PRD(ZLLG65ASM250-L), gMachineInfo.motorVersion = 3
                                    {0x7F, 0, 0, 0, 0, 0, 0, 0xFF}, //ZL 6.5" 2048PRD(), gMachineInfo.motorVersion = 4
                                    {0x7F, 0, 0, 0, 0, 0, 0, 0xFF}, //ZL 6.5" 4096PRD(), gMachineInfo.motorVersion = 5
                                    {0x7F, 6, 2, 1, 4, 5, 3, 0xFF}, //DongXingChang.Tech 6.5" 1024PRD(FDK10529B024-1), gMachineInfo.motorVersion = 6
                                    {0x7F, 2, 4, 3, 6, 1, 5, 0xFF},  //YaTeng.Tech 5.5" 4096PRD), gMachineInfo.motorVersion = 7
                                  	{0x7F, 4, 2, 3, 6, 5, 1, 0xFF} // YaTeng 6.5 4096 gMachineInfo.motorVersion = 8 add by hzy
																	};
PUBLIC UINT16 GetHallState(UINT16 AxisID, UINT32 MotorVersion)
{
	UINT16 hall_val=0;
    UINT16 hallA,hallB,hallC;
 
    if(AXIS_LEFT == AxisID)
    {
        hallA = gpio_input_bit_get(GPIOD, GPIO_PIN_7);
        hallB = gpio_input_bit_get(GPIOB, GPIO_PIN_6);
        hallC = gpio_input_bit_get(GPIOB, GPIO_PIN_7);
    }
    else if(AXIS_RIGHT == AxisID)
    {
        hallA = gpio_input_bit_get(GPIOC, GPIO_PIN_12);
        hallB = gpio_input_bit_get(GPIOC, GPIO_PIN_10);
        hallC = gpio_input_bit_get(GPIOC, GPIO_PIN_11);
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
    UINT16 res = !gpio_input_bit_get(GPIOD, GPIO_PIN_2) 
               + !gpio_input_bit_get(GPIOB, GPIO_PIN_2) 
               + !gpio_input_bit_get(GPIOE, GPIO_PIN_7)
               + !gpio_input_bit_get(GPIOE, GPIO_PIN_14) 
               + !gpio_input_bit_get(GPIOE, GPIO_PIN_15);
    
    return res;
}

PUBLIC UINT16 GetIbusOverCurState(UINT16 AxisID)
{
		UINT16 res = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
    
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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    delay_us(300);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
}

/***********************************************************************
 * DESCRIPTION: Start adc sample
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT16 ReadPwmoutIOState(UINT16 AxisID)
{
    if (AXIS_LEFT == AxisID)
    {
        return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    }
    else  if (AXIS_RIGHT == AxisID)
    {
        return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    }
}
