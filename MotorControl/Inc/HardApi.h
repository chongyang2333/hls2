/*******************************************************************
 * 
 * FILE NAME:  HardApi.h
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

#ifndef _HARDAPI_H_
#define _HARDAPI_H_

#include "UserDataTypes.h"

#define ADC2_JDR4_GAIN  0.01859225f    // Motor Bus Voltage coff

/* Time stamp module */
extern PUBLIC void TimeStampTimerInit(void);
extern PUBLIC UINT32 ReadTimeStampTimer(void);

/* PWM module */
extern PUBLIC void PwmInit(void);
extern PUBLIC void PwmEnable(UINT16 AxisID);
extern PUBLIC void PwmDisable(UINT16 AxisID);
extern PUBLIC void PwmUpdate(UINT16 AxisID, UINT16 PowerFlag, UINT16 Ta, UINT16 Tb, UINT16 Tc);

/* ADC module */
extern PUBLIC void AdcInit(void);
extern PUBLIC void AdcOffsetCal(void);
extern PUBLIC UINT16 GetAdcInitState(UINT16 AxisID);
extern PUBLIC void AdcSampleStart(void);
extern PUBLIC void AdcSampleClearFlag(void);
extern PUBLIC void GetMotorAdc(UINT16 *leftAdc, UINT16 *rightAdc);
extern PUBLIC void GetMosAdc(UINT16 *leftMosAdc, UINT16 *rightMosAdc);
extern PUBLIC void GetPhaseCurrent(UINT16 AxisID, float *Ia, float *Ib);
extern PUBLIC float GetChargeCurrent(void);
extern PUBLIC float GetBatteryCurrent(void);
extern PUBLIC float GetBatteryVoltage(void);
extern PUBLIC void GetDcVoltage(float *Vbus);
PUBLIC REAL32 GetDcVoltageNoFilter(void);
extern PUBLIC float GetChargeVoltage(void);

/* Encoder module */
extern PUBLIC void IncEncoderInit(void);
extern PUBLIC UINT32 GetIncEncoderPulse(UINT16 AxisID);
extern PUBLIC void ClearIncEncoderPulse(UINT16 AxisID);
extern PUBLIC UINT16 GetHallState(UINT16 AxisID, UINT32 MotorVersion);

PUBLIC UINT16 GetHardOverCurState(UINT16 AxisID);
PUBLIC void ResetACS711(void);

#endif  // _HARDAPI_H_
