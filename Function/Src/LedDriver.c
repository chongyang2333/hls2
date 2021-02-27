/*******************************************************************
 *
 * FILE NAME:  LedDriver.c
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2019.3.23
 *
 * AUTHOR:      Liang
 *
 * History:   GuoDong   2019.4.20  Optimize IIC Write or Read.
------------------------------------------------------------------------
23-3-2019 Version 1.00 : Created by Liang
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/

#include "LedDriver.h"
#include "i2c.h"
#include "CanApp.h"
#include "PowerManager.h"
#include "systick.h"

#define LED_LEFT	     	0
#define LED_RIGHT      	1
#define LED_LEFT_RIGHT 	2

typedef struct
{
		UINT16 timeBaseMs;
		UINT16 timeNumber;
		UINT16 pwmStepCnt;
	  UINT8  stepUpDown;
		UINT8  pwmValueMax;
		UINT8  pwmValueMin;
		UINT8  pwmValueDiff;
		UINT8  curPwmValue;
	  float  pwmStepLen;
}LedBreathePwmStruct;


PRIVATE void I2C_LedDriver_ByteWrite(UINT8 WriteAddr, UINT8 WriteData);
PRIVATE void I2C_LedDriver_BufferRead(UINT8* pBuffer, UINT8 ReadAddr, UINT8 NumByteToRead);
PRIVATE void I2C_sTlc59108fAllRegRead(void);
PRIVATE void I2C_sTlc59108fAllRegUpDate(void);

PRIVATE void LedRGBPwmControl(UINT8 leftRight, UINT8 redPwm, UINT8 greenPwm, UINT8 bluePwm);
PRIVATE void LedRGBBlinkControl(UINT8 leftRight, UINT8 redPwm, UINT8 greenPwm, UINT8 bluePwm, UINT8 gprFreq);
PRIVATE void LedBreathePwmConfig(LedBreathePwmStruct* p, UINT16 timeBaseMs, UINT16 timeNumber, UINT8 pwmMax, UINT8 pwmMin, UINT8 stepUpDown);
PRIVATE UINT8 GetBreathePwmValue(LedBreathePwmStruct* p);

PRIVATE void LedFsmInit(LedFsmStruct *pLedFsm, LedFsmTableStruct *pLedFsmTable, UINT8 stuMaxNum, LedStateEnum curState);

static LedBreathePwmStruct sGreenBreathePwm = {0};
static LedBreathePwmStruct sRedBreathePwm   = {0};
static LedBreathePwmStruct sBlueBreathePwm  = {0};

static  LedFsmTableStruct sLedFsmTable[] = {  
    { LED_EVENT_START_FINISH,   EVENT_START_FINISH_CUR_STATE_MASK,    EVENT_START_FINISH_NEXT_STATE_MASK,  	NULL},
    { LED_EVENT_REMOTE_CONTROL, EVENT_REMOTE_CONTROL_CUR_STATE_MASK,  EVENT_REMOTE_CONTROL_NEXT_STATE_MASK, NULL},		
    { LED_EVENT_MCU_POWEROFF,   EVENT_MCU_POWEROFF_CUR_STATE_MASK,  	EVENT_MCU_POWEROFF_NEXT_STATE_MASK, 	NULL},
    { LED_EVENT_CHARGE_ING,     EVENT_CHARGE_ING_CUR_STATE_MASK,  		EVENT_CHARGE_ING_NEXT_STATE_MASK, 		NULL},
    { LED_EVENT_CHARGE_OUT,     EVENT_CHARGE_OUT_CUR_STATE_MASK,  		EVENT_CHARGE_OUT_NEXT_STATE_MASK, 		NULL},    
};

LedFsmStruct sLedFsm;

static t_Tlc59108fReg sTlc59108fReg={0};

static UINT8 TimeCnt = 0;
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void I2C_LedDriver_ByteWrite(UINT8 WriteAddr, UINT8 WriteData)
{
	// HAL_I2C_Mem_Write(&hi2c2, I2C2_LedDriverWrite, WriteAddr, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 1000);
	// while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void I2C_sTlc59108fAllRegUpDate(void)
{
	// HAL_I2C_Mem_Write(&hi2c2, I2C2_LedDriverWrite, MODE1 + 0x80, I2C_MEMADD_SIZE_8BIT, (UINT8*)&sTlc59108fReg,18,1000);
	// while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void I2C_LedDriver_BufferRead(UINT8* pBuffer, UINT8 ReadAddr, UINT8 NumByteToRead)
{
	// HAL_I2C_Mem_Read(&hi2c2, I2C2_LedDriverRead, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead,1000);
	// while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void I2C_sTlc59108fAllRegRead()
{
	// HAL_I2C_Mem_Read(&hi2c2, I2C2_LedDriverRead, MODE1 + 0x80, I2C_MEMADD_SIZE_8BIT, (UINT8*)&sTlc59108fReg, 18,1000);
	// while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void LedRGBPwmControl(UINT8 leftRight, UINT8 redPwm, UINT8 greenPwm, UINT8 bluePwm)
{
	if(leftRight == LED_LEFT)
	{
		sTlc59108fReg.RegLEDOUT0.bit.LDR0  = 2;
		sTlc59108fReg.RegLEDOUT0.bit.LDR1  = 2;
		sTlc59108fReg.RegLEDOUT0.bit.LDR2  = 2;
		sTlc59108fReg.RegPWM0 = redPwm;
		sTlc59108fReg.RegPWM1 = greenPwm;
		sTlc59108fReg.RegPWM2 = bluePwm;				
	}
	else if(leftRight == LED_RIGHT)
	{
		sTlc59108fReg.RegLEDOUT1.bit.LDR5  = 2;
		sTlc59108fReg.RegLEDOUT1.bit.LDR6  = 2;
		sTlc59108fReg.RegLEDOUT1.bit.LDR7  = 2;
		sTlc59108fReg.RegPWM5 = redPwm;		
		sTlc59108fReg.RegPWM6 = greenPwm;
		sTlc59108fReg.RegPWM7 = bluePwm;		
	}
	else if(leftRight == LED_LEFT_RIGHT)
	{
		sTlc59108fReg.RegLEDOUT0.bit.LDR0  = 2;
		sTlc59108fReg.RegLEDOUT0.bit.LDR1  = 2;
		sTlc59108fReg.RegLEDOUT0.bit.LDR2  = 2;
		sTlc59108fReg.RegPWM0 = redPwm;
		sTlc59108fReg.RegPWM1 = greenPwm;
		sTlc59108fReg.RegPWM2 = bluePwm;		
		sTlc59108fReg.RegLEDOUT1.bit.LDR5  = 2;
		sTlc59108fReg.RegLEDOUT1.bit.LDR6  = 2;
		sTlc59108fReg.RegLEDOUT1.bit.LDR7  = 2;
		sTlc59108fReg.RegPWM5 = redPwm;		
		sTlc59108fReg.RegPWM6 = greenPwm;
		sTlc59108fReg.RegPWM7 = bluePwm;
	}
	else
	{
		return;
	}
}

/***********************************************************************
 * DESCRIPTION: 
 *							 Blinking Period = (gprFreq + 1)/24 s;
 * RETURNS:
 *
***********************************************************************/
PRIVATE void LedRGBBlinkControl(UINT8 leftRight, UINT8 redPwm, UINT8 greenPwm, UINT8 bluePwm, UINT8 gprFreq)
{
	if(leftRight == LED_LEFT)
	{
		sTlc59108fReg.RegLEDOUT0.bit.LDR0  = 3;
		sTlc59108fReg.RegLEDOUT0.bit.LDR1  = 3;
		sTlc59108fReg.RegLEDOUT0.bit.LDR2  = 3;
		sTlc59108fReg.RegPWM0 = redPwm;
		sTlc59108fReg.RegPWM1 = greenPwm;
		sTlc59108fReg.RegPWM2 = bluePwm;
	}
  else if(leftRight == LED_RIGHT)
	{
		sTlc59108fReg.RegLEDOUT1.bit.LDR5  = 3;
		sTlc59108fReg.RegLEDOUT1.bit.LDR6  = 3;
		sTlc59108fReg.RegLEDOUT1.bit.LDR7  = 3;
		sTlc59108fReg.RegPWM5 = redPwm;
		sTlc59108fReg.RegPWM6 = greenPwm;
		sTlc59108fReg.RegPWM7 = bluePwm;
	}
	else if(leftRight == LED_LEFT_RIGHT)
	{
		sTlc59108fReg.RegLEDOUT0.bit.LDR0  = 3;
		sTlc59108fReg.RegLEDOUT0.bit.LDR1  = 3;
		sTlc59108fReg.RegLEDOUT0.bit.LDR2  = 3;
		sTlc59108fReg.RegPWM0 = redPwm;
		sTlc59108fReg.RegPWM1 = greenPwm;
		sTlc59108fReg.RegPWM2 = bluePwm;		
		sTlc59108fReg.RegLEDOUT1.bit.LDR5  = 3;
		sTlc59108fReg.RegLEDOUT1.bit.LDR6  = 3;
		sTlc59108fReg.RegLEDOUT1.bit.LDR7  = 3;
		sTlc59108fReg.RegPWM5 = redPwm;
		sTlc59108fReg.RegPWM6 = greenPwm;
		sTlc59108fReg.RegPWM7 = bluePwm;	
	}
	else
	{
		return;
	}

	sTlc59108fReg.RegMODE2.bit.DMBLNK  = 1;  			//Group control = blinking
	sTlc59108fReg.RegGRPFREQ           = gprFreq; //globalblinkingperiod = (sTlc59108fReg.RegGRPFREQ + 1)/24 s;
	sTlc59108fReg.RegGRPPWM            = 0X80;	

}
/***********************************************************************
 * DESCRIPTION:
 *							Breathe Period = timeBaseMs*timeNumber*2 ms
								stepUpDown: 0-Up  1-Down
 * RETURNS:
 *
***********************************************************************/
PRIVATE void LedBreathePwmConfig(LedBreathePwmStruct* p, UINT16 timeBaseMs, UINT16 timeNumber, UINT8 pwmMax, UINT8 pwmMin, UINT8 stepUpDown)
{
	p->timeBaseMs = timeBaseMs;
	p->timeNumber = timeNumber;
	p->pwmValueMax = pwmMax;
	p->pwmValueMin = pwmMin;
	p->pwmValueDiff = p->pwmValueMax - p->pwmValueMin;
	p->pwmStepLen = (p->pwmValueDiff * 1.0) / p->timeNumber;
	p->pwmStepCnt = 0;
	p->stepUpDown = stepUpDown;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 GetBreathePwmValue(LedBreathePwmStruct* p)
{	
	if(!p->stepUpDown)
	{
		p->curPwmValue = p->pwmValueMin + p->pwmStepCnt * p->pwmStepLen;
	}
	else
	{
		p->curPwmValue = p->pwmValueMax - p->pwmStepCnt * p->pwmStepLen;
	}
	
	p->pwmStepCnt++;
	
	if(p->pwmStepCnt > p->timeNumber)
	{
		p->stepUpDown = !p->stepUpDown;
		p->pwmStepCnt = 0;
	}

	return p->curPwmValue;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void LedFsmInit(LedFsmStruct *pLedFsm, LedFsmTableStruct *pLedFsmTable, UINT8 stuMaxNum, LedStateEnum curState)
{
	pLedFsm->sLedFsmTable = pLedFsmTable;
	pLedFsm->curState = curState;
	pLedFsm->stuMaxNum = stuMaxNum;
	pLedFsm->stateTransferFlag = 1;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void LedDriverInit(void)
{
//PWM0-------LR
//PWM1-------LG
//PWM2-------LB
//PWM3-------DO0
//PWM4-------DO1
//PWM5-------RR
//PWM6-------RG
//PWM7-------RB
//RegLEDOUT0(Left)    bit  7:6         5:4         3:2         1:0      LDRx[1:0] = 00 → OFF; 01 or 10 or 11 → ON;
//                         PWM3(DO0)   PWM2(LB)    PWM1(LG)    PWM0(LR)
//RegLEDOUT1(Right)   bit  7:6         5:4         3:2         1:0      LDRx[1:0] = 00 → OFF; 01 or 10 or 11 → ON;
//                         PWM7(RB)    PWM6(RG)    PWM5(RR)    PWM4(DO1)

	sTlc59108fReg.RegMODE1.bit.OSC     = 0;
	sTlc59108fReg.RegLEDOUT0.bit.LDR0  = 0;
	sTlc59108fReg.RegLEDOUT0.bit.LDR1  = 0;
	sTlc59108fReg.RegLEDOUT0.bit.LDR2  = 0;
	sTlc59108fReg.RegLEDOUT0.bit.LDR3  = 0;
	sTlc59108fReg.RegLEDOUT1.bit.LDR4  = 0;
	sTlc59108fReg.RegLEDOUT1.bit.LDR5  = 0;
	sTlc59108fReg.RegLEDOUT1.bit.LDR6  = 0;
	sTlc59108fReg.RegLEDOUT1.bit.LDR7  = 0;		
	I2C_sTlc59108fAllRegUpDate();
    
	
	LedFsmInit(&sLedFsm, sLedFsmTable, sizeof(sLedFsmTable)/sizeof(LedFsmTableStruct), LED_STATE_START);
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:

 *
***********************************************************************/
PUBLIC void LedPowerOn()
{
		//LedFsmInit(&sLedFsm, sLedFsmTable, sizeof(sLedFsmTable)/sizeof(LedFsmTableStruct), LED_STATE_START);
		sLedFsm.curState = LED_STATE_START;
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:

 *
***********************************************************************/
extern PUBLIC void LedPowerOff(void)
{
		LedFsmEventHandle(&sLedFsm, LED_EVENT_MCU_POWEROFF, NULL, 0);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:

 *
***********************************************************************/
PUBLIC void LedDriverExec(void)
{
	TimeCnt++;
	
	switch(sLedFsm.curState)
	{		
		case LED_STATE_CLOSE://close led				
			if(sLedFsm.stateTransferFlag)
			{			
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x00, 0x00);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}
			break;

		case LED_STATE_START:
			if(sLedFsm.stateTransferFlag)//left and right show light blue, it blinks twice within 500ms
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
				
				TimeCnt = 0;
			}
			
			else
			{
				if(TimeCnt == 5)
				{
					LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x00, 0x00);
					I2C_sTlc59108fAllRegUpDate();
				}
				
				if(TimeCnt == 10)
				{
					LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96);
					I2C_sTlc59108fAllRegUpDate();					
				}
				
				if(TimeCnt == 15)
				{
					LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x00, 0x00);
					I2C_sTlc59108fAllRegUpDate();
				}
			
				if(TimeCnt == 20)
				{
					LedFsmEventHandle(&sLedFsm, LED_EVENT_START_FINISH, LED_STATE_AWAIT, NULL);
				}
			}
			break;

		case LED_STATE_AWAIT://left and right show light blue		
			if(sLedFsm.stateTransferFlag)
			{	
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96);
				I2C_sTlc59108fAllRegUpDate();	
				
				sLedFsm.stateTransferFlag = 0;
			}
			break;			
		
		case LED_STATE_RUN_STRAIGHT://left and right show light blue				
			if(sLedFsm.stateTransferFlag)
			{	
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96);
				I2C_sTlc59108fAllRegUpDate();	

				sLedFsm.stateTransferFlag = 0;
			}
			break;
		
		case LED_STATE_RUN_TRUNL://left shows yellow with 500ms blinking period, right shows light blue  	
			if(sLedFsm.stateTransferFlag)
			{	
				LedRGBBlinkControl(LED_LEFT, 0xFF, 0x5A, 0x00, 11);
				LedRGBPwmControl(LED_RIGHT, 0x00, 0x32, 0x96);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}
			break;

		case LED_STATE_RUN_TRUNR://right shows yellow with 500ms blinking period, left shows light blue		
			if(sLedFsm.stateTransferFlag)
			{
				LedRGBBlinkControl(LED_RIGHT, 0xFF, 0x5A, 0x00, 11);
				LedRGBPwmControl(LED_LEFT, 0x00, 0x32, 0x96);
				I2C_sTlc59108fAllRegUpDate();		
	
				sLedFsm.stateTransferFlag = 0;	
			}
			break;
			
		case LED_STATE_DISPATCH://left and right show orange
			if(sLedFsm.stateTransferFlag)
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, 0xD2, 0x1E, 0x00);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}
			break;
		
		case LED_STATE_ARRIVE://left and right show light blue with 3s breathing period
			if(sLedFsm.stateTransferFlag)
			{
				LedBreathePwmConfig(&sGreenBreathePwm, 25, 60, 0x32, 0x00, 0);	
				LedBreathePwmConfig(&sBlueBreathePwm, 25, 60, 0x96, 0x00, 0);
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), GetBreathePwmValue(&sBlueBreathePwm));
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}	
			else
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), GetBreathePwmValue(&sBlueBreathePwm));
				I2C_sTlc59108fAllRegUpDate();
			}
			break;
			
		case LED_STATE_CHARGE_0_19://left and right show red with 7.5s breathing period
			if(sLedFsm.stateTransferFlag)
			{
				LedBreathePwmConfig(&sRedBreathePwm, 25, 150, 0xFF, 0x00, 0);
				LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm), 0x00, 0x00);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}	
			else
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm), 0x00, 0x00);
				I2C_sTlc59108fAllRegUpDate();
			}
			break;

		case LED_STATE_CHARGE_20_39://left and right show orange with 6s breathing period
			if(sLedFsm.stateTransferFlag)
			{
				LedBreathePwmConfig(&sRedBreathePwm, 25, 120, 0xD2, 0x00, 0);
				LedBreathePwmConfig(&sGreenBreathePwm, 25, 120, 0x0A, 0x00, 0);
				LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm), GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}	
			else
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm), GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
			}
			break;			

		case LED_STATE_CHARGE_40_59://left and right show yellow with 4.5s breathing period
			if(sLedFsm.stateTransferFlag)
			{
				LedBreathePwmConfig(&sRedBreathePwm, 25, 90, 0xD2, 0x00, 0);
				LedBreathePwmConfig(&sGreenBreathePwm, 25, 90, 0x32, 0x00, 0);
				LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm), GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}	
			else
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm), GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
			}
			break;
			
		case LED_STATE_CHARGE_60_79://left and right show green with 3s breathing period
			if(sLedFsm.stateTransferFlag)
			{
				LedBreathePwmConfig(&sGreenBreathePwm, 25, 60, 0xFF, 0x00, 0);
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}	
			else
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
			}
			break;
			
		case LED_STATE_CHARGE_80_99://left and right show light blue with 1.5s breathing period
			if(sLedFsm.stateTransferFlag)
			{
				LedBreathePwmConfig(&sGreenBreathePwm, 25, 30, 0x32, 0x00, 0);
				LedBreathePwmConfig(&sBlueBreathePwm, 25, 30, 0x96, 0x00, 0);
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), GetBreathePwmValue(&sBlueBreathePwm));
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}	
			else
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), GetBreathePwmValue(&sBlueBreathePwm));
				I2C_sTlc59108fAllRegUpDate();
			}
			break;
			
		case LED_STATE_CHARGE_ING://left and right show green with 3s breathing period
			if(sLedFsm.stateTransferFlag)
			{
				LedBreathePwmConfig(&sGreenBreathePwm, 25, 60, 0xFF, 0x00, 0);
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}	
			else
			{
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm), 0x00);
				I2C_sTlc59108fAllRegUpDate();
			}
			break;
						
		case LED_STATE_CHARGE_FINISH://left and right show light blue
			if(sLedFsm.stateTransferFlag)
			{	
				LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;
			}
			break;
		
		case LED_STATE_ERROR://left and right show red with 500ms blinking period	
			if(sLedFsm.stateTransferFlag)
			{	
				LedRGBBlinkControl(LED_LEFT_RIGHT, 0xFF, 0x00, 0x00, 11);
				I2C_sTlc59108fAllRegUpDate();
				
				sLedFsm.stateTransferFlag = 0;	
			}					
			break;
			
		default:			
			break;	
	}
}


extern uint8_t get_charge_state( void );

typedef struct{
    volatile int8_t r_color;
    volatile int8_t g_color;
    volatile int8_t b_color;
    volatile int8_t toggled;
    volatile uint8_t mode;
    volatile uint8_t dir;
    volatile uint32_t timer;
    volatile uint32_t arg;
} LED_BAR_PARAM;

LED_BAR_PARAM bar_param[2] =
{
    {.r_color = 0, .g_color = 0, .b_color = 0, .toggled = 1, .mode = 0, .dir = 0, .timer = 0, .arg = 0 },
    {.r_color = 0, .g_color = 0, .b_color = 0, .toggled = 1, .mode = 0, .dir = 0, .timer = 0, .arg = 0 }
};

void chassis_led_ctrl( uint8_t *buff )
{
    uint8_t lcolor = buff[2];
    uint8_t lmode = buff[3];
    uint32_t larg = buff[6] | (buff[5]<<8) | ( buff[4] << 16 );
    uint8_t bar_id = buff[1];
    uint8_t idx = 0;

    if( bar_id != 13 && bar_id != 14 )
    {
        return;
    }

    idx = bar_id - 13;
    if( ReadChargeAppState() )
    {
        return;
    }

    if( lcolor > 6 || lmode > 6 )
    {
        return;
    }

    if( lcolor == 1)
    {
        bar_param[idx].r_color = 0xFF;
        bar_param[idx].g_color = 0xFF;
        bar_param[idx].b_color = 0xFF;
    } else if( lcolor == 2 ) {
        bar_param[idx].r_color = 0xFF;
        bar_param[idx].g_color = 0;
        bar_param[idx].b_color = 0;
    } else if( lcolor == 3 ) {
        bar_param[idx].r_color = 0;
        bar_param[idx].g_color = 0xFF;
        bar_param[idx].b_color = 0;
    } else if( lcolor == 4 ) {
        bar_param[idx].r_color = 0;
        bar_param[idx].g_color = 0;
        bar_param[idx].b_color = 0xFF;
    } else if( lcolor == 5 ) {
        bar_param[idx].r_color = 0xFF;
        bar_param[idx].g_color = 0xA5;
        bar_param[idx].b_color = 0;
    } else if( lcolor == 6 ){
        bar_param[idx].r_color = 0xFF;
        bar_param[idx].g_color = 0xFF;
        bar_param[idx].b_color = 0;
    }
    bar_param[idx].mode = lmode;
    bar_param[idx].arg = larg / 40;

    if( bar_param[idx].mode == 5 )
    {
        bar_param[idx].r_color = buff[4];
        bar_param[idx].g_color = buff[5];
        bar_param[idx].b_color = buff[6];
        bar_param[idx].mode = 1;
    }
    if( bar_param[idx].mode == 1 || bar_param[idx].mode == 0 )
    {
        bar_param[idx].toggled = 0;
    }

}

void led_bar_driver( void )
{
    uint8_t bar = 0;
    
    bar_param[0].timer++;
    bar_param[1].timer++;
    
    
    if( bar_param[0].mode == 2)
    {
        if(( bar_param[0].timer % bar_param[0].arg ) == 0 && bar_param[0].dir == 0)
        {
            LedRGBPwmControl(LED_LEFT, bar_param[0].r_color, bar_param[0].g_color, bar_param[0].b_color);
            I2C_sTlc59108fAllRegUpDate();
            bar_param[0].dir = 1;
        } else if( ( bar_param[0].timer % bar_param[0].arg ) == 0 && bar_param[0].dir == 1 ) {
            LedRGBPwmControl(LED_LEFT, 0, 0, 0);
            I2C_sTlc59108fAllRegUpDate();
            bar_param[0].dir = 0;
        }
    } else if( bar_param[0].mode == 0 && bar_param[0].toggled == 0 ){
        LedRGBPwmControl(LED_LEFT, 0, 0, 0);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[0].toggled = 1;
    } else if ( bar_param[0].mode == 1 && bar_param[0].toggled == 0 ) {
        LedRGBPwmControl(LED_LEFT, bar_param[0].r_color, bar_param[0].g_color, bar_param[0].b_color);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[0].toggled = 1;
    }
    
    if( bar_param[1].mode == 2)
    {
        if(( bar_param[1].timer % bar_param[1].arg ) == 0 && bar_param[1].dir == 0)
        {
            LedRGBPwmControl(LED_RIGHT, bar_param[1].r_color, bar_param[1].g_color, bar_param[1].b_color);
            I2C_sTlc59108fAllRegUpDate();
            bar_param[1].dir = 1;
        } else if( ( bar_param[1].timer % bar_param[1].arg ) == 0 && bar_param[1].dir == 1 ) {
            LedRGBPwmControl(LED_RIGHT, 0, 0, 0);
            I2C_sTlc59108fAllRegUpDate();
            bar_param[1].dir = 0;
        }
    } else if( bar_param[1].mode == 0 && bar_param[1].toggled == 0 ){
        LedRGBPwmControl(LED_RIGHT, 0, 0, 0);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[1].toggled = 1;
    } else if ( bar_param[1].mode == 1 && bar_param[1].toggled == 0 ) {
        LedRGBPwmControl(LED_RIGHT, bar_param[1].r_color, bar_param[1].g_color, bar_param[1].b_color);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[1].toggled = 1;
    }
    
}


/***********************************************************************
 * DESCRIPTION: Handle LedFsm event
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void LedFsmEventHandle(LedFsmStruct *pLedFsm, UINT8 event, LedStateEnum aimState, void *parm)
{
	LedFsmTableStruct *pAcTable = pLedFsm->sLedFsmTable;
	void (*eventActFun)(void *) = NULL;
	LedStateEnum NextState;
	LedStateEnum CurState = pLedFsm->curState;
	UINT8 stateSwitchFlag = 0;
       	
	for (UINT8 i = 0; i < pLedFsm->stuMaxNum; i++) // ergodic table
	{
		if( (event == pAcTable[i].event) && ((pAcTable[i].curStateMask >> pLedFsm->curState)&0x0001) 
			&& ((pAcTable[i].nextStateMask >> aimState)&0x0001) )
		{
			if (pLedFsm->curState == aimState)
			{
					return ;
			}
			stateSwitchFlag = 1;				
			eventActFun = pAcTable[i].eventActFun;			
			NextState = aimState;	
			
			break;    
		}
	}
    
	if (stateSwitchFlag)
	{
		if (eventActFun != NULL)
		{
			eventActFun(parm);            //execute fuction
		}
		pLedFsm->curState = NextState;  //transfer state 
		pLedFsm->stateTransferFlag = 1; //transfer flag
	}
	else
	{
			// do nothing
	}
}

/***********************************************************************
 * DESCRIPTION:Audio Power Amplifier Mute off
 *
 * RETURNS:
 *
***********************************************************************/
void YDA138_MuteOff(void)
{
    I2C_LedDriver_BufferRead(&sTlc59108fReg.RegLEDOUT1.ALL,LEDOUT1,1);
    delay_ms(1);
    sTlc59108fReg.RegLEDOUT1.bit.LDR4 = 1;
    I2C_LedDriver_ByteWrite(LEDOUT1, sTlc59108fReg.RegLEDOUT1.ALL);
    delay_ms(1);
}

/***********************************************************************
 * DESCRIPTION:Audio Power Amplifier Mute on
 *
 * RETURNS:
 *
***********************************************************************/
void YDA138_MuteOn(void)
{
    I2C_LedDriver_BufferRead(&sTlc59108fReg.RegLEDOUT1.ALL,LEDOUT1,1);
    delay_ms(1);
    sTlc59108fReg.RegLEDOUT1.bit.LDR4 = 0;
    I2C_LedDriver_ByteWrite(LEDOUT1, sTlc59108fReg.RegLEDOUT1.ALL);
    delay_ms(1);
}

/***********************************************************************
 * DESCRIPTION:A2M7 Ctrl On
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void A2M7_CtrlOn(UINT8 Spd)
{
    I2C_LedDriver_BufferRead(&sTlc59108fReg.RegLEDOUT0.ALL, LEDOUT0, 1);
    delay_ms(1);
    sTlc59108fReg.RegLEDOUT0.bit.LDR3 = 2;
    I2C_LedDriver_ByteWrite(LEDOUT0, sTlc59108fReg.RegLEDOUT0.ALL);
    
    sTlc59108fReg.RegPWM3 = Spd;
    I2C_LedDriver_ByteWrite(PWM3, sTlc59108fReg.RegPWM3);
    delay_ms(1);
}


/***********************************************************************
 * DESCRIPTION:A2M7 Ctrl Off
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void A2M7_CtrlOff(void)
{
    I2C_LedDriver_BufferRead(&sTlc59108fReg.RegLEDOUT0.ALL,LEDOUT0,1);
    sTlc59108fReg.RegLEDOUT0.bit.LDR3 = 0;
    I2C_LedDriver_ByteWrite(LEDOUT0, sTlc59108fReg.RegLEDOUT0.ALL);
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT8 GetBatteryLevelForLed(UINT8 BatteryLevel)
{
		UINT8 ucRetVal;
		UINT8 ucBatteryLevel = BatteryLevel / 20;
		switch(ucBatteryLevel)
		{
				case 0:
					ucRetVal = LED_STATE_CHARGE_0_19;
					break;
				case 1:
					ucRetVal = LED_STATE_CHARGE_20_39;
					break;
				case 2:
					ucRetVal = LED_STATE_CHARGE_40_59;
					break;
				case 3:
					ucRetVal = LED_STATE_CHARGE_60_79;
					break;
				case 4:
					ucRetVal = LED_STATE_CHARGE_80_99;
					break;
				case 5:
					ucRetVal = LED_STATE_CHARGE_FINISH;
					break;
		}
		return ucRetVal;
}