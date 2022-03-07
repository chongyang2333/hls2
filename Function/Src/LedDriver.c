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
#include "delay.h"
#include "CanApp.h"
#include "PowerManager.h"
#include "gpio.h"
#include "delay.h"

#define LED_LEFT	        0
#define LED_RIGHT      	    1
#define LED_BACK            2
#define LED_LEFT_RIGHT 	    3
#define LED_LEFT_RIGHT_BACK 4

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
} LedBreathePwmStruct;


PRIVATE void I2C_LedDriver_ByteWrite(UINT8 WriteAddr, UINT8 WriteData);
PRIVATE void I2C_LedDriver_BufferRead(UINT8* pBuffer, UINT8 ReadAddr, UINT8 NumByteToRead);
PRIVATE void I2C_sTlc59108fAllRegRead(void);
PRIVATE void I2C_sTlc59108fAllRegUpDate(void);

PRIVATE void LedRGBPwmControl(UINT8 leftRight, UINT8 redPwm, UINT8 greenPwm, UINT8 bluePwm);
PRIVATE void LedRGBBlinkControl(UINT8 leftRight, UINT8 redPwm, UINT8 greenPwm, UINT8 bluePwm, UINT8 gprFreq);
PRIVATE void LedBreathePwmConfig(LedBreathePwmStruct* p, UINT16 timeBaseMs, UINT16 timeNumber, UINT8 pwmMax, UINT8 pwmMin, UINT8 stepUpDown);
PRIVATE UINT8 GetBreathePwmValue(LedBreathePwmStruct* p);
PRIVATE void chassis_led_color_extract(uint8_t bar_temp,uint8_t *buff,uint8_t r_color,uint8_t g_color,uint8_t b_color);
PRIVATE void LedFsmInit(LedFsmStruct *pLedFsm, LedFsmTableStruct *pLedFsmTable, UINT8 stuMaxNum, LedStateEnum curState);

static LedBreathePwmStruct sGreenBreathePwm[3] = {0,0,0};
static LedBreathePwmStruct sRedBreathePwm[3]   = {0,0,0};
static LedBreathePwmStruct sBlueBreathePwm[3]  = {0,0,0};

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
    i2c_mem_write(I2C1, I2C2_LedDriverWrite, WriteAddr, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 5000);
}

PRIVATE void I2C_LedDriver_ByteWrite_Weak(UINT8 WriteAddr, UINT8 WriteData)
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
    i2c_mem_write(I2C1, I2C2_LedDriverWrite, MODE1 + 0x80, I2C_MEMADD_SIZE_8BIT, (UINT8*)&sTlc59108fReg, 18, 5000);
}

PRIVATE void I2C_sTlc59108fAllRegUpDate_Weak(void)
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
    i2c_mem_read(I2C1, I2C2_LedDriverRead, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead, 5000);
}

PRIVATE void I2C_LedDriver_BufferRead_Weak(UINT8* pBuffer, UINT8 ReadAddr, UINT8 NumByteToRead)
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
PRIVATE void I2C_sTlc59108fAllRegRead(void)
{
    i2c_mem_read(I2C1, I2C2_LedDriverRead, MODE1 + 0x80, I2C_MEMADD_SIZE_8BIT, (UINT8*)&sTlc59108fReg, 18, 5000);
}

PRIVATE void I2C_sTlc59108fAllRegRead_Weak(void)
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
    switch(leftRight)
    {
    case LED_LEFT:
    {
        sTlc59108fReg.RegLEDOUT0.bit.LDR0  = 2;
        sTlc59108fReg.RegLEDOUT0.bit.LDR1  = 2;
        sTlc59108fReg.RegLEDOUT0.bit.LDR2  = 2;
        sTlc59108fReg.RegPWM0 = redPwm;
        sTlc59108fReg.RegPWM1 = greenPwm;
        sTlc59108fReg.RegPWM2 = bluePwm;
    }break;
    case LED_RIGHT:
    {
        sTlc59108fReg.RegLEDOUT1.bit.LDR5  = 2;
        sTlc59108fReg.RegLEDOUT1.bit.LDR6  = 2;
        sTlc59108fReg.RegLEDOUT1.bit.LDR7  = 2;
        sTlc59108fReg.RegPWM5 = redPwm;
        sTlc59108fReg.RegPWM6 = greenPwm;
        sTlc59108fReg.RegPWM7 = bluePwm;
    }break;
    case LED_BACK :
    {
        sTlc59108fReg.RegLEDOUT1.bit.LDR4  = 2;
        sTlc59108fReg.RegPWM4 = 0xff - bluePwm;
    }break;
    case LED_LEFT_RIGHT :
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

    }break;
    case LED_LEFT_RIGHT_BACK:
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
        sTlc59108fReg.RegLEDOUT1.bit.LDR4  = 2;
        sTlc59108fReg.RegPWM4 = 0xff -  bluePwm;
    }break;
    default:
    {
        return;
    }break;
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
    switch(leftRight)
    {
    case LED_LEFT:
    {
        sTlc59108fReg.RegLEDOUT0.bit.LDR0  = 3;
        sTlc59108fReg.RegLEDOUT0.bit.LDR1  = 3;
        sTlc59108fReg.RegLEDOUT0.bit.LDR2  = 3;
        sTlc59108fReg.RegPWM0 = redPwm;
        sTlc59108fReg.RegPWM1 = greenPwm;
        sTlc59108fReg.RegPWM2 = bluePwm;
    }break;
    case LED_RIGHT:
    {
        sTlc59108fReg.RegLEDOUT1.bit.LDR5  = 3;
        sTlc59108fReg.RegLEDOUT1.bit.LDR6  = 3;
        sTlc59108fReg.RegLEDOUT1.bit.LDR7  = 3;
        sTlc59108fReg.RegPWM5 = redPwm;
        sTlc59108fReg.RegPWM6 = greenPwm;
        sTlc59108fReg.RegPWM7 = bluePwm;
    }break;
    case LED_LEFT_RIGHT :
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

    }break;
    case LED_BACK :
    {
        sTlc59108fReg.RegLEDOUT1.bit.LDR4  = 3;
        sTlc59108fReg.RegPWM4 = bluePwm;
    }break;
    case LED_LEFT_RIGHT_BACK:
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
        sTlc59108fReg.RegLEDOUT1.bit.LDR4  = 3;
        sTlc59108fReg.RegPWM4 = bluePwm;
    }break;
    default:
    {
        return;
    }break;
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

    case LED_STATE_START: //上电初始化状态
        if(sLedFsm.stateTransferFlag)//left and right show light blue, it blinks twice within 500ms
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96); //pudu blue
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;

            TimeCnt = 0;
        }

        else
        {
            if(TimeCnt == 5)
            {
                LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x00, 0x00); //off 200ms
                I2C_sTlc59108fAllRegUpDate();
            }

            if(TimeCnt == 10)
            {
                LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96);//on 200ms
                I2C_sTlc59108fAllRegUpDate();
            }

            if(TimeCnt == 15)
            {
                LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x00, 0x00);//off 200ms
                I2C_sTlc59108fAllRegUpDate();
            }

            if(TimeCnt == 20)
            {
                LedFsmEventHandle(&sLedFsm, LED_EVENT_START_FINISH, LED_STATE_AWAIT, NULL);
            }
        }
        break;

    case LED_STATE_AWAIT://left and right show light pudu blue
        if(sLedFsm.stateTransferFlag)
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x32, 0x96);
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
            TimeCnt = 0;
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
            LedBreathePwmConfig(&sGreenBreathePwm[0], 25, 60, 0x32, 0x00, 0);
            LedBreathePwmConfig(&sBlueBreathePwm[0], 25, 60, 0x96, 0x00, 0);
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), GetBreathePwmValue(&sBlueBreathePwm[0]));
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
        }
        else
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), GetBreathePwmValue(&sBlueBreathePwm[0]));
            I2C_sTlc59108fAllRegUpDate();
        }
        break;

    case LED_STATE_CHARGE_0_19://left and right show red with 7.5s breathing period
        if(sLedFsm.stateTransferFlag)
        {
            LedBreathePwmConfig(&sRedBreathePwm[0], 25, 150, 0xFF, 0x00, 0);
            LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm[0]), 0x00, 0x00);
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
        }
        else
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm[0]), 0x00, 0x00);
            I2C_sTlc59108fAllRegUpDate();
        }
        break;

    case LED_STATE_CHARGE_20_39://left and right show orange with 6s breathing period
        if(sLedFsm.stateTransferFlag)
        {
            LedBreathePwmConfig(&sRedBreathePwm[0], 25, 120, 0xFF, 0x00, 0);
            LedBreathePwmConfig(&sGreenBreathePwm[0], 25, 120, 0x40, 0x00, 0);
            LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm[0]), GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
            //LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm[0]), 0x00, 0x00);
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
        }
        else
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm[0]), GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
            I2C_sTlc59108fAllRegUpDate();
        }
        break;

    case LED_STATE_CHARGE_40_59://left and right show yellow with 4.5s breathing period
        if(sLedFsm.stateTransferFlag)
        {
            LedBreathePwmConfig(&sRedBreathePwm[0], 25, 90, 0xFF, 0x00, 0);
            LedBreathePwmConfig(&sGreenBreathePwm[0], 25, 90, 0xB4, 0x00, 0);
            LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm[0]), GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
        }
        else
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, GetBreathePwmValue(&sRedBreathePwm[0]), GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
            I2C_sTlc59108fAllRegUpDate();
        }
        break;

    case LED_STATE_CHARGE_60_79://left and right show green with 3s breathing period
        if(sLedFsm.stateTransferFlag)
        {
            LedBreathePwmConfig(&sGreenBreathePwm[0], 25, 60, 0xFF, 0x00, 0);
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
        }
        else
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
            I2C_sTlc59108fAllRegUpDate();
        }
        break;

    case LED_STATE_CHARGE_80_99://left and right show light blue with 1.5s breathing period
        if(sLedFsm.stateTransferFlag)
        {
            LedBreathePwmConfig(&sGreenBreathePwm[0], 25, 30, 0x32, 0x00, 0); //pudu blue 00 32 96
            LedBreathePwmConfig(&sBlueBreathePwm[0], 25, 30, 0x96, 0x00, 0);
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), GetBreathePwmValue(&sBlueBreathePwm[0]));
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
        }
        else
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), GetBreathePwmValue(&sBlueBreathePwm[0]));
            I2C_sTlc59108fAllRegUpDate();
        }
        break;

    case LED_STATE_CHARGE_ING://left and right show green with 3s breathing period
        if(sLedFsm.stateTransferFlag)
        {
            LedBreathePwmConfig(&sGreenBreathePwm[0], 25, 60, 0xFF, 0x00, 0);
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
            I2C_sTlc59108fAllRegUpDate();

            sLedFsm.stateTransferFlag = 0;
        }
        else
        {
            LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, GetBreathePwmValue(&sGreenBreathePwm[0]), 0x00);
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
    case LED_STATE_POWEROFF://left and right show red with 500ms blinking period
        if(sLedFsm.stateTransferFlag)
        {
            if(sLedFsm.stateTransferFlag)
            {
                LedRGBPwmControl(LED_LEFT_RIGHT, 0x00, 0x00, 0x00);
                I2C_sTlc59108fAllRegUpDate();
                //ChassisLED_PowerOff();
                sLedFsm.stateTransferFlag = 0;
            }

        }
        break;

    default:
        break;
    }
}


extern uint8_t get_charge_state( void );

typedef struct {
    volatile int8_t r_color;
    volatile int8_t g_color;
    volatile int8_t b_color;
    volatile int8_t toggled;
    volatile uint8_t mode;
    volatile uint16_t number;  // 这里用16位用于判断是否无限闪烁
    volatile uint32_t timer;
    volatile uint32_t arg;
} LED_BAR_PARAM;

LED_BAR_PARAM bar_param[3] =
{
    {.r_color = 0, .g_color = 0, .b_color = 0, .toggled = 1, .mode = 0, .number = 0, .timer = 0, .arg = 0 },
    {.r_color = 0, .g_color = 0, .b_color = 0, .toggled = 1, .mode = 0, .number = 0, .timer = 0, .arg = 0 },
    {.r_color = 0, .g_color = 0, .b_color = 0, .toggled = 1, .mode = 0, .number = 0, .timer = 0, .arg = 0 }
};

void chassis_led_ctrl( uint8_t *buff )
{
    uint8_t lcolor = buff[2]; // 灯条颜色编号
    uint8_t lmode = buff[3];  //控制模式
    uint32_t larg = buff[6] | (buff[5]<<8); //时间 或灯条颜色
    uint8_t num_flash = buff[4]; //次数或灯条颜色
    uint8_t bar_id = buff[1]; //灯条编号
    uint8_t idx = 0;
    uint8_t bar_temp = 0; // 灯条重新编码 左侧灯条 位0，右侧灯条 位1，后侧灯条 位2；

    // 编码灯条
    if(bar_id == 12) // 全部灯条
    {
        bar_temp = 7;
    }
    else if(bar_id == 13) // 左侧灯条
    {
        bar_temp = 1;
    }
    else if(bar_id == 14) // 右侧灯条
    {
        bar_temp = 2;
    }
    else if(bar_id == 15) // 后侧灯条
    {
        bar_temp = 4;
    }
    else
    {
        return;
    }

    if( ReadChargeAppState() ) // 充电的时候不通过上位机控制
    {
        return;
    }

    if( lcolor > 6 || lmode > 6 )// 指令错误
    {
        return;
    }

    if( lcolor == 1) // 白色
    {
        chassis_led_color_extract(bar_temp,buff,0xff,0xff,0xff);

    } else if( lcolor == 2 ) {  // 红色

        chassis_led_color_extract(bar_temp,buff,0xff,0x00,0x00);

    } else if( lcolor == 3 ) {  // 绿色

        chassis_led_color_extract(bar_temp,buff,0x00,0xff,0x00);

    } else if( lcolor == 4 ) {  // pudu蓝色

        chassis_led_color_extract(bar_temp,buff,0x00,0x32,0x96);

    } else if( lcolor == 5 ) {  // 橙色

        chassis_led_color_extract(bar_temp,buff,0xff,0xa5,0x00);

    } else if( lcolor == 6 ) {  // 黄色

        chassis_led_color_extract(bar_temp,buff,0xff,0xff,0x00);

    }
    else if(lcolor == 0 && lmode == 5)
    {
        chassis_led_color_extract(bar_temp,buff,buff[4],buff[5],buff[6]);
    }

}
/**
 * @brief  用于辅助底盘led控制函数，将can总线下行的数据解析到指定参数组中
 * @param[in|out]  bar_temp 重新编码的led编号，0位表示灯条1；1位表示灯条2；2位表示灯条3 ，buff,数据帧
                   r_color，g_color，b_color ：颜色编码
 * @return      NULL
 */
PRIVATE void chassis_led_color_extract(uint8_t bar_temp,uint8_t *buff,uint8_t r_color,uint8_t g_color,uint8_t b_color)
{
    if( (bar_temp & 0x01) == 0x01 ) //左侧灯条
    {
        bar_param[0].r_color = r_color;
        bar_param[0].g_color = g_color;
        bar_param[0].b_color = b_color;
        bar_param[0].mode = buff[3];
        if(bar_param[0].mode == 5)//CAN发送RGB模式
        {
            bar_param[0].number = 0;
        }
        else
        {
            bar_param[0].arg = (buff[6] | (buff[5]<<8))/50; //周期
            bar_param[0].number =  buff[4]*2 + 1; //次数
        }
        bar_param[0].toggled = 0;

        if(bar_param[0].number == 0x1)
        {
            bar_param[0].number = 0x8000; // 闪烁次数寄存器的第15位为连续闪烁模式
        }
        bar_param[0].timer = 0;

    }
    else
    {
    }

    if( (bar_temp & 0x02) == 0x02 )//右侧灯条
    {
        bar_param[1].r_color = r_color;
        bar_param[1].g_color = g_color;
        bar_param[1].b_color = b_color;
        bar_param[1].mode = buff[3];
        if(bar_param[1].mode == 5)
        {
            bar_param[1].number = 0;
        }
        else
        {
            bar_param[1].arg = (buff[6] | (buff[5]<<8))/50;
            bar_param[1].number =  buff[4]*2 + 1;
        }
        bar_param[1].toggled = 0;

        if(bar_param[1].number == 0x01)
        {
            bar_param[1].number = 0x8000; // 闪烁次数寄存器的第15位为连续闪烁模式
        }
        bar_param[1].timer = 0;
    }
    else
    {
    }

    // 灯带3 是单色灯，只有单蓝色才会点亮，否则全部熄灭
    if( (bar_temp & 0x04) == 0x04 )
    {
        if( b_color == 0xff && r_color == 0x00 && g_color == 0x00 )
        {
            bar_param[2].r_color = 0x00;
            bar_param[2].g_color = 0x00;
            bar_param[2].b_color = 0xff;  // 原理图逻辑是反的，00 才是亮的
            bar_param[2].mode = buff[3];
            if(bar_param[2].mode == 5)
            {
                bar_param[2].number = 0;
            }
            else
            {
                bar_param[2].arg = (buff[6] | (buff[5]<<8))/50;
                bar_param[2].number =  buff[4]*2+1;
            }
            bar_param[2].toggled = 0;
            if(bar_param[2].number == 0x01)
            {
                bar_param[2].number = 0x8000; // 闪烁次数寄存器的第15位为连续闪烁模式
            }
            bar_param[2].timer = 0;

        }
        else
        {
            bar_param[2].r_color = 0x00;
            bar_param[2].g_color = 0x00;
            bar_param[2].b_color = 0x00;
            bar_param[2].mode = 0x00;
            bar_param[2].arg = 0x00;
            bar_param[2].number = 0x00;
            bar_param[2].toggled = 0;
        }
    }
    else
    {
    }
}

void led_bar_driver( void )
{
    uint8_t bar = 0;
    if( ReadChargeAppState() ) // 充电的时候不通过上位机控制
    {
        return;
    }
    bar_param[0].timer++; // 25ms
    bar_param[1].timer++;
    bar_param[2].timer++;
    //左侧灯条
    if( bar_param[0].mode == 2 || bar_param[0].mode == 4)  // 闪亮模式，通过 求余以及除法 计算
    {
        if(( bar_param[0].timer % bar_param[0].arg ) == 0 && ( ( bar_param[0].timer / bar_param[0].arg ) % 2 ) == 0 && bar_param[0].number > 0) // 闪烁完成常亮
        {
            if(bar_param[0].number != 1) //该周期将要减为0的时候不再向灯条内写入数据
            {
                LedRGBPwmControl(LED_LEFT, bar_param[0].r_color, bar_param[0].g_color, bar_param[0].b_color);
                I2C_sTlc59108fAllRegUpDate();
            }
            if(bar_param[0].number != 0x8000) // 无限次闪烁
            {
                bar_param[0].number --;
            }
        }
        else if(( bar_param[0].timer % bar_param[0].arg ) == 0 && ( ( bar_param[0].timer / bar_param[0].arg ) % 2 ) == 1 && bar_param[0].number > 0 )
        {   // 闪烁完成长灭
            if(bar_param[0].number != 1)
            {
                LedRGBPwmControl(LED_LEFT, 0, 0, 0);
                I2C_sTlc59108fAllRegUpDate();
            }
            if(bar_param[0].number != 0x8000) // 无限次闪烁
            {
                bar_param[0].number --;
            }
        }
        if(bar_param[0].number == 0 && bar_param[0].toggled == 0) // 闪烁完成，并且判断是否被触发过
        {
            if( bar_param[0].mode == 2 ) // 闪烁完成之后熄灭
            {
                LedRGBPwmControl(LED_LEFT, 0, 0, 0);
                I2C_sTlc59108fAllRegUpDate();
                bar_param[0].toggled = 1;
            }
            else if( bar_param[0].mode == 4)
            {   // 闪烁完成之后点亮
                LedRGBPwmControl(LED_LEFT, bar_param[0].r_color, bar_param[0].g_color, bar_param[0].b_color);
                I2C_sTlc59108fAllRegUpDate();
                bar_param[0].toggled = 1;
            }
        }

    }
    else if( bar_param[0].mode == 0 && bar_param[0].toggled == 0 )
    {
        LedRGBPwmControl(LED_LEFT, 0, 0, 0);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[0].toggled = 1;
    }
    else if (( bar_param[0].mode == 1 || bar_param[0].mode == 5) && bar_param[0].toggled == 0 )
    {
        LedRGBPwmControl(LED_LEFT, bar_param[0].r_color, bar_param[0].g_color, bar_param[0].b_color);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[0].toggled = 1;
    }
    else if(bar_param[0].mode == 3) // 呼吸灯模式
    {
        if(bar_param[0].toggled == 0)
        {
            LedBreathePwmConfig(&sGreenBreathePwm[0], 25, bar_param[0].arg, bar_param[0].g_color, 0x00, 0);
            LedBreathePwmConfig(&sBlueBreathePwm[0], 25, bar_param[0].arg,  bar_param[0].b_color, 0x00, 0);
            LedBreathePwmConfig(&sRedBreathePwm[0], 25, bar_param[0].arg,  bar_param[0].r_color, 0x00, 0);
            bar_param[0].toggled = 1;
        }
        else
        {
        }
        if(sBlueBreathePwm[0].pwmStepCnt == sBlueBreathePwm[0].timeNumber && bar_param[0].number != 0 && bar_param[0].number != 0x8000  )
        {
            bar_param[0].number --;
        }
        if(bar_param[0].number > 1)
        {
            LedRGBPwmControl(LED_LEFT, GetBreathePwmValue(&sRedBreathePwm[0]),GetBreathePwmValue(&sGreenBreathePwm[0]),GetBreathePwmValue(&sBlueBreathePwm[0]));
            I2C_sTlc59108fAllRegUpDate();
        }
        else if(bar_param[0].number > 0)
        {
            LedRGBPwmControl(LED_LEFT,0,0,0);
            I2C_sTlc59108fAllRegUpDate();
        }
    }
    // 右侧灯条
    if( bar_param[1].mode == 2 || bar_param[1].mode == 4)  // 闪亮模式，通过 求余以及除法 计算
    {
        if(( bar_param[1].timer % bar_param[1].arg ) == 0 && ( ( bar_param[1].timer / bar_param[1].arg ) % 2 ) == 0 && bar_param[1].number > 0) // 闪烁完成常亮
        {
            if(bar_param[1].number != 1) //该周期将要减为0的时候不再向灯条内写入数据
            {
                LedRGBPwmControl(LED_RIGHT, bar_param[1].r_color, bar_param[1].g_color, bar_param[1].b_color);
                I2C_sTlc59108fAllRegUpDate();
            }
            if(bar_param[1].number != 0x8000) // 无限次闪烁
            {
                bar_param[1].number --;
            }
        }
        else if(( bar_param[1].timer % bar_param[1].arg ) == 0 && ( ( bar_param[1].timer / bar_param[1].arg ) % 2 ) == 1 && bar_param[1].number > 0 )
        {   // 闪烁完成长灭
            if(bar_param[1].number != 1)
            {
                LedRGBPwmControl(LED_RIGHT, 0, 0, 0);
                I2C_sTlc59108fAllRegUpDate();
            }
            if(bar_param[1].number != 0x8000) // 无限次闪烁
            {
                bar_param[1].number --;
            }
        }
        if(bar_param[1].number == 0 && bar_param[1].toggled == 0) // 闪烁完成，并且判断是否被触发过
        {
            if( bar_param[1].mode == 2 ) // 闪烁完成之后熄灭
            {
                LedRGBPwmControl(LED_RIGHT, 0, 0, 0);
                I2C_sTlc59108fAllRegUpDate();
                bar_param[1].toggled = 1;
            }
            else if( bar_param[1].mode == 4)
            {   // 闪烁完成之后点亮
                LedRGBPwmControl(LED_RIGHT, bar_param[1].r_color, bar_param[1].g_color, bar_param[1].b_color);
                I2C_sTlc59108fAllRegUpDate();
                bar_param[1].toggled = 1;
            }
        }
    }
    else if( bar_param[1].mode == 0 && bar_param[1].toggled == 0 )
    {
        LedRGBPwmControl(LED_RIGHT, 0, 0, 0);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[1].toggled = 1;
    }
    else if (( bar_param[1].mode == 1 || bar_param[1].mode == 5) && (bar_param[1].toggled == 0 ))
    {
        LedRGBPwmControl(LED_RIGHT, bar_param[1].r_color, bar_param[1].g_color, bar_param[1].b_color);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[1].toggled = 1;
    }
    else if(bar_param[1].mode == 3) // 呼吸灯模式
    {
        if(bar_param[1].toggled == 0)
        {
            LedBreathePwmConfig(&sGreenBreathePwm[1], 25, bar_param[1].arg, bar_param[1].g_color, 0x00, 0);
            LedBreathePwmConfig(&sBlueBreathePwm[1], 25, bar_param[1].arg,  bar_param[1].b_color, 0x00, 0);
            LedBreathePwmConfig(&sRedBreathePwm[1], 25, bar_param[1].arg,  bar_param[1].r_color, 0x00, 0);
            bar_param[1].toggled = 1;
        }
        else
        {
        }
        if(sBlueBreathePwm[1].pwmStepCnt == sBlueBreathePwm[1].timeNumber && bar_param[1].number != 0 && bar_param[1].number != 0x8000 )
        {
            bar_param[1].number --;
        }
        if(bar_param[1].number > 1)
        {
            LedRGBPwmControl(LED_RIGHT, GetBreathePwmValue(&sRedBreathePwm[1]),GetBreathePwmValue(&sGreenBreathePwm[1]),GetBreathePwmValue(&sBlueBreathePwm[1]));
            I2C_sTlc59108fAllRegUpDate();
        }
        else if(bar_param[1].number > 0)
        {
            LedRGBPwmControl(LED_RIGHT,0,0,0);
            I2C_sTlc59108fAllRegUpDate();
        }
    }
    //后侧灯条
    if( bar_param[2].mode == 2)
    {
        if(( bar_param[2].timer % bar_param[2].arg ) == 0 && ( ( bar_param[2].timer / bar_param[2].arg ) % 2 ) == 0 && bar_param[2].number > 0) // 闪烁完成常亮
        {
            if(bar_param[2].number != 1) //该周期将要减为0的时候不再向灯条内写入数据
            {
                LedRGBPwmControl(LED_BACK, bar_param[2].r_color, bar_param[2].g_color, bar_param[2].b_color);
                I2C_sTlc59108fAllRegUpDate();
            }
            if(bar_param[2].number != 0x8000) // 无限次闪烁
            {
                bar_param[2].number --;
            }
        }
        else if(( bar_param[2].timer % bar_param[2].arg ) == 0 && ( ( bar_param[2].timer / bar_param[2].arg ) % 2 ) == 1 && (bar_param[2].number > 0 ))
        {   // 闪烁完成长灭
            if(bar_param[2].number != 1)
            {
                LedRGBPwmControl(LED_BACK, 0, 0, 0);
                I2C_sTlc59108fAllRegUpDate();
            }
            if(bar_param[2].number != 0x8000) // 无限次闪烁
            {
                bar_param[2].number --;
            }
        }
        if(bar_param[2].number == 0 && bar_param[2].toggled == 0) // 闪烁完成，并且判断是否被触发过
        {
            if( bar_param[2].mode == 2 ) // 闪烁完成之后熄灭
            {
                LedRGBPwmControl(LED_BACK, 0, 0, 0);
                I2C_sTlc59108fAllRegUpDate();
                bar_param[2].toggled = 1;
            }
            else if( bar_param[2].mode == 4)
            {   // 闪烁完成之后点亮
                LedRGBPwmControl(LED_BACK, bar_param[2].r_color, bar_param[2].g_color, bar_param[2].b_color);
                I2C_sTlc59108fAllRegUpDate();
                bar_param[2].toggled = 1;
            }
        }
    }
    else if( bar_param[2].mode == 0 && bar_param[2].toggled == 0 )
    {
        LedRGBPwmControl(LED_BACK, 0, 0, 0);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[2].toggled = 1;
    }
    else if (( bar_param[2].mode == 1 || bar_param[2].mode == 5) && bar_param[2].toggled == 0 )
    {
        LedRGBPwmControl(LED_BACK, bar_param[2].r_color, bar_param[2].g_color, bar_param[2].b_color);
        I2C_sTlc59108fAllRegUpDate();
        bar_param[2].toggled = 1;
    }
    else if(bar_param[2].mode == 3) // 呼吸灯模式
    {
        if(bar_param[2].toggled == 0)
        {
//            LedBreathePwmConfig(&sGreenBreathePwm[2], 25, bar_param[2].arg, bar_param[2].g_color, 0x00, 0);
            LedBreathePwmConfig(&sBlueBreathePwm[2], 25, bar_param[2].arg,  bar_param[2].b_color, 0x00, 0);
//            LedBreathePwmConfig(&sRedBreathePwm[2], 25, bar_param[2].arg,  bar_param[2].r_color, 0x00, 0);
            bar_param[2].toggled = 1;
        }
        else
        {
        }
        if(sBlueBreathePwm[2].pwmStepCnt == sBlueBreathePwm[2].timeNumber && bar_param[2].number != 0 && bar_param[2].number != 0x8000  )
        {
            bar_param[2].number --;
        }
        if(bar_param[2].number > 1)
        {
            LedRGBPwmControl(LED_BACK, GetBreathePwmValue(&sRedBreathePwm[2]),GetBreathePwmValue(&sGreenBreathePwm[2]),GetBreathePwmValue(&sBlueBreathePwm[2]));
            I2C_sTlc59108fAllRegUpDate();
        }
        else if(bar_param[2].number > 0)
        {
            LedRGBPwmControl(LED_BACK,0,0,0);
            I2C_sTlc59108fAllRegUpDate();
        }

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
extern PUBLIC UINT32 ReadTimeStampTimer(void);
extern struct PowerManagerStruct sPowerManager;
PUBLIC void GetChargeState(uint8_t *recbuff)
{
    UINT8 chargestate = recbuff[1];
    UINT8 soc = recbuff[3];
    static UINT32 timeold = 0;
    UINT32 timenow = ReadTimeStampTimer();
	
		if(recbuff[1] == 11)//充电桩充电
		{
				sPowerManager.sChargeInfo.ChargeMode = 2;
		}
		else if(recbuff[1] == 1)
		{
				sPowerManager.sChargeInfo.ChargeMode = 1;
		}
		else
		{
				sPowerManager.sChargeInfo.ChargeMode = 0;
		}
    if((timenow-timeold)<27000000)//1s
    {
        return ;
    }
    else
    {
        timeold = timenow;
    }
    //根据充电状态，切换灯带状态
    if ((recbuff[1]==1)||(recbuff[1]==11))
    {
        //插上充电后，即显示充电呼吸效果
        LedFsmEventHandle(&sLedFsm, LED_EVENT_CHARGE_ING, GetBatteryLevelForLed(recbuff[3]), NULL);
        sPowerManager.sChargeInfo.ChargeAppState = CHARGING;
    }
    else if(((recbuff[1] >= 5)&&(recbuff[1] <= 10))||(recbuff[1] >= 14))//电池故障增加了
    {
        LedFsmEventHandle(&sLedFsm, LED_EVENT_CHARGE_OUT, LED_STATE_AWAIT, NULL);
        //当出现故障时，接收上位机控制，提示红色闪烁故障灯
        LedFsmEventHandle(&sLedFsm, LED_EVENT_REMOTE_CONTROL, LED_STATE_ERROR, NULL);
        sPowerManager.sChargeInfo.ChargeAppState = POWER_ALARM;
    }
    else //if(recbuff[1] == 0) // 未充电
    {
        //拔掉充电器后，进入正常的蓝色待机状态
        LedFsmEventHandle(&sLedFsm, LED_EVENT_CHARGE_OUT, LED_STATE_AWAIT, NULL);
        sPowerManager.sChargeInfo.ChargeAppState = NOT_CHARGED;
    }
    gSensorData.BatteryLevel0x400D = recbuff[4];
}
