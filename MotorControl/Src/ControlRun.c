/**********************************************************************
 *
 * FILE NAME:  ControlRun.c
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
#include "Encoder.h"
#include "ControlRun.h"
#include "HardApi.h"
#include "UartApp.h"
#include "stdio.h"
#include "CanopenApp.h"
#include "StateMachine.h"
#include "stm32f7xx_hal.h"
#include "myiic.h"
#include "Gyro.h"
#include "Rgb.h"
#include "Param.h"
#include "CanApp.h"
#include "ErrorLog.h"
#include "PowerManager.h"
#include "LedDriver.h"
#include "gpio.h"
#include "MachineAdditionalInfo.h"

#define CAN_LOG_MODULE

#define MAX_SCH_NUM  10000  // max scheduler number  1s

extern PUBLIC void CurrentLoopInit(struct AxisCtrlStruct *P);
extern PUBLIC void CurrentLoopExec(struct AxisCtrlStruct *P);
extern PUBLIC void SpeedLoopInit(struct AxisCtrlStruct *P);
extern PUBLIC void SpeedLoopExec(struct AxisCtrlStruct *P);
extern PUBLIC void PositionLoopInit(struct AxisCtrlStruct *P);
extern PUBLIC void PositionLoopExec(struct AxisCtrlStruct *P);

extern PUBLIC void EncoderInit(struct AxisCtrlStruct *P);
extern PUBLIC void GetEncoderPulse(struct EncoderStruct *P, UINT16 AxisID);
extern PUBLIC void AbsEncoderReadStart(void);
extern PUBLIC void EncoderParUpdate(struct EncoderStruct *P);

extern PUBLIC void AlarmInit(struct AxisCtrlStruct *P);
extern PUBLIC void AlarmExec(struct AxisCtrlStruct *P);
extern PUBLIC void AlarmExec_5(struct AxisCtrlStruct *P);
extern PUBLIC void ClearCanBreakAlarm(void);

extern PUBLIC void InnerCtrlExec(struct AxisCtrlStruct *P);

extern PUBLIC void VeneerAgingTest(void);



struct AxisCtrlStruct sAxis[MAX_AXIS_NUM];
struct SchedulerStruct sScheduler;

PRIVATE void CanFdbMcInfoExec(void);
PRIVATE void PowerOnOffExec(struct AxisCtrlStruct *P, UINT32 IsrTime);
PRIVATE void MotionParamUpdateExec(void);
PRIVATE void ParamFdbUpdate(void);


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ControlRunInit(void)
{
    for(int i=0;i<MAX_AXIS_NUM;i++)
    {
        sAxis[i].AxisID = i;
        
        sAxis[i].BootStrapCapChargeEn = sAxis[i].BootStrapCapChargeFlag = 0;
        sAxis[i].BootStrapCapChargeTime = 10;  //1ms

        sAxis[i].StarSealProtectFlag = 0;
		sAxis[i].StarSealProtectEn = 0;
        sAxis[i].StarSealProtectTime = 1000;//100ms//100;  //10ms        
        
        EncoderInit(&sAxis[i]);
        CurrentLoopInit(&sAxis[i]);
        SpeedLoopInit(&sAxis[i]);
        PositionLoopInit(&sAxis[i]);
        AlarmInit(&sAxis[i]);
        
        sAxis[i].sFilterCfg.all = gParam[i].FilterConfig0x2105;
    }    
}

/***********************************************************************
 * DESCRIPTION: main ISR 10khz
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ControlRunExec(void)
{
	/* Read time stamp at the entrance of interrupt:
            for calculate the elapsed time of interrupt*/
	UINT32 IsrStartTime = ReadTimeStampTimer();
    
    static UINT16 Cnt_1ms = 0;
    
	/* Start phase current,DC current,DC voltage sample */
	AdcSampleStart();
    
	/* Get encoder counter*/
	GetEncoderPulse(&sAxis[0].sEncoder, AXIS_LEFT);
    GetEncoderPulse(&sAxis[1].sEncoder, AXIS_RIGHT);
      
	/* Encoder calculation: 
             for electric angle,speed increment,position increment */
	EncoderCalExec(&sAxis[0]);
    EncoderCalExec(&sAxis[1]);
    
	/* Alarm execute each cycle*/
	AlarmExec(&sAxis[0]);
    AlarmExec(&sAxis[1]);
    
    InnerCtrlExec(&sAxis[0]);
    InnerCtrlExec(&sAxis[1]);
    
	/* Execute every 5 cycles */
	UINT16 Tick = sScheduler.SchNum%5;
//*
	if(sScheduler.SchNum%2)
	{
		SpeedLoopExec(&sAxis[0]);
        SpeedLoopExec(&sAxis[1]);
	}
	else
	{
        CiA402_MotionControl();
	}
//*/	
	switch(Tick)
	{
		case 0:
			/* Position loop calculation */
			PositionLoopExec(&sAxis[0]);
            PositionLoopExec(&sAxis[1]);
        
			/* Speed loop calculation */
//			SpeedLoopExec(&sAxis[0]);
//            SpeedLoopExec(&sAxis[1]);
		break;

		case 1:
			/* Alarm execute every 5 cycles*/
			AlarmExec_5(&sAxis[0]);
		break;

		case 2:
            /* Alarm execute every 5 cycles*/
			AlarmExec_5(&sAxis[1]);
		break;

		case 3:
            //CiA402_MotionControl();
		break;

		case 4:
            if(Cnt_1ms++>=2)
            {
                Cnt_1ms = 0;
                timerForCan1MS();
            }
		break;
	}

    /* wait for ADC to complete, then acknowledge flag	*/ 
    AdcSampleClearFlag();

	/* Calculate phase current */
	GetPhaseCurrent(AXIS_LEFT,  &sAxis[0].sCurLoop.Ia,  &sAxis[0].sCurLoop.Ib );
    GetPhaseCurrent(AXIS_RIGHT, &sAxis[1].sCurLoop.Ia, &sAxis[1].sCurLoop.Ib);
    
    GetDcVoltage(&sAxis[0].sCurLoop.Vdc);
    sAxis[1].sCurLoop.Vdc = sAxis[0].sCurLoop.Vdc;
    
	/* Current loop calculation */
	CurrentLoopExec(&sAxis[0]);
    CurrentLoopExec(&sAxis[1]);
    
	/* Update PWM compare value */
    PwmUpdate(AXIS_LEFT, sAxis[0].PowerFlag, sAxis[0].sCurLoop.TaNumber, sAxis[0].sCurLoop.TbNumber, sAxis[0].sCurLoop.TcNumber);
    PwmUpdate(AXIS_RIGHT, sAxis[1].PowerFlag, sAxis[1].sCurLoop.TaNumber, sAxis[1].sCurLoop.TbNumber, sAxis[1].sCurLoop.TcNumber);    
	
    /* PWM on/off judgment */
    PowerOnOffExec(&sAxis[0], sScheduler.TickCnt);
    PowerOnOffExec(&sAxis[1], sScheduler.TickCnt);
    
    DataCollectingLoop(sScheduler.TickCnt);
    
	/* Data collect module  */
//	data_collect_loop();

    sScheduler.SchNum++;
	sScheduler.SchNum = sScheduler.SchNum%MAX_SCH_NUM;
    
    if(sScheduler.SchNum == 0)  // 1s
    {
        sScheduler.TimeStamp++;
        gParam[0].TimeStamp0x2500++;
    }

	/* Main interrupt tick: add 1 at each cycle */
	sScheduler.TickCnt++;
    
	/* Calculate the elapsed time of interrupt */
    sScheduler.IsrElapsedTime = ReadTimeStampTimer() - IsrStartTime;

}

/***********************************************************************
 * DESCRIPTION:general timer interrupt . 40HZ
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void TimerIsrExec(void)
{   
    UINT32 StartTime = ReadTimeStampTimer();

    CumulativeTimeCount();
    
    //CanFdb motion control info
    CanFdbMcInfoExec();
    
    //Gyro task 
    GyroExec();
    
    LedDriverExec();
    
    led_bar_driver();
    // StateMachine
    CiA402_StateMachine();
    
    //Parameter Reference update
    MotionParamUpdateExec();
    
    //Parameter Feedback update
    ParamFdbUpdate();
    
    //Rgb task 
    RgbExec();

    // CanApp task
    CanAppExec();
    
    // PowerManager task    
    PowerManagerExec();
	
//	VeneerAgingTest();
    
    ClearCanBreakAlarm();
    
    sScheduler.Tim7IsrTime = ReadTimeStampTimer() - StartTime;
    
    if(sScheduler.Tim7IsrTime > MAX_TIMER_ISR_TIME) // 15ms
    {
        sAxis[0].sAlarm.ErrReg.bit.InnerErr = 1;
        sAxis[1].sAlarm.ErrReg.bit.InnerErr = 1;
    }    
}

/***********************************************************************
* DESCRIPTION: // 40HZ
*
*
*
***********************************************************************/
PUBLIC void VeneerAgingTest(void)
{
	static UINT16 Cnt_25ms = 0;
	static UINT16 testFlg = 0;
	static UINT16 testStateBak = 0;

	if(VeneerAgingTestState() == 2 && testStateBak == 2 && testFlg != 2)
	{
		testFlg = 1;
		
		gParam[1].ProfileAcc0x6083 = gParam[1].ProfileDec0x6084 = 40000;
		gParam[0].ProfileAcc0x6083 = gParam[0].ProfileDec0x6084 = 40000;
		gParam[0].ControlWord0x6040 = 0xF;
		gParam[1].ControlWord0x6040 = 0xF;
		Cnt_25ms ++;
		if(Cnt_25ms <= 60)
		{
			gParam[0].TargetVelocity0x60FF = 13653;
			//gParam[1].TargetVelocity0x60FF = 13653;
		}
		else if(Cnt_25ms <= 120)
		{
			//gParam[0].TargetVelocity0x60FF = 13653;
			gParam[1].TargetVelocity0x60FF = 13653;
		}
		else if(Cnt_25ms <= 180)
		{
			gParam[0].TargetVelocity0x60FF = -13653;
			//gParam[1].TargetVelocity0x60FF = 13653;
		}
		else if(Cnt_25ms <= 240)
		{
			//gParam[0].TargetVelocity0x60FF = -13653;
			gParam[1].TargetVelocity0x60FF = -13653;
			if(Cnt_25ms == 240)
			{
				Cnt_25ms = 0;
			}
		}	
	}
	else if((testFlg != 0) )//&& (VeneerAgingTestState() == 0))
	{
		Cnt_25ms++;
		testFlg = 2;
		if(Cnt_25ms > 250)
		{
			testFlg = 0;
			Cnt_25ms = 0;
		}		
		gParam[0].ControlWord0x6040 = 0x6;
		gParam[1].ControlWord0x6040 = 0x6;
		//gParam[0].TargetVelocity0x60FF = 0;
		//gParam[1].TargetVelocity0x60FF = 0;
	}
	testStateBak = VeneerAgingTestState();

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT32 GetIsrElapsedTime(void)
{
    return sScheduler.IsrElapsedTime;
}

/***********************************************************************
 * DESCRIPTION: 40HZ
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanFdbMcInfoExec(void)
{
    static UINT8 Tick = 0;    
    static INT16 LeftPulseOld = 0;
    static INT16 RightPulseOld = 0;
    volatile INT16 Tmp;
    INT16 LeftInc ,RightInc;
    
	Tmp = GetIncEncoderPulse(0);
    LeftInc= sAxis[0].sEncoder.MotorDirection*(Tmp - LeftPulseOld);
    LeftPulseOld = Tmp;
    
    Tmp = GetIncEncoderPulse(1);
    RightInc= sAxis[1].sEncoder.MotorDirection*(Tmp - RightPulseOld);
    RightPulseOld = Tmp;

    {
        static unsigned long  RemainingMileage; 
        static unsigned char  OdometerRefreshPeriod;
        
        UpdateWheelEncoderPlusCount( &WheelsMileage[AXIS_LEFT], LeftInc );
        UpdateWheelEncoderPlusCount( &WheelsMileage[AXIS_RIGHT], RightInc );
        OdometerRefreshPeriod++;
        if( OdometerRefreshPeriod >= 4 )
        {
            unsigned long LeftWheel;
            unsigned long RightWheel;
            
            OdometerRefreshPeriod = 0;
            CalculateWheelMileage( &WheelsMileage[AXIS_LEFT], gMachineInfo.machineWheelPerimeter, gParam[AXIS_LEFT].EncoderPPR0x2202 );
            CalculateWheelMileage( &WheelsMileage[AXIS_RIGHT], gMachineInfo.machineWheelPerimeter, gParam[AXIS_RIGHT].EncoderPPR0x2202 );
            LeftWheel = ( unsigned long )WheelsMileage[AXIS_LEFT].Mileage;
            WheelsMileage[AXIS_LEFT].Mileage -= ( float )LeftWheel;
            RightWheel = ( unsigned long )WheelsMileage[AXIS_RIGHT].Mileage;
            WheelsMileage[AXIS_RIGHT].Mileage -= ( float )RightWheel;
            RemainingMileage += LeftWheel + RightWheel;
            OdometerCounter += RemainingMileage >> 1;
            RemainingMileage &= 0x01;
        }
    }
    
    CanSendSpdFdb(LeftInc, RightInc);
    
    //Fdb Error :10HZ
    if(Tick == 0)
    {
        CanSendErrorCode(sAxis[0].sAlarm.ErrReg.all, sAxis[1].sAlarm.ErrReg.all);
		CanSendErrorCodeHigh((sAxis[0].sAlarm.ErrReg.all>>16), (sAxis[1].sAlarm.ErrReg.all>>16));
    }
 
#if defined CAN_LOG_MODULE  
    
    //Left Log :10HZ
    if(Tick == 1)
    {
        CanSendLog1Left(sAxis[0].sSpdLoop.SpdRef, sAxis[0].sSpdLoop.SpdFdb, 
                       sAxis[0].sCurLoop.IqRef*1000, sAxis[0].sAlarm.ErrReg.all);
        
        CanSendLog2Left(sAxis[0].sCurLoop.Idc*1000, sAxis[0].sCurLoop.IqFdb*1000, gParam[0].MosTemp0x230B, gParam[0].MotorTemp0x230C);
    }
    
    //Right Log :10HZ
    if(Tick == 2)
    {
        CanSendLog1Right(sAxis[1].sSpdLoop.SpdRef, sAxis[1].sSpdLoop.SpdFdb, 
                        sAxis[1].sCurLoop.IqRef*1000, sAxis[1].sAlarm.ErrReg.all);
        
        CanSendLog2Right(sAxis[1].sCurLoop.Idc*1000, sAxis[1].sCurLoop.IqFdb*1000, gParam[1].MosTemp0x230B, gParam[1].MotorTemp0x230C);
    }
    
    if(Tick == 3)
    {

    }
    
    (Tick >= 3) ? (Tick = 0) : Tick++;
    
#endif   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PowerOnOffExec(struct AxisCtrlStruct *P, UINT32 IsrTime)
{
    if((POWER_ON == P->PowerEn) && (POWER_OFF == P->PowerFlag))
    {
        P->BootStrapCapChargeEn = 1;
    }
    else if((POWER_OFF == P->PowerEn) && (POWER_ON == P->PowerFlag))
    {
        P->StarSealProtectEn = 1;
    }
    
    /*Boot Strap Cap Charge Judge if OpenLoop switch to CloseLoop */
    if (1 == P->BootStrapCapChargeEn)
    {
        /*Record Cap Charge Start Time*/
        if (0 == P->BootStrapCapChargeFlag)
        {
            P->BootStrapCapChargeStartTime = IsrTime;
        }
        if ((IsrTime - P->BootStrapCapChargeStartTime) >= P->BootStrapCapChargeTime)
        {
            P->BootStrapCapChargeEn = 0;
            P->BootStrapCapChargeFlag = 0;
        }
        else
        {
            /*Boot Strap Cap Charge*/
            PwmUpdate(P->AxisID, P->PowerFlag, 0, 0, 0);
            if (0 == P->BootStrapCapChargeFlag)
            {
                PwmEnable(P->AxisID);
                P->PowerFlag = 1;
                P->BootStrapCapChargeFlag = 1;
            }
        }
    }

    /*Star Sealing Protection if CloseLoop switch to OpenLoop*/
    if (1 == P->StarSealProtectEn)
    {
        /*Record Star Sealing Protection Start Time*/
        if (0 == P->StarSealProtectFlag)
        {
            P->StarSealProtectStartTime = IsrTime;
        }
        if ((IsrTime - P->StarSealProtectStartTime) >= P->StarSealProtectTime)
        {
            P->StarSealProtectEn = 0;
            P->StarSealProtectFlag = 0;
            PwmDisable(P->AxisID);
            P->PowerFlag = 0;
        }
        else
        {
            /*Star Sealing Protection*/
            PwmUpdate(P->AxisID, P->PowerFlag, 0, 0, 0);
            if (0 == P->StarSealProtectFlag)
            {
                P->StarSealProtectFlag = 1;
            }
        }
    }
 }


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
#define SPD_KP_FACTOR   0.0001f
#define SPD_KI_FACTOR   0.000001f
#define CUR_KP_FACTOR   0.001f
#define CUR_KI_FACTOR   0.001f
PRIVATE void MotionParamUpdateExec(void)
{
    volatile float Tmp = 0;
    
    Tmp = (float)gParam[0].ProfileAcc0x6083/(float)gParam[0].EncoderPPR0x2202;
    sAxis[0].sSpdLoop.AccMax = Tmp*6.0f;//Tmp*SPEED_PRD*60.0f;
    if(gParam[0].QuickStopEn0x6086)
    {
        Tmp = (float)gParam[0].QuickStopDec0x6085/(float)gParam[0].EncoderPPR0x2202;
    }
    else
    {
        Tmp = (float)gParam[0].ProfileDec0x6084/(float)gParam[0].EncoderPPR0x2202;
    }
    sAxis[0].sSpdLoop.DecMax = Tmp*6.0f;//Tmp*SPEED_PRD*60.0f;
    
    Tmp = (float)gParam[1].ProfileAcc0x6083/(float)gParam[1].EncoderPPR0x2202;
    sAxis[1].sSpdLoop.AccMax = Tmp*6.0f;//Tmp*SPEED_PRD*60.0f;
    if(gParam[1].QuickStopEn0x6086)
    {
        Tmp = (float)gParam[1].QuickStopDec0x6085/(float)gParam[1].EncoderPPR0x2202;
    }
    else
    {
        Tmp = (float)gParam[1].ProfileDec0x6084/(float)gParam[1].EncoderPPR0x2202;
    }
    sAxis[1].sSpdLoop.DecMax = Tmp*6.0f;//Tmp*SPEED_PRD*60.0f;
    
    sAxis[0].sCurLoop.Cp = (float)gParam[0].CurrentLoopKp0x2103*CUR_KP_FACTOR;
    sAxis[0].sCurLoop.Ci = (float)gParam[0].CurrentLoopKi0x2104*CUR_KI_FACTOR;
    
    sAxis[1].sCurLoop.Cp = (float)gParam[1].CurrentLoopKp0x2103*CUR_KP_FACTOR;
    sAxis[1].sCurLoop.Ci = (float)gParam[1].CurrentLoopKi0x2104*CUR_KI_FACTOR;
    
    sAxis[0].sSpdLoop.Vp = (float)gParam[0].VelocityLoopKp0x2101*SPD_KP_FACTOR;
    sAxis[0].sSpdLoop.Vi = (float)gParam[0].VelocityLoopKi0x2102*SPD_KI_FACTOR;
    
    sAxis[1].sSpdLoop.Vp = (float)gParam[1].VelocityLoopKp0x2101*SPD_KP_FACTOR; 
    sAxis[1].sSpdLoop.Vi = (float)gParam[1].VelocityLoopKi0x2102*SPD_KI_FACTOR;
    
//    sAxis[0].sSpdLoop.Kpdff = (float)gParam[0].PositionLoopKp0x2100*0.01f;
//    sAxis[1].sSpdLoop.Kpdff = sAxis[0].sSpdLoop.Kpdff;
    
    sAxis[0].sSpdLoop.TorFFGain = (float)gParam[0].TorffGain0x210B*0.1f;
    sAxis[1].sSpdLoop.TorFFGain = (float)gParam[1].TorffGain0x210B*0.1f;
    sAxis[0].sSpdLoop.DisturGain = (float)gParam[0].DisturGainFc0x210D*0.01f;
    sAxis[1].sSpdLoop.DisturGain = (float)gParam[1].DisturGainFc0x210D*0.01f;
    
    if(gParam[0].InnerCtrlEnable0x3004)
    {
        sAxis[0].sInnerCtrl.Type = gParam[0].InnerCtrlType0x3005;
        sAxis[0].sInnerCtrl.Cycle = gParam[0].InnerCtrlCycle0x3006;
        sAxis[0].sInnerCtrl.Period = gParam[0].InnerCtrlPeriod0x3007;
        sAxis[0].sInnerCtrl.Amp = gParam[0].InnerCtrlAmp0x3008;
        sAxis[0].sInnerCtrl.Offset = gParam[0].InnerCtrlOffset0x3009;
        sAxis[0].sInnerCtrl.En = 1;
        
        sAxis[1].sInnerCtrl.Type = gParam[0].InnerCtrlType0x3005;
        sAxis[1].sInnerCtrl.Cycle = gParam[0].InnerCtrlCycle0x3006;
        sAxis[1].sInnerCtrl.Period = gParam[0].InnerCtrlPeriod0x3007;
        sAxis[1].sInnerCtrl.Amp = gParam[0].InnerCtrlAmp0x3008;
        sAxis[1].sInnerCtrl.Offset = gParam[0].InnerCtrlOffset0x3009;
        sAxis[1].sInnerCtrl.En = 1;
    }
    else
    {
        sAxis[0].sInnerCtrl.En = 0;
        sAxis[1].sInnerCtrl.En = 0;
    }
     
}
/***********************************************************************
 * DESCRIPTION: // 40HZ
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ParamFdbUpdate(void)
{
    static UINT32 Tick=0;
     
    if(Tick == 0)
    {
        for(int i=0; i<MAX_AXIS_NUM; i++)
        {
            gParam[i].IdRef0x2300 = sAxis[i].sCurLoop.IdRef*1000.0f;
            gParam[i].IqRef0x2301 = sAxis[i].sCurLoop.IqRef*1000.0f;
            gParam[i].IdFdb0x2302 = sAxis[i].sCurLoop.IdFdb*1000.0f;
            gParam[i].IqFdb0x2303 = sAxis[i].sCurLoop.IqFdb*1000.0f;
            gParam[i].VdRef0x2304 = sAxis[i].sCurLoop.VdRef*1000.0f*sAxis[i].sCurLoop.Vdc;
            gParam[i].VqRef0x2305 = sAxis[i].sCurLoop.VqRef*1000.0f*sAxis[i].sCurLoop.Vdc;
            gParam[i].Ia0x2306    = sAxis[i].sCurLoop.Ia*1000.0f;
            gParam[i].Ib0x2307    = sAxis[i].sCurLoop.Ib*1000.0f;
            gParam[i].Ic0x2308    = sAxis[i].sCurLoop.Ic*1000.0f;
            gParam[i].Vdc0x2309   = sAxis[i].sCurLoop.Vdc*1000.0f;
            gParam[i].EncoderSingleTurn0x230A = sAxis[i].sEncoder.SingleTurn;
        //    gParam[i].MotorTemp0x230C = 
            gParam[i].ErrorRegister0x230D = sAxis[i].sAlarm.ErrReg.all;
 
            gParam[i].ActualVelocity0x606C = (float)sAxis[i].sEncoder.PulseMax*sAxis[i].sSpdLoop.SpdFdb/60.0f;
            gParam[i].ActualCurrent0x6078 = sAxis[i].sCurLoop.IqFdb*1000.0f;
            
            gParam[i].ActualPosition0x6064 = sAxis[i].sPosLoop.PosFdb;
        }
    }
    else if(Tick == 1)
    {
        GetGyroData(&gSensorData.GyroSpeedX0x4000);

    }
    else if(Tick == 2)
    {
        
    }
    else if(Tick == 3)
    {
        
    }
    
    (Tick >= 3) ? (Tick = 0) : Tick++;
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void MC_SetReference(struct McRefStruct *pRef, UINT16 AxisID)
{
    if(AXIS_LEFT == AxisID)
    {
        sAxis[0].PowerEn = pRef->PowerOn;     
        sAxis[0].CtrlMode = pRef->ModeRef;
        if(sAxis[0].CtrlMode == CUR_CTRL)
        {
            sAxis[0].sCurLoop.IqRef = pRef->CurRef;
        }
        else if(sAxis[0].CtrlMode == SPD_CTRL)
        {
            sAxis[0].sSpdLoop.SpdRef = pRef->SpdRef;
        }
        else if(sAxis[0].CtrlMode == POS_CTRL)
        {
            sAxis[0].sPosLoop.PosRef = pRef->PosRef;
        } 
     
    }
    else if(AXIS_RIGHT == AxisID)
    {
        sAxis[1].PowerEn = pRef->PowerOn;              
        sAxis[1].CtrlMode = pRef->ModeRef;
        if(sAxis[1].CtrlMode == CUR_CTRL)
        {
            sAxis[1].sCurLoop.IqRef = pRef->CurRef;
        }
        else if(sAxis[1].CtrlMode == SPD_CTRL)
        {
            sAxis[1].sSpdLoop.SpdRef = pRef->SpdRef;
        }
        else if(sAxis[1].CtrlMode == POS_CTRL)
        {
            sAxis[1].sPosLoop.PosRef = pRef->PosRef;
        } 
    }   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void MC_GetFeedback(struct McFdbStruct *pFdb, UINT16 AxisID)
{
    if(AXIS_LEFT == AxisID)
    {

    }
    else if(AXIS_RIGHT == AxisID)
    {

    }     
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT16 MC_GetErrorReg(UINT16 AxisID)
{
    UINT16 ErrReg=0;

    ErrReg = sAxis[AxisID].sAlarm.ErrReg.all;

    return ErrReg;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC INT32 DataCollectGetValue(UINT16 Index)
{
    INT32 res=0;
    switch(Index)
    {
        case 0x2300 :
            res = sAxis[0].sCurLoop.IdRef*1000;
            break;
        
        case 0x2301 :
            res = sAxis[0].sCurLoop.IqRef*1000;
            break;
        
        case 0x2302 :
            res = sAxis[0].sCurLoop.IdFdb*1000;//sAxis[0].sCurLoop.IValidFdb*1000;//
            break;
        
        case 0x2303 :
            res = sAxis[0].sCurLoop.IqFdb*1000;//sAxis[0].sCurLoop.IValidFdb*1000;//
            break;
        
        case 0x2304 :
            res = sAxis[0].sCurLoop.VdRef*1000;
            break;
        
        case 0x2305 :
            res = sAxis[0].sCurLoop.VqRef*1000;
            break;
        
         case 0x2306 :
            res = sAxis[0].sCurLoop.Ia*1000;
            break;
         
        case 0x2307 :
            res = sAxis[0].sCurLoop.Ib*1000;
            break;
        
        case 0x2308 :
            res = sAxis[0].sCurLoop.Ic*1000;
            break;  
        
         case 0x2309 :
            res = sAxis[0].sCurLoop.Vdc*1000;
            break;
         
        case 0x230A :
            res = sAxis[0].sEncoder.SingleTurn;
            break;
        
        case 0x606C:
            res = sAxis[0].sSpdLoop.SpdFdb*10;
            break;    

        case 0x60FF:
            res = sAxis[0].sSpdLoop.SpdRefLit*10;
            break;         
       
        
        case 0x2800 :
            res = sAxis[1].sCurLoop.IdRef*1000;
            break;
        
        case 0x2801 :
            res = sAxis[1].sCurLoop.IqRef*1000;
            break;
        
        case 0x2802 :
            res = sAxis[1].sCurLoop.IdFdb*1000;
            break;
        
        case 0x2803 :
            res = sAxis[1].sCurLoop.IqFdb*1000;
            break;
        
        case 0x2804 :
            res = sAxis[1].sCurLoop.VdRef*1000;
            break;
        
        case 0x2805 :
            res = sAxis[1].sCurLoop.VqRef*1000;
            break;
        
         case 0x2806 :
            res = sAxis[1].sCurLoop.Ia*1000;
            break;
         
        case 0x2807 :
            res = sAxis[1].sCurLoop.Ib*1000;
            break;
        
        case 0x2808 :
            res = sAxis[1].sCurLoop.Ic*1000;
            break;  
        
         case 0x2809 :
            res = sAxis[1].sCurLoop.Vdc*1000;
            break;
         
        case 0x280A :
            res = sAxis[1].sEncoder.SingleTurn;
            break;
        
        case 0x686C:
            res = sAxis[1].sSpdLoop.SpdFdb*10;
            break;    

        case 0x68FF:
            res = sAxis[1].sSpdLoop.SpdRefLit*10;
            break;  
        
        default :
            break;
    }
    
    return res;
}
