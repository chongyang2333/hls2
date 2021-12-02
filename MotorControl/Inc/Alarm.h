/*******************************************************************
 *
 * FILE NAME:  Alarm.h
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
#ifndef _Alarm_H_
#define _Alarm_H_

#include "UserDataTypes.h"


struct AlarmBits  // bits   description
{   		    
    UINT32 HardCurOver:1;  
    UINT32 BusCurOver:1;    
	UINT32 VdcOver:1;
    UINT32 VdcUnder:1;
    UINT32 TempOver:1;	  
	UINT32 EncCount:1;		
	UINT32 AbzBreak:1;
	UINT32 HallErr:1;           
	
    UINT32 SpdOver:1;
    UINT32 MotorStall:1;
    UINT32 MotorIIt:1;
    UINT32 SpeedFollow:1;
	UINT32 CanLost:1;	
    UINT32 CanBreakErr:1; 
    UINT32 PartnerErr:1;        // Indicate the other axis error
    UINT32 FatalErr:1;            
    
    // Fatal Error bit
    UINT32 CurInit:1;      	    // can't be reset by soft clear	
	UINT32 InnerErr:1;			// can't be reset by soft clear	
	UINT32 EepromErr:1;         // can't be reset by soft clear	
	UINT32 OutLosePhase:1;
	UINT32 PhaseCurOver:1;
	UINT32 MotorTempOver:1;
	UINT32 MosTempOver:1;
    UINT32 VbusSSMosOpenCircuitFailure:1; // Mos for Soft Start Open Circuit Failure
    UINT32 Rsv1:1;
    UINT32 PwmoutBreak:1;
    
	UINT32 Rsv31:5;
	UINT32 StutterStop:1;
};

union AlarmReg {
	UINT32             all;
    struct AlarmBits   bit;
};

struct AlarmStruct
{
	union AlarmReg  ErrReg;
  
    REAL32  I2T_Setpoint;
    REAL32  I2T_accumulator;
    
    REAL32  VdcMax;
    REAL32  VdcMin;
    REAL32  VdcWarn;
    
    REAL32  MotorMaxSpd;
    
    INT16   TempLimitMax;
    
    UINT16  VdcOverCnt;
    UINT16  VdcUnderCnt;
    UINT16  VdcDischargeFlag;
    UINT16  SpdOverCnt;
    UINT16  SpdFollwCnt;
    UINT16  StallCnt;
    
    UINT16  TempOverCnt;
	UINT16  MosTempOverCnt;
	UINT16  MotorTempOverCnt;
    
    UINT16  HallErrCnt;
    UINT16  BusCurOverCnt;
	UINT16  EmergencyStopRstCnt;
    UINT16  EmergencyStopSetCnt;
    
    UINT16  PhaseCurrentLimit;

	UINT16  IuOverCnt;
	UINT16  IvOverCnt;
	UINT16  IwOverCnt;

	REAL32  IuTotle;			 //  u相电流累加值
	REAL32  IvTotle;			 //  v相电流累加值
	REAL32  IwTotle;			 //  w相电流累加值
	REAL32  phaselose_time;		 //  缺相累加值检测时间
	UINT16  PwmoutDisconnectCnt;
	UINT16  phaselose_cnt;
};



#endif
