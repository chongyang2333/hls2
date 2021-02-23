/*******************************************************************
 *
 * FILE NAME:  ControlRun.h
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

#ifndef SOURCES_CONTROL_RUN_H_
#define SOURCES_CONTROL_RUN_H_

#include "UserDataTypes.h"
#include "Filter.h"
#include "Encoder.h"
#include "Alarm.h"


/*------------------------- Public Constants ----------------------*/
#define MAX_AXIS_NUM        2

#define AXIS_LEFT           0
#define AXIS_RIGHT          1

#define POWER_ON            1
#define POWER_OFF           0

#define VF_CTRL             0   
#define IF_CTRL             1
#define FF_CTRL             2
#define CUR_CTRL            3
#define SPD_CTRL            4
#define POS_CTRL            5
#define INNER_CUR_CTRL      6
#define INNER_SPD_CTRL      7

#define PWM_PERIOD_VALUE   (10800)  /* PWM Period Value  100us */
#define SYSTEM_FRQ          216000000

#define PWM_FRQ             (SYSTEM_FRQ/PWM_PERIOD_VALUE/2)
#define CURRENT_FRQ         PWM_FRQ
//#define SPEED_FRQ           CURRENT_FRQ/5
//#define POSITION_FRQ        SPEED_FRQ
#define SPEED_FRQ           CURRENT_FRQ/2
#define POSITION_FRQ        CURRENT_FRQ/5



#define CURRENT_PRD         (1.0f/(float)CURRENT_FRQ)
//#define SPEED_PRD           (5.0f*CURRENT_PRD)
//#define POSITION_PRD        SPEED_PRD
#define SPEED_PRD           (2.0f*CURRENT_PRD)
#define POSITION_PRD        (5.0f*CURRENT_PRD)


#define PI                  3.14159265f
#define C_FSqrt3   	        1.732050807568877f		//Sqrt(3)
#define C_FSqrt3Inv   	    (1.0f/C_FSqrt3)		//1/Sqrt(3)
#define C_FSqrt3D2          0.8660254037844386f		//Sqrt(3)/2
#define C_F1D2  	        0.5f
#define C_Tonmin            (108*1)  //1us

#define C_SinUint 		    32768
#define C_SinUint1D8        (C_SinUint>>3)
#define C_SinUint1D4        (C_SinUint>>2)
#define C_SinUint1D2        (C_SinUint>>1)
#define C_SinUint3D4        ((C_SinUint>>2)*3)

#define MAX_TIMER_ISR_TIME   (15000*27)  // 15MS
#define MAX_PWM_ISR_TIME     (80*27)     // Max is 100us

#define VBUS_VOLTAGE_CLIMB_SLOPE_CONDITION  0.03f
#define VBUS_VOLTAGE_CONDITION  18.0f
#define DEBUG_VBUS_SOFT_START   (0)

/*------------------------- Current Loop Struct ----------------------*/
struct CurrentLoopStruct
{
	REAL32	Ia;                  // phase a actual current
	REAL32	Ib;                  // phase b actual current
	REAL32	Ic;                  // phase c actual current
	REAL32	Ialfa;               // Ialfa actual current
	REAL32	Ibeta;               // Ibeta actual current
    
    REAL32  Idc;                 // DC current
    REAL32  Vdc;                 // DC voltate

    REAL32	IdRef;               //  Id Reference
    REAL32	IqRef;               //  Iq Reference
    REAL32	IqRefOld;            //  Last cycle Iq Reference

    REAL32  IqRefMax;            // Iq reference positive limit
    REAL32  IqRefMin;            // Iq reference negetive limit

    REAL32	IdFdb;               //  Id Feedback
    REAL32	IqFdb;               //  Iq Feedback
    REAL32	IdErr;               //  Id following error
    REAL32	IqErr;               //  Iq following error
    REAL32	IValidFdb;           //  I Valid value Feedback  // ��Чֵ

    REAL32	Cp;                  //  current loop proportion gain
    REAL32	Ci;                  //  current loop intigration gain
    REAL32	IdKpPart;			 //  pid_Id proportion part
	REAL32	IqKpPart;			 //  pid_Iq proportion part
	REAL32	IdKiPart;			 //  pid_Id intigration part
    REAL32	IqKiPart;            //  pid_Iq intigration part

    REAL32	IdqKiPartMax;		 //  pid(Id\Iq) intigration part Max value
    REAL32	IdqKiPartMin;		 //  pid(Id\Iq) intigration part Min value
    REAL32	IdqPidOutMax;		 //  pid(Id\Iq) output Max value
    REAL32	IdqPidOutMin;		 //  pid(Id\Iq) output Min value

    REAL32	IdOutPreSat;         //  pid_Id output value (not limitted by IdqPidOutMax)
    REAL32	IqOutPreSat;         //  pid_Iq output value (not limitted by IdqPidOutMax)
    REAL32	VdRef;				 //  pid_Id output value is VdRef (limitted by IdqPidOutMax)
    REAL32	VqRef;               //  pid_Iq output value is VqRef (limitted by IdqPidOutMax)

    REAL32	M_Ke;			     //  back EMF constant ke
    REAL32  M_Kt;                //  torque constant kt
    REAL32	M_Rs;                //  motor phase resistance
    REAL32	M_Lq;                //  motor q-axis inductance
    REAL32	M_Ld;                //  motor d-axis inductance
    REAL32  M_J;                 //  motor inertia
    REAL32	M_RatedI;            //  motor rated current
    REAL32  M_PeakI;             //  motor peak current

    INT32   ElecAngle;           // electric angle
	REAL32	SinTheta;            // sin(ElecAngle) for park
	REAL32	CosTheta;            // cos(ElecAngle) for park
	REAL32	Valfa;               // alfa-axis voltage
	REAL32	Vbeta;               // beta-axis voltage

	/* SVPWM Module */
	INT16	TaNumber;            // SVPWM output for timer compare value
	INT16	TbNumber;            // SVPWM output for timer compare value
	INT16	TcNumber;            // SVPWM output for timer compare value
	INT16	Tonmin;              // PWM on min value (timer counter)
	INT16	Tonmax;              // PWM on max value (timer counter)
	INT16	PWMPRD;              // PWM period value (timer counter)
	REAL32	VDCinvTSQRT;         // for svpwm
	REAL32	VDCinvTCon0;         // for svpwm

	REAL32  FF_IdRef;
	REAL32  FF_DeltaId;
	UINT32  FF_StableCnt;
	UINT32  FF_Cnt;
	UINT16  FF_Step;
	UINT16  FF_State;

	REAL32  VF_VoltRef;
	REAL32  VF_DeltaAngle;
    
    REAL32  IF_CurRef;
	REAL32  IF_DeltaAngle;

	struct IIR1LPFStruct sTorFilter1;
//	struct AverageFilter sIvalFilter1;
    struct FIR8Struct    sIqRefFilter;
  
};


/*------------------------- Speed Loop Struct ----------------------*/
struct SpeedLoopStruct
{
	REAL32	SpdRef;         // Input: Reference input
    REAL32	SpdRefLit;
	REAL32	SpdRefOld;
	REAL32	SpdFdb;         // Input: Feedback input
	REAL32	SpdErr;         // Variable: Error
	REAL32	Vp;             // Parameter: Proportional gain
	REAL32	Vi;             // Parameter: Integral gain
	REAL32  Kpdff;          // Parameter: pdff gain
	REAL32	SpdKpPart;      // Variable: Proportional output
	REAL32	SpdKiPart;      // Variable: Integral output
	REAL32	OutPreSat;      // Variable: Pre-saturated output
	REAL32	OutMax;         // Parameter: Maximum output
	REAL32	OutMin;         // Parameter: Minimum output
	REAL32	Output;         // Output: spd loop output

	REAL32  SpdRefMax;      // speed reference positive limit
	REAL32  SpdRefMin;      // speed reference negetive limit

	REAL32  AccMax;         // speed reference accerlation limit
	REAL32  DecMax;         // speed reference decerlation limit

	REAL32  IncToRpmUnit;   // encoder pulse unit to RPM unit

	REAL32  Jtotal;
	REAL32  TorFF;
	REAL32  TorFFGain;
    
    REAL32  DisturGain;
    REAL32  DisturFilterCoff;
    REAL32  FilteredDisturComp;
    REAL32  DisturbanceComp;
    REAL32  SpdRefNoFilter;
	REAL32  SpdRefActul;     // ʵ��s���߿��ƺ���ٶ�

	struct IIR1LPFStruct  sTorFFFilter;
	struct IIR1LPFStruct  sSpdRefFilter;
	struct IIR1LPFStruct  sSpdFdbFilter;

};
struct  CurveParaStruct
{
	REAL32 Acc;				// ���ٶ�
	REAL32 CoeffA1;			// ����ʽϵ��
    REAL32 CoeffA2;
	REAL32 CoeffA3;
	REAL32 CoeffA4;
	REAL32 CoeffA5;
	REAL32 Counts;
	REAL32 SpdRefInc;		// �����ٶȱ仯��
	REAL32 SpdRefActulOld;
	REAL32 SpdRefActulBak;

};


/*------------------------- Position Loop Struct ----------------------*/
struct PositionLoopStruct
{
	INT32 	PosFdb;			    //position feedback.  pulse unit
	INT32 	PosRef;			    //position reference.  pulse unit
    INT32 	PosRefLit;	
	INT32 	PosRefOld;
	INT32 	PosErr;				//position following error
	INT32   PosErrMax;          //position max following error
    UINT32  Acc;
	REAL32	Pp;				    //position loop proportion gain

	REAL32 	Output;				//position loop output. rpm unit

	UINT16  SpdFFSelect;
	REAL32  SpdFF;
	REAL32  SpdFFGain;
	struct IIR1LPFStruct  sSpdFFFilter;
	struct IIR1LPFStruct  sPosRefFilter;

};

struct InnerCtrlStruct
{
    UINT16  En;
    UINT16  Type;
    UINT16  Cycle;
    UINT32  Period;  // uint : pwm period
    UINT32  Tick;
    REAL32  Amp;
    REAL32  Offset;
    REAL32  Output;
};

struct EncoderStruct
{
	INT32		Model;			    // encoder type
	INT32		Pulse;              // encoder feedback pulse
	INT32		PulseINC;		    // current pulse - last pulse
	INT32		CNTPulseOld;        // last cycle pulse
	INT32		PulseMax;           // total pulse per resolution
    INT32       PulsePerElecPRD;    // pulse per electrical period
	INT32		MechAngle;		    // mechanical angle (0--PulseMax)
	INT32		ElecAngle;		    // electrical angle (0--SinUnit)
	INT32		PolePairs;	        // pole pairs number
	INT32		MechAngleOffset;    // mechanical angle offset
	INT32		SinUnit;            // electric angle max value
	INT32       PosFdbPulse;        // position feedback(pulse unit)
	INT32       SpdFdbPulseInc;     // speed feedback delta pulse(pulse unit)
    
    INT32       SingleTurn;
    INT32       MultiTurn;
    UINT8       VirtualPhaseZTrig;
   
    UINT16      HallEnable;
    UINT16      HallState;
    UINT16      HallStateBuffer;
    UINT16      HallDebounceCnt;
    UINT16      HallStateLast;
    
    INT16       MotorDirection;     // 1 for positive, -1 for negagive
};

struct FilterCfgBits {   		    // bits   description
	UINT16 PosRefFilter:1;	    //0----
	UINT16 SpdRefFilter:1;      //1-----
	UINT16 SpdFdbFilter:1;	    //2--
	UINT16 CurRefFilter:1; 		//3--
	UINT16 Rsv6: 12;            //4 --- 15
};

union FilterCfgReg {
	UINT16                 all;
    struct FilterCfgBits   bit;
};

/*------------------------- Axis Struct ----------------------*/
struct AxisCtrlStruct 
{
    UINT16 AxisID;
    UINT16 BootStrapCapChargeEn;
    UINT16 BootStrapCapChargeFlag;
    UINT32 BootStrapCapChargeStartTime;
    UINT32 BootStrapCapChargeTime; //unit: 100us
    UINT16 StarSealProtectEn;      //0:LoopClosed 1:StarSeal 2:Free
    UINT16 StarSealProtectFlag;
    UINT32 StarSealProtectStartTime;
    UINT32 StarSealProtectTime;        //unit:100us
    
    UINT16 PowerEn;            // control pwm on/off. 1:pwm on;  0:pwm off
	UINT16 PowerFlag;          // indicate pwm state. 1:pwm on;  0:pwm off
	UINT16 CtrlMode;           // control mode: vf,cur,spd,pos,pf
	union  FilterCfgReg        sFilterCfg;
	struct PositionLoopStruct  sPosLoop;
	struct SpeedLoopStruct     sSpdLoop;
	struct CurrentLoopStruct   sCurLoop;
	struct EncoderStruct       sEncoder;
	struct AlarmStruct         sAlarm;
    struct InnerCtrlStruct     sInnerCtrl;
	struct CurveParaStruct	   CurvePara;
};

struct SchedulerStruct
{
    UINT32 SchNum;
	UINT32 TickCnt;
	UINT32 IsrElapsedTime;
    UINT32 TimeStamp;
    UINT32 Tim7IsrTime;
};

/* Motion Control Reference Struct */
struct McRefStruct  
{
    UINT16  PowerOn;
    UINT16  ModeRef;
    REAL32  PosRef;
    REAL32  SpdRef;
    REAL32  CurRef;
};

/* Motion Control Feedback Struct */
struct McFdbStruct
{
    UINT16  StatusWord;
    UINT16  ModeFdb;
    REAL32  PosFdb;
    REAL32  SpdFdb;
    REAL32  CurFdb; 
};


extern struct SchedulerStruct sScheduler;

extern PUBLIC void AlarmLoop(struct AxisCtrlStruct *P);

extern PUBLIC UINT32 GetIsrElapsedTime(void);
extern PUBLIC void MC_SetReference(struct McRefStruct *pRef, UINT16 AxisID);
extern PUBLIC void MC_GetFeedback(struct McFdbStruct *pFdb, UINT16 AxisID);
extern PUBLIC UINT16 MC_GetErrorReg(UINT16 AxisID);
extern PUBLIC INT32 DataCollectGetValue(UINT16 Index);

extern PUBLIC void ControlRunInit(void);
extern PUBLIC void ControlRunExec(void);
extern PUBLIC void TimerIsrExec(void);

PUBLIC void EncoderCalExec(struct AxisCtrlStruct *P);

#endif /* SOURCES_CONTROL_RUN_H_ */
