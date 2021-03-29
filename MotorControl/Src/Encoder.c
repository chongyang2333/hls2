/*******************************************************************
 *
 * FILE NAME:  Encoder.c
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
#include "ControlRun.h"
#include "HardApi.h"
#include "Param.h"

#define C_EncoderShift  16

PRIVATE void IncEncoderCal(struct EncoderStruct *P);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void EncoderInit(struct AxisCtrlStruct *P)
{
    struct EncoderStruct *pEnc = &P->sEncoder;
    
	pEnc->Model = 0;
	pEnc->Pulse = 0;
	pEnc->PulseINC = 0;
	pEnc->CNTPulseOld = 0;
	pEnc->PulseMax = gParam[P->AxisID].EncoderPPR0x2202;
//    pEnc->PulseMax = 4096;
	pEnc->MechAngle = 0;
	pEnc->ElecAngle = 0;
	pEnc->PolePairs = gParam[P->AxisID].MotorPolePairs0x2201;
//    pEnc->PolePairs = 10;
    
    pEnc->PulsePerElecPRD = pEnc->PulseMax/pEnc->PolePairs ;

//	pEnc->MechAngleOffset = 0;
	pEnc->SinUnit = C_SinUint;		

    pEnc->HallEnable = gParam[P->AxisID].HallEnable0x2203;
//    pEnc->HallEnable = 1;.

    if(gParam[P->AxisID].MotorInverse0x2208)
    {
        pEnc->MotorDirection = -1;
    }
    else
    {
        pEnc->MotorDirection = 1;
    }
 
    pEnc->HallState = GetHallState(P->AxisID, gMachineInfo.motorVersion);
    pEnc->HallStateLast = pEnc->HallState;
    pEnc->HallStateBuffer = pEnc->HallState;
    pEnc->HallDebounceCnt = 0;
    
}

/***********************************************************************
 * DESCRIPTION:encoder calculation.
 *
 * RETURNS:
 *
***********************************************************************/
#define HALL_DEBOUNCE_AMOUNT  5
PUBLIC void GetEncoderPulse(struct EncoderStruct *P, UINT16 AxisID)
{
	P->Pulse = GetIncEncoderPulse(AxisID);
    
    // hall debounce
    UINT16  HallStateNew = GetHallState(AxisID, gMachineInfo.motorVersion);
    if(HallStateNew != P->HallState)
    {
        if(HallStateNew == P->HallStateBuffer)
        {
            if(P->HallDebounceCnt >= HALL_DEBOUNCE_AMOUNT)
            {
                P->HallState = P->HallStateBuffer;
                P->HallDebounceCnt = 0;
            }
            else
            {
                P->HallDebounceCnt++;
            }
        }
        else
        {
            P->HallStateBuffer = HallStateNew;
            P->HallDebounceCnt = 0;
        }
    }
    else
    {
        P->HallDebounceCnt = 0;
    }
    
}

/***********************************************************************
 * DESCRIPTION:encoder calculation.
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AbsEncoderReadStart(void)
{
	;
}

/***********************************************************************
 * DESCRIPTION:encoder calculation.
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void EncoderCalExec(struct AxisCtrlStruct *P)
{
    INT32 offset = 0;
    INT32 tmp = 0;
    
    struct EncoderStruct *pEnc = &P->sEncoder; 
    
    IncEncoderCal(pEnc);
    
    if(pEnc->HallEnable)
    {
        // UVW Hall Alarm
        if(pEnc->HallState > 6)
        {
            return ;
        }
        else
        {
            if(FF_FIRST_START == P->sCurLoop.FF_State)
            {
                pEnc->MechAngle =(pEnc->HallState-1)*pEnc->PulsePerElecPRD/6; 
                
                P->sCurLoop.FF_State = FF_WAIT_CHECK;
            }
            else
            {   
                //务必确认这个校正点是HALL下降沿触发的,原因是HALL上升沿被端口的滤波电容滤波的上升沿很缓慢，可能会导致电气角的校正是有问题的
                if(pEnc->HallState == 1 && pEnc->HallStateLast == 6)
                {
                    /*Single Turn Check*/
                    if (pEnc->VirtualPhaseZTrig)
                    {
                        P->sCurLoop.FF_State = FF_WAIT_CHECK;
                        pEnc->VirtualPhaseZTrig = 0;
                    }
                    
                    if(P->sCurLoop.FF_State < FF_HAVE_CHECKED)
                    {
                        //将机械角直接校准为电气角此时理论值，设定为330度
                        if (pEnc->MechAngle > pEnc->PulsePerElecPRD)
                        {
                            pEnc->MechAngle = pEnc->PulseMax - pEnc->PulsePerElecPRD/12;
                        }
                        else
                        {
                            pEnc->MechAngle = pEnc->PulsePerElecPRD*11/12;
                        }

                        P->sCurLoop.FF_State = FF_HAVE_CHECKED;
                    }  
                    else   
                    {
                        //计算电气角
                        tmp = pEnc->MechAngle % pEnc->PulsePerElecPRD;
                        
                        UINT16 E_Angle_15_Tmp = 15.0f/360*pEnc->PulsePerElecPRD;
                        UINT16 E_Angle_285_Tmp = 285.0f/360*pEnc->PulsePerElecPRD;
                        /*电气角和电气角理论值(330度)的偏差超过45度报警，
                          即电气角值为15度到285度时报警*/
                        if(tmp > E_Angle_15_Tmp && tmp < E_Angle_285_Tmp)
                        {
                            // Encoder counter alarm
                            P->sAlarm.ErrReg.bit.EncCount = 1;
                        }
                    }    

                }
                else if(pEnc->HallState == 1 && pEnc->HallStateLast==2)
                {     
                    /*Single Turn Check*/
                    if (pEnc->VirtualPhaseZTrig)
                    {
                        P->sCurLoop.FF_State = FF_WAIT_CHECK;
                        pEnc->VirtualPhaseZTrig = 0;
                    }
                    
                    if(P->sCurLoop.FF_State < FF_HAVE_CHECKED) 
                    {
                        //将机械角直接校准为电气角此时理论值，设定为30度
                        if (pEnc->MechAngle > pEnc->PulsePerElecPRD)
                        {
                            pEnc->MechAngle = pEnc->PulseMax - pEnc->PulsePerElecPRD*11/12;
                        }
                        else
                        {
                            pEnc->MechAngle = pEnc->PulsePerElecPRD/12;
                        }
                        
                        P->sCurLoop.FF_State = FF_HAVE_CHECKED; // 校准完成
                    }  
                    else   
                    {
                        //计算电气角
                        tmp = pEnc->MechAngle % pEnc->PulsePerElecPRD;
                        
                        UINT16 E_Angle_75_Tmp = 75.0f/360*pEnc->PulsePerElecPRD;
                        UINT16 E_Angle_345_Tmp = 345.0f/360*pEnc->PulsePerElecPRD;                        
                        /*电气角和电气角理论值(30度)的偏差超过45度报警，
                          即电气角值为75度到345度时报警*/
                        if(tmp > E_Angle_75_Tmp && tmp < E_Angle_345_Tmp)
                        {
                            // Encoder counter alarm
                            P->sAlarm.ErrReg.bit.EncCount = 1;
                        }
                    } 
                }                            
            }//end if(FF_FIRST_START ==
            
            pEnc->HallStateLast = pEnc->HallState;
        }//end if(pEnc->HallState > 6)
        
    }// end if(pEnc->HallEnable)

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ClearEncoderPulses(struct EncoderStruct *P, UINT16 AxisID)
{
	P->Pulse = 0;
//    P->MechAngleOffset = 0;
    ClearIncEncoderPulse(AxisID);

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void EncoderParUpdate(struct EncoderStruct *P)
{


}


/***********************************************************************
 * DESCRIPTION:increment encoder calculation.
 *
 * RETURNS: update ElecAngle, MechAngle, pos,spd
 *
***********************************************************************/
PRIVATE void IncEncoderCal(struct EncoderStruct *P)
{
	INT32 Tmp = 0;
	Tmp = P->Pulse << C_EncoderShift;
	P->PulseINC = (Tmp - P->CNTPulseOld) >> C_EncoderShift;

	P->CNTPulseOld  = Tmp;
    
	// should be clear SpeedInc in speed loop
	P->SpdFdbPulseInc  = P->SpdFdbPulseInc + P->MotorDirection*P->PulseINC;

	P->PosFdbPulse = P->PosFdbPulse + P->MotorDirection*P->PulseINC;

	P->MechAngle = P->MechAngle + P->PulseINC;

	if(P->MechAngle > P->PulseMax)
	{
		P->MechAngle = P->MechAngle - P->PulseMax;
        P->MultiTurn++;
        P->VirtualPhaseZTrig = 1;
	}
    else if(P->MechAngle < 0)
	{
		P->MechAngle = P->MechAngle + P->PulseMax;
        P->MultiTurn--;
        P->VirtualPhaseZTrig = 1;
	}
    
    P->SingleTurn = P->MechAngle;
    
//    INT32 mechAngle = P->MechAngle;

//	if(mechAngle < 0)
//	{
//		mechAngle = mechAngle + P->PulseMax;
//	}

	Tmp = (P->MechAngle * P->PolePairs) % P->PulseMax;
	P->ElecAngle = (Tmp * P->SinUnit) / P->PulseMax;
    
//    UINT16 HallState = GetHallState();
//    
//    P->ElecAngle = ((float)HallState)*0.166667f *P->SinUnit;
    

}


