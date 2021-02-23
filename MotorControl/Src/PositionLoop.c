/**********************************************************************
 *
 * FILE NAME:  PositionLoop.c
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

PRIVATE void PositionRefSelect(struct AxisCtrlStruct *P);
PRIVATE REAL32 RampCtrl (REAL32 in, REAL32 out, REAL32 rampDelta);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PositionLoopInit(struct AxisCtrlStruct *P)
{
    struct PositionLoopStruct *pPos = &P->sPosLoop;
    
    pPos->Acc = 1;
	pPos->PosErrMax = 5000;
	pPos->Pp = 0.05f;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PositionLoopExec(struct AxisCtrlStruct *P)
{
	INT32 Tmp;
	REAL32 Tmp2;
    struct PositionLoopStruct *pPos = &P->sPosLoop;
    
    if (0 == P->BootStrapCapChargeFlag)
    {
        //Reference source select
        PositionRefSelect(P);
    }
    
	// update position feedback
	pPos->PosFdb = P->sEncoder.MultiTurn*P->sEncoder.PulseMax + P->sEncoder.SingleTurn;
    
	// position error
	pPos->PosErr = pPos->PosRefLit - pPos->PosFdb;
    
    if(pPos->PosErr > pPos->PosErrMax)
    {
        pPos->PosErr = pPos->PosErrMax;
    }

	if(1 == pPos->SpdFFSelect)
	{
		Tmp = pPos->PosRefLit - pPos->PosRefOld;
		Tmp2 = Tmp*POSITION_FRQ*P->sSpdLoop.IncToRpmUnit;
		pPos->SpdFF = FilterIIR1LPFExec(&pPos->sPosRefFilter, Tmp2);
	}
	else if(2 == pPos->SpdFFSelect)
	{

	}
	else
	{
		pPos->SpdFF = 0.0f;
	}

	pPos->Output = pPos->PosErr* pPos->Pp + pPos->SpdFF* pPos->SpdFFGain;

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PositionRefSelect(struct AxisCtrlStruct *P)
{
 //   struct PositionLoopStruct *pPos = &P->sPosLoop;
    
	switch(P->CtrlMode)
	{
		case VF_CTRL:

		break;

		case CUR_CTRL :

		break;

		case SPD_CTRL :

		break;

		case POS_CTRL :
            P->sPosLoop.PosRefLit = RampCtrl(P->sPosLoop.PosRef, P->sPosLoop.PosRefLit, P->sPosLoop.Acc);

		break;

		case FF_CTRL :

		break;

		default:
		break;
	}
}

/***********************************************************************
 * DESCRIPTION: Ramp control
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE REAL32 RampCtrl (REAL32 in, REAL32 out, REAL32 rampDelta)
{
    REAL32 err;
    err = in - out;
    if (err > rampDelta)
        return(out + rampDelta);
    else if (err < -rampDelta)
        return(out - rampDelta);
    else
        return(in);

}
