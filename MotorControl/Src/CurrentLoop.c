/**********************************************************************
 *
 * FILE NAME:  CurrentLoop.c
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
#include "Param.h"
    
PRIVATE void CurrentRefSelect(struct AxisCtrlStruct *P);
PRIVATE void FhaseFind(struct AxisCtrlStruct *P);
PRIVATE void CurrentPICal(struct CurrentLoopStruct *P);
PRIVATE void SinCosCal(struct CurrentLoopStruct *P);
PRIVATE void SVPWMCal(struct CurrentLoopStruct *P);
//PRIVATE REAL32 RampCtrl (REAL32 in, REAL32 out, REAL32 rampDelta);

extern PUBLIC void ClearEncoderPulses(struct EncoderStruct *P, UINT16 AxisID);
extern PUBLIC void CurrentSampleTimeCal(struct CurrentLoopStruct *P,UINT16 AxisID);
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CurrentLoopInit(struct AxisCtrlStruct *P)
{
    struct CurrentLoopStruct *pCur = &P->sCurLoop;
 
	pCur->IdqKiPartMax = 0.99f;
	pCur->IdqKiPartMin = -0.99f;
	pCur->IdqPidOutMax = 0.99f;
	pCur->IdqPidOutMin = -0.99f;

	pCur->PWMPRD = PWM_PERIOD_VALUE;
	pCur->VDCinvTSQRT = pCur->PWMPRD >> 1;
	pCur->VDCinvTCon0 = pCur->VDCinvTSQRT * C_FSqrt3;
	pCur->Tonmin = C_Tonmin;
	pCur->Tonmax = pCur->PWMPRD - (pCur->Tonmin * 2);

    pCur->Cp = 0.1f;
	pCur->Ci = 0.01f;
    
	pCur->M_PeakI = 0.001f*(float)gParam[P->AxisID].MotorPeakCurrent0x220A;
	//pCur->M_RatedI = 0.001f*(float)gParam[P->AxisID].MotorRatedCurrent0x2209*1.414f;
	//pCur->M_RatedI = 0.001f*(float)gParam[P->AxisID].MotorRatedCurrent0x2209*1.414f;
	if(gMachineInfo.motorVersion == 4)
	{
			pCur->M_RatedI = 4.0f;
			gParam[P->AxisID].MotorRatedCurrent0x2209 = 4000;
	}
	else if((gMachineInfo.motorVersion == 1)||(gMachineInfo.motorVersion == 2))
	{
				pCur->M_RatedI = 5.5f;
				gParam[P->AxisID].MotorRatedCurrent0x2209 = 5500;
	}
	else if(gMachineInfo.motorVersion == 7)
	{
			pCur->M_RatedI = 4.5f;
			gParam[P->AxisID].MotorRatedCurrent0x2209 = 4500;
	}
	else
	{
			pCur->M_RatedI = 0.001f*(float)gParam[P->AxisID].MotorRatedCurrent0x2209*1.414f;
	}
	pCur->M_Ke = 0;
	pCur->M_Ld = 0;
	pCur->M_Lq = 0;
	pCur->M_Rs = 0.5f;

	pCur->IqRefMax = 0.001f*(float)gParam[P->AxisID].CurrentLimit0x2003;// ????????????
	pCur->IqRefMin = -pCur->IqRefMax;

	pCur->FF_IdRef = 0.5f;
	pCur->FF_DeltaId = 0.00001f;
	pCur->FF_StableCnt = 20000;
	pCur->FF_Cnt = 0;
	pCur->FF_Step = 0;
	pCur->FF_State = FF_FIRST_START;

	pCur->VF_DeltaAngle = 16;
	pCur->VF_VoltRef = 0.05f;
    
    pCur->IF_DeltaAngle = 16;
	pCur->IF_CurRef = 0.12f;

	pCur->IValidFdb = 0.0f;

	IIR1LPFParameterCal(&pCur->sTorFilter1, gParam[P->AxisID].CurRefFilterFc0x2109, CURRENT_PRD);
	// ????????
//	AveFilterResetState(&pCur->sIvalFilter1);
        
    for(int i=0;i<5;i++)
	{
		pCur->sIqRefFilter.Coeff[i] = 0.2f;
	}

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
float FF_elecAngle = 0.0f;
UINT16 IdCnt[2]={0};
PUBLIC void CurrentLoopExec(struct AxisCtrlStruct *P)
{
    struct CurrentLoopStruct *pCur = &P->sCurLoop;
	float IValid;
    
    if (0 == P->BootStrapCapChargeFlag)
    {
        //Reference source select
        CurrentRefSelect(P);
    }
    
    if(pCur->IdRef != 0.0f)
    {
        if(IdCnt[P->AxisID]++ > 50)
        {
            pCur->IdRef = 0.0f;
            IdCnt[P->AxisID] = 0;
        }
    }
    
    if(FF_CTRL == P->CtrlMode)
    {
        pCur->ElecAngle = FF_elecAngle;
        pCur->IdKiPart = 0;
		pCur->IqKiPart = 0;
    }

    //Iq reference limit
	if(pCur->IqRef > pCur->IqRefMax)
	{
		pCur->IqRef = pCur->IqRefMax;
	}
	else if(pCur->IqRef < pCur->IqRefMin)
	{
		pCur->IqRef = pCur->IqRefMin;
	}
    
	//Sine & Cosine calculate
	SinCosCal(pCur);
    
    pCur->Ic = 0.0f - pCur->Ia - pCur->Ib;
    
	//Clarke
	pCur->Ialfa = pCur->Ia;
	pCur->Ibeta = (2.0f * pCur->Ib + pCur->Ia) * C_FSqrt3Inv;
    
	//Park
	pCur->IdFdb = (pCur->Ialfa * pCur->CosTheta) + (pCur->Ibeta * pCur->SinTheta);
	pCur->IqFdb = (-pCur->Ialfa * pCur->SinTheta) + (pCur->Ibeta * pCur->CosTheta);

	// I Valid value // ????
//	pCur->IValidFdb = __sqrtf((pCur->Ialfa * pCur->Ialfa + pCur->Ibeta * pCur->Ibeta)/2);	
//	pCur->IValidFdb = AveFilter(&pCur->sIvalFilter1,pCur->IValidFdb,2000);// ????????200ms
	IValid = __sqrtf((pCur->Ialfa * pCur->Ialfa + pCur->Ibeta * pCur->Ibeta)/2);
	pCur->IValidFdb = pCur->IValidFdb + (IValid - pCur->IValidFdb)/16;
	 
	
	// pid calculation
	CurrentPICal(pCur);
    if(POWER_OFF == P->PowerFlag || P->CtrlMode < CUR_CTRL)
	{
		pCur->IdKiPart = 0;
		pCur->IqKiPart = 0;
	}

	//Open loop VF control
	if(VF_CTRL == P->CtrlMode)
	{
		pCur->IdKiPart = 0.0f;
		pCur->IqKiPart = 0.0f;
		pCur->VdRef = 0.0f;
		pCur->VqRef = pCur->VF_VoltRef;
	}
    
    if(POWER_ON == P->PowerFlag && (pCur->FF_State < FF_HAVE_CHECKED) && FF_CTRL == P->CtrlMode)
    {
        FhaseFind(P);
    }
    
	// inverse park
	pCur->Valfa = -pCur->VqRef * pCur->SinTheta + pCur->VdRef * pCur->CosTheta;
    pCur->Vbeta = pCur->VqRef * pCur->CosTheta + pCur->VdRef * pCur->SinTheta;
      
	// svpwm
	SVPWMCal(pCur);
	// Cal ADC Sample Time
	CurrentSampleTimeCal(pCur,P->AxisID);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CurrentRefSelect(struct AxisCtrlStruct *P)
{
    struct CurrentLoopStruct *pCur = &P->sCurLoop;
    
	switch(P->CtrlMode)
	{
		case VF_CTRL:
			pCur->ElecAngle = pCur->ElecAngle + pCur->VF_DeltaAngle;
//            pCur->ElecAngle = P->sEncoder.ElecAngle;
			pCur->ElecAngle = pCur->ElecAngle % C_SinUint;
		break;

        case IF_CTRL:
			pCur->ElecAngle = pCur->ElecAngle + pCur->IF_DeltaAngle;
			pCur->ElecAngle = pCur->ElecAngle % C_SinUint;
			pCur->IdRef = 0.0f;
			pCur->IqRef = pCur->IF_CurRef;
		break;
                
		case CUR_CTRL :
			pCur->ElecAngle = P->sEncoder.ElecAngle;
//			pCur->IdRef = 0.0f;
//			pCur->IqRef = 0;
		break;

        case INNER_SPD_CTRL:
		case SPD_CTRL :
			pCur->ElecAngle = P->sEncoder.ElecAngle;
			pCur->IdRef = 0.0f;
			pCur->IqRef = P->sEncoder.MotorDirection*P->sSpdLoop.Output*pCur->M_PeakI;
        
            // current reference filter
            if(P->sFilterCfg.bit.CurRefFilter)
                pCur->IqRef = FilterIIR1LPFExec(&pCur->sTorFilter1, pCur->IqRef);
		break;

		case POS_CTRL :
			pCur->ElecAngle = P->sEncoder.ElecAngle;
			pCur->IdRef = 0.0f;
			pCur->IqRef = P->sSpdLoop.Output*pCur->M_PeakI;
		break;
        
        case INNER_CUR_CTRL:
            pCur->ElecAngle = P->sEncoder.ElecAngle;
			pCur->IdRef = 0.0f;
			pCur->IqRef = P->sInnerCtrl.Output*0.001f;
        break;

		case FF_CTRL :
			if(POWER_ON == P->PowerFlag && (pCur->FF_State < FF_HAVE_CHECKED))
			{
//				FhaseFind(P);
			}
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
#if 0
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
#endif
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 * ????,????
***********************************************************************/
#define FHASE_FIND_VOLTAGE  0.05f;  // max is 0.98
PRIVATE void FhaseFind(struct AxisCtrlStruct *P)
{
    struct CurrentLoopStruct *pCur = &P->sCurLoop;
    
    pCur->VqRef = 0.0f;
    
	switch(pCur->FF_Step)
	{
		case 0:
			pCur->IdRef = 0;
			pCur->IqRef = 0;
			pCur->ElecAngle = 0;
			pCur->FF_Cnt = 0;
			pCur->FF_Step = 1;
		break;

		case 1:
			pCur->ElecAngle = FF_elecAngle;
//			pCur->IdRef = RampCtrl(pCur->FF_IdRef ,pCur->IdRef, pCur->FF_DeltaId);
            pCur->VqRef = 0.0f;
            pCur->VdRef = FHASE_FIND_VOLTAGE;
//			if(pCur->IdRef >= pCur->FF_IdRef)
//			{
				if(pCur->FF_Cnt++ >= pCur->FF_StableCnt)
				{
					pCur->FF_Cnt = 0;
					ClearEncoderPulses(&P->sEncoder, P->AxisID);
//					pCur->FF_Step = 2;
                    pCur->FF_Step = 4;
                    pCur->VdRef = 0.0f;
				}
//			}
		break;

		case 2:
			if(pCur->ElecAngle >= 5461)   // 5461 : 60 degree
			{
				pCur->ElecAngle = 5461;
				if(pCur->FF_Cnt++ >= pCur->FF_StableCnt)
				{
					if(P->sEncoder.ElecAngle > 5006      // 5006 : 55 degree
							&& P->sEncoder.ElecAngle < 5916)  // 5916 : 65 degree
					{
						pCur->FF_Step = 3;
					}
					else
					{
						pCur->FF_Step = 0;
						; // todo  error;
					}
					pCur->FF_Cnt = 0;
				}
			}
			else
			{
				pCur->ElecAngle = pCur->ElecAngle + 1;
			}
		break;

		case 3:
			if(pCur->ElecAngle <= 0)   // 0 degree
			{
				pCur->ElecAngle = 0;
				if(pCur->FF_Cnt++ >= pCur->FF_StableCnt)
				{
					if(P->sEncoder.ElecAngle > 455   // 455 :5 degree
							&& P->sEncoder.ElecAngle < 32313) // 32313 : 355 degree
					{
						pCur->FF_Step = 0;
						; // todo  error;
					}
					else
					{
						ClearEncoderPulses(&P->sEncoder, P->AxisID);
						pCur->FF_Step = 4;
					}
					pCur->FF_Cnt = 0;
				}
			}
			else
			{
				pCur->ElecAngle = pCur->ElecAngle - 1;
			}

		break;

		case 4:
//			pCur->ElecAngle = 0;
//			pCur->IdRef = RampCtrl(0 ,pCur->IdRef, pCur->FF_DeltaId);
//			if(pCur->IdRef <=0)
//			{
                pCur->VdRef = 0.0f;
				pCur->FF_Step = 0;
				pCur->FF_State = FF_HAVE_CHECKED;
                pCur->ElecAngle = 0;
//			}
		break;

		default :
		break;
	}
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CurrentPICal(struct CurrentLoopStruct *P)
{
	P->IdErr = P->IdRef - P->IdFdb;
	P->IqErr = P->IqRef - P->IqFdb;
 
	P->IqKpPart = P->Cp * P->IqErr;// +P->VoltForwardUqOut+P->RPMUqOut;
	P->IqKiPart = P->IqKiPart + P->Ci * P->IqErr;
    
	if(P->IqKiPart > P->IdqKiPartMax)
	{
		P->IqKiPart = P->IdqKiPartMax;
	}

	if(P->IqKiPart < P->IdqKiPartMin)
	{
		P->IqKiPart = P->IdqKiPartMin;
	}

	P->IqOutPreSat = P->IqKpPart + P->IqKiPart;

	if(P->IqKpPart > P->IdqPidOutMax)
	{
		P->VqRef = P->IdqPidOutMax;
		P->IqKiPart = 0.0f;
	}
	else if(P->IqKpPart < P->IdqPidOutMin)
	{
		P->VqRef = P->IdqPidOutMin;
		P->IqKiPart = 0.0f;
	}
	else if(P->IqOutPreSat > P->IdqPidOutMax)
	{
		P->VqRef = P->IdqPidOutMax;
//		P->IqKiPart = P->IdqPidOutMax - P->IqKpPart;
        P->IqKiPart = P->IqKiPart - P->Ci * P->IqErr;
	}
	else if(P->IqOutPreSat < P->IdqPidOutMin)
	{
		P->VqRef = P->IdqPidOutMin;
//		P->IqKiPart = P->IdqPidOutMin - P->IqKpPart;
        P->IqKiPart = P->IqKiPart - P->Ci * P->IqErr;
	}
	else
	{
		P->VqRef = P->IqOutPreSat;
	}

	P->IdKpPart = P->Cp * P->IdErr;

	P->IdKiPart = P->IdKiPart + P->Ci * P->IdErr;
	if(P->IdKiPart > P->IdqKiPartMax)
	{
		P->IdKiPart = P->IdqKiPartMax;
	}

	if(P->IdKiPart < P->IdqKiPartMin)
	{
		P->IdKiPart = P->IdqKiPartMin;
	}

	P->IdOutPreSat = P->IdKpPart + P->IdKiPart;

	if(P->IdKpPart > P->IdqPidOutMax)
	{
		P->VdRef = P->IdqPidOutMax;
		P->IdKiPart = 0.0f;
	}
	else if(P->IdKpPart < P->IdqPidOutMin)
	{
		P->VdRef = P->IdqPidOutMin;
		P->IdKiPart = 0.0f;
	}
	else if(P->IdOutPreSat > P->IdqPidOutMax)
	{
		P->VdRef = P->IdqPidOutMax;
//		P->IdKiPart = P->IdqPidOutMax - P->IdKpPart;
        P->IdKiPart = P->IdKiPart - P->Ci * P->IdErr;
	}
	else if(P->IdOutPreSat < P->IdqPidOutMin)
	{
		P->VdRef = P->IdqPidOutMin;
//		P->IdKiPart = P->IdqPidOutMin - P->IdKpPart;
        P->IdKiPart = P->IdKiPart - P->Ci * P->IdErr;
	}
	else
	{
		P->VdRef = P->IdOutPreSat;
	}

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
extern const REAL32 SinCTable[];
PRIVATE void SinCosCal(struct CurrentLoopStruct *P)
{
	if(P->ElecAngle > C_SinUint3D4)
	{
		P->SinTheta = -SinCTable[C_SinUint - P->ElecAngle];
		P->CosTheta = SinCTable[P->ElecAngle - C_SinUint3D4];
	}
	else if(P->ElecAngle > C_SinUint1D2)
	{
		P->SinTheta = -SinCTable[P->ElecAngle - C_SinUint1D2];
		P->CosTheta = -SinCTable[C_SinUint3D4 - P->ElecAngle];
	}
	else if(P->ElecAngle > C_SinUint1D4)
	{
		P->SinTheta = SinCTable[C_SinUint1D2 - P->ElecAngle];
		P->CosTheta = -SinCTable[P->ElecAngle - C_SinUint1D4];
	}
	else
	{
		P->SinTheta = SinCTable[P->ElecAngle];
		P->CosTheta = SinCTable[C_SinUint1D4 - P->ElecAngle];
	}
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SVPWMCal(struct CurrentLoopStruct *P)
{
	INT32 Temp0 = 0;
	REAL32 Va,Vb,Vc;
	INT16  Sector,X,Y,Z,T1,T2,Ta,Tb,Tc;

	Va = P->Vbeta;
	Vb = (-P->Vbeta + C_FSqrt3 * P->Valfa) * C_F1D2;
	Vc = (-P->Vbeta - C_FSqrt3 * P->Valfa) * C_F1D2;

    /* 60 degrees sector determination */
	Sector = 0;
	if ( Va > 0 )  { Sector += 1 ;}
	if ( Vb > 0 )  { Sector += 2 ;}
	if ( Vc > 0 )  { Sector += 4 ;}

	P->Sector = Sector;
	X = (INT16)(2 * P->Vbeta * P->VDCinvTSQRT);
	Y = (INT16)(P->Vbeta * P->VDCinvTSQRT + P->Valfa * P->VDCinvTCon0);
	Z = (INT16)(P->Vbeta * P->VDCinvTSQRT - P->Valfa * P->VDCinvTCon0);

    /* T1 and T2 calculation depending on the sector number */
	switch( Sector )
	{
		case 0:
			T1= -Y;
			T2= -Z;
			break;
		case 1:
			T1=  Z;
			T2=  Y;
			break;
		case 2:
			T1=  Y;
			T2= -X;
			break;
		case 3:
			T1= -Z;
			T2=  X;
			break;
		case 4:
			T1= -X;
			T2=  Z;
			break;
		case 5:
			T1=  X;
			T2= -Y;
			break;
		case 6:
			T1= -Y;
			T2= -Z;
			break;
		case 7:
			T1= -Y;
			T2= -Z;
			break;
	}

	if ((T1 - P->Tonmin) < 0 ) { T1 = P->Tonmin ; }
	if ((T2 - P->Tonmin) < 0 ) { T2 = P->Tonmin ; }

	Temp0= T1 + T2;
	if ((Temp0 - P->Tonmax) > 0 )
	{
		T1= (T1 * P->Tonmax)/Temp0;
		T2= (T2 * P->Tonmax)/Temp0;
	}
	
	Ta= ((P->PWMPRD- T1- T2) >> 1);
	Tb= Ta + T1;
	Tc= Tb + T2;

	switch( Sector )
	{

		case 0:
			P->TaNumber= Tb;
			P->TbNumber= Tc;
			P->TcNumber= Ta;
			break;
		case 1:
			P->TaNumber= Tb;
			P->TbNumber= Ta;
			P->TcNumber= Tc;
			break;
		case 2:
			P->TaNumber= Ta;
			P->TbNumber= Tc;
			P->TcNumber= Tb;
			break;
		case 3:
			P->TaNumber= Ta;
			P->TbNumber= Tb;
			P->TcNumber= Tc;
			break;
		case 4:
			P->TaNumber= Tc;
			P->TbNumber= Tb;
			P->TcNumber= Ta;
			break;
		case 5:
			P->TaNumber= Tc;
			P->TbNumber= Ta;
			P->TcNumber= Tb;
			break;
		case 6:
			P->TaNumber= Tb;
			P->TbNumber= Tc;
			P->TcNumber= Ta;
			break;
		case 7:
			P->TaNumber= Tb;
			P->TbNumber= Tc;
			P->TcNumber= Ta;
			break;
        default :
            break;
	}
    
  
}



