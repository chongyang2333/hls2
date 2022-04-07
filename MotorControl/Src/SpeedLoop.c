/**********************************************************************
 *
 * FILE NAME:  SpeedLoop.c
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
#include "math.h"

#define SpeedPeriod    (SPEED_FRQ/10)	//500.0f  //  每个周期

#define S_curve		0	// s曲线加减速
#define Trapezoid	1	// 梯形曲线加减速
#define Quintics	0	// 五次曲线

PRIVATE void SpeedRefSelect(struct AxisCtrlStruct *P);
PRIVATE void SpeedPICal(struct SpeedLoopStruct *P);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void SpeedLoopInit(struct AxisCtrlStruct *P)
{
    struct SpeedLoopStruct *pSpd = &P->sSpdLoop;
    struct CurveParaStruct *pCurve = &P->CurvePara;

    pSpd->Vp = 0.008;
    pSpd->Vi = 0.00001;
    pSpd->Kpdff = 1.0f;

    pSpd->TorFFGain = (float)gParam[P->AxisID].TorffGain0x210B*0.1f;

    pSpd->OutMax = 1.0f;
    pSpd->OutMin = -1.0f;

    pSpd->SpdRefMax = gParam[P->AxisID].SpeedLimit0x2002;
    pSpd->SpdRefMin = -pSpd->SpdRefMax;

    pSpd->AccMax = 0.1f;
    pSpd->DecMax = 0.1f;

    pSpd->IncToRpmUnit = (60.0f*SPEED_FRQ)/((float)P->sEncoder.PulseMax);

    pSpd->DisturGain = (float)gParam[P->AxisID].DisturGainFc0x210D*0.01f;
    pSpd->DisturFilterCoff = (float)gParam[P->AxisID].DisturFilterFc0x210C*2*PI*SPEED_PRD;

    pCurve->Acc = 0.0f;	//
    pCurve->Counts = 0.0f; //

    IIR1LPFParameterCal(&pSpd->sSpdRefFilter, gParam[P->AxisID].SpdRefFilterFc0x2107, SPEED_PRD);
    IIR1LPFParameterCal(&pSpd->sSpdFdbFilter, gParam[P->AxisID].SpdFdbFilterFc0x2108, SPEED_PRD);
    IIR1LPFParameterCal(&pSpd->sTorFFFilter, gParam[P->AxisID].TorffFilterFc0x210A, SPEED_PRD);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
float Jtmp = 0.0003125f;// 0.00285  �����ṩֵ
PUBLIC void SpeedLoopExec(struct AxisCtrlStruct *P)
{
    struct SpeedLoopStruct *pSpd = &P->sSpdLoop;
    struct CurveParaStruct *pCurve = &P->CurvePara;

    if (0 == P->BootStrapCapChargeFlag) // �ǳ��״̬
    {
        //Reference source select
        SpeedRefSelect(P);
    }

    //Speed feedback calculation
    float RawSpdFdb = pSpd->IncToRpmUnit*P->sEncoder.SpdFdbPulseInc;
    P->sEncoder.SpdFdbPulseInc = 0;

//    pSpd->SpdFdb = FilterFIR8Exec(&pSpd->sSpdFdbFilter2, RawSpdFdb);

    //Speed feedback filter
    if(P->sFilterCfg.bit.SpdFdbFilter)
        pSpd->SpdFdb = FilterIIR1LPFExec(&pSpd->sSpdFdbFilter, RawSpdFdb);

    //Speed reference limit
    // ��ֵ�˲�
    //pSpd->SpdFdb = AveFilter(&pSpd->sSpdFdbFilter4,pSpd->SpdFdb,10);
//	if(S_curve || Quintics)  //s�ͻ���������߼Ӽ���
    {
        if(pSpd->SpdRefActul > pSpd->SpdRefMax)
        {
            pSpd->SpdRefActul = pSpd->SpdRefMax;
        }
        else if(pSpd->SpdRefActul < pSpd->SpdRefMin)
        {
            pSpd->SpdRefActul = pSpd->SpdRefMin;
        }
        pSpd->SpdRefLit = pSpd->SpdRefActul;
    }
    /*	else //Trapezoid T�ͼӼ���
    	{
    		if(pSpd->SpdRef > pSpd->SpdRefMax)
    		{
    			pSpd->SpdRef = pSpd->SpdRefMax;
    		}
    		else if(pSpd->SpdRef < pSpd->SpdRefMin)
    		{
    			pSpd->SpdRef = pSpd->SpdRefMin;
    		}
    		//pSpd->SpdRefActul = pSpd->SpdRef;
    		pSpd->SpdRefLit = pSpd->SpdRef;
    	}*/

    // pid calculation

    // disturbance obserer and compensation  ת�ع۲�
    float Tmp = P->sCurLoop.IqFdb + pSpd->DisturFilterCoff*Jtmp*pSpd->SpdFdb;

    UTILS_LP_FAST(pSpd->FilteredDisturComp, Tmp, pSpd->DisturFilterCoff);

    Tmp = pSpd->FilteredDisturComp - pSpd->DisturFilterCoff*Jtmp*pSpd->SpdFdb;

    pSpd->DisturbanceComp = P->sEncoder.MotorDirection * pSpd->DisturGain * Tmp/P->sCurLoop.M_PeakI;

    //pSpd->Kpdff = 0.9;//pSpd->TorFFGain / 1000.0f;  // PDFFϵ��������ת��ǰ��ϵ��
    //if(S_curve || Quintics)
    {
        SpeedPICal(pSpd); // �ٶ�ƽ��
    }
    /*	else
    	{
    		SpeedPICal(pSpd);
    	}*/
    if(POWER_OFF == P->PowerFlag)
    {
        pSpd->SpdKiPart = 0.0f;
        pSpd->Output = 0.0f;
    }

}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PolynomialCoeffCalculation(struct CurveParaStruct *P)
{
    P->CoeffA1 = 0;
    P->CoeffA2 = P->Acc / 2.0f;
    P->CoeffA3 = 20.0f*SpeedPeriod - 8.0f*P->SpdRefInc - 3.0f*P->Acc;
    P->CoeffA3 = P->CoeffA3 / 2.0f;
    P->CoeffA4 = 14.0f*P->SpdRefInc - 30.0f*SpeedPeriod + 3.0f*P->Acc;
    P->CoeffA4 = P->CoeffA4 / 2.0f;
    P->CoeffA5 = 12.0f*SpeedPeriod - 6.0f*P->SpdRefInc - P->Acc;
    P->CoeffA5 = P->CoeffA5 / 2;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SpeedRefSelect(struct AxisCtrlStruct *P)
{
    REAL32 Acc = 0.0f;
    REAL32 tmp = 0.0f;
    REAL32 tmp1 = 0.0f;

    struct SpeedLoopStruct *pSpd = &P->sSpdLoop;
    struct CurveParaStruct *pCurve = &P->CurvePara;

    switch(P->CtrlMode)
    {
    case SPD_CTRL :
    {
        Acc = pSpd->SpdRef - pSpd->SpdRefOld;

        if(Acc > (pSpd->AccMax/SpeedPeriod))
        {
            pSpd->SpdRefNoFilter = pSpd->SpdRefOld + (pSpd->AccMax/SpeedPeriod);
        }
        else if(Acc < - (pSpd->DecMax/SpeedPeriod))
        {
            pSpd->SpdRefNoFilter = pSpd->SpdRefOld - (pSpd->DecMax/SpeedPeriod);
        }
        else
        {
            pSpd->SpdRefNoFilter = pSpd->SpdRef;
        }

        tmp = (pSpd->SpdRefNoFilter - pSpd->SpdRefOld )*pSpd->TorFFGain;
        pSpd->TorFF = FilterIIR1LPFExec(&pSpd->sTorFFFilter, tmp);

        pSpd->SpdRefOld = pSpd->SpdRefNoFilter;

        //Speed reference filter
        if(P->sFilterCfg.bit.SpdRefFilter)
            pSpd->SpdRefActul = FilterIIR1LPFExec(&pSpd->sSpdRefFilter, pSpd->SpdRefNoFilter );
        if( fabs(pSpd->SpdRefActul- pSpd->SpdRefNoFilter)<1.0)
        {
            pSpd->SpdRefActul = pSpd->SpdRefNoFilter;
        }
    }
    if(!P->PowerFlag)
    {
        pSpd->SpdRefNoFilter = pSpd->SpdRefOld = pSpd->SpdFdb;
        pSpd->SpdRefActul = pSpd->SpdFdb;
        pCurve->CoeffA2 = 0.0f;
        pCurve->CoeffA3 = 0.0f;
        pCurve->CoeffA4 = 0.0f;
        pCurve->CoeffA5 = 0.0f;
    }

    break;

    case POS_CTRL :
        pSpd->SpdRef = P->sPosLoop.Output;
        pSpd->TorFF = 0.0f;
        break;

    case INNER_SPD_CTRL:

        pSpd->SpdRef = P->sInnerCtrl.Output;

        tmp = (pSpd->SpdRef - pSpd->SpdRefOld )*pSpd->TorFFGain;
        pSpd->TorFF = FilterIIR1LPFExec(&pSpd->sTorFFFilter, tmp);

        pSpd->SpdRefOld = pSpd->SpdRef;

        if(!P->PowerFlag)
            pSpd->SpdRefOld = pSpd->SpdFdb;
        break;

    default:
        pSpd->SpdKiPart = 0;
        break;
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SpeedPICal(struct SpeedLoopStruct *P)
{

    P->SpdErr = P->SpdRefActul - P->SpdFdb;;

    // PDFF control
    P->SpdKpPart = P->Vp* (P->Kpdff* P->SpdRefActul - P->SpdFdb);
    P->SpdKiPart = P->SpdKiPart + P->Vi * P->SpdErr;

    if(P->SpdKiPart > P->OutMax)
    {
        P->SpdKiPart = P->OutMax;
    }

    if(P->SpdKiPart < P->OutMin)
    {
        P->SpdKiPart = P->OutMin;
    }

    P->OutPreSat = P->SpdKpPart + P->SpdKiPart + P->TorFF + P->DisturbanceComp;

    if(P->SpdKpPart > P->OutMax)
    {
        P->Output = P->OutMax;
        P->SpdKiPart = 0.0f;
    }
    else if(P->SpdKpPart < P->OutMin)
    {
        P->Output = P->OutMin;
        P->SpdKiPart = 0.0f;
    }
    else if(P->OutPreSat > P->OutMax)
    {
        P->Output = P->OutMax;
//		P->SpdKiPart = P->OutMax - P->SpdKpPart;
        P->SpdKiPart = P->SpdKiPart - P->Vi * P->SpdErr;
    }
    else if(P->OutPreSat < P->OutMin)
    {
        P->Output = P->OutMin;
//		P->SpdKiPart = P->OutMin - P->SpdKpPart;
        P->SpdKiPart = P->SpdKiPart - P->Vi * P->SpdErr;
    }
    else
    {
        P->Output = P->OutPreSat;
    }


}


