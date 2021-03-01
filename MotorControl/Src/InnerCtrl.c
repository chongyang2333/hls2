/*******************************************************************
 *
 * FILE NAME:  InnerCtrl.c
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2019.02.16
 *
 * AUTHOR:      Denny
 *
 * History:
------------------------------------------------------------------------
16-02-2019 Version 1.00 : Created by Denny
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/
#include "InnerCtrl.h"
#include "ControlRun.h"
#include "Param.h"

#define TYPE_SQUARE_WAVE  1
#define TYPE_SINE_WAVE    2
/***********************************************************************
 * DESCRIPTION: pwm frequecy, 10khz
 *
 * RETURNS:
 *
***********************************************************************/
extern const REAL32 SinCTable[];
PUBLIC void InnerCtrlExec(struct AxisCtrlStruct *P)
{
    struct InnerCtrlStruct *pCtrl = &P->sInnerCtrl;
    
    if(pCtrl->En)
    {
        pCtrl->Tick++;
        UINT32 tmp = pCtrl->Tick%pCtrl->Period;
        
        if(pCtrl->Type == TYPE_SQUARE_WAVE)
        {    
            if(tmp < (pCtrl->Period>>1))
            {
                pCtrl->Output = pCtrl->Offset + pCtrl->Amp;
            }
            else
            {
                pCtrl->Output = pCtrl->Offset - pCtrl->Amp;
            }
            
        }
        else if(pCtrl->Type == TYPE_SINE_WAVE)
        {
            REAL32 SinTheta = 0.0f;
            UINT32 angle = tmp*C_SinUint/pCtrl->Period;
            
            if(angle > C_SinUint3D4)
            {
                SinTheta = -SinCTable[C_SinUint - angle];
            }
            else if(angle > C_SinUint1D2)
            {
                SinTheta = -SinCTable[angle - C_SinUint1D2];
            }
            else if(angle > C_SinUint1D4)
            {
                SinTheta = SinCTable[C_SinUint1D2 - angle];
            }
            else
            {
                SinTheta = SinCTable[angle];
            }
            
            pCtrl->Output = pCtrl->Offset + pCtrl->Amp*SinTheta;
            
        }
        else
        {
            pCtrl->Output = 0.0f;
        }
        
        if(pCtrl->Tick >= (pCtrl->Cycle*pCtrl->Period))
        {
            pCtrl->Output = 0.0f;
            pCtrl->En = 0;
            pCtrl->Tick = 0; 
            gParam[0].InnerCtrlEnable0x3004 = 0;
        }
        
    }
    else
    {
        pCtrl->Tick = 0; 
        pCtrl->Output = 0.0f;
    }
    
}

