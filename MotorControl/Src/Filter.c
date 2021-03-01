/*******************************************************************
 *
 * FILE NAME:  Filter.c
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
#include "string.h"
#include "math.h"
#include "ControlRun.h"

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC REAL32 FilterIIR1LPFExec(struct IIR1LPFStruct *P, REAL32 In)
{
	P->In = In;
	P->Yn = P->B0*(P->In + P->In1)+P->A1*P->Yn;
	P->In1 = P->In;
	return P->Yn;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void IIR1LPFParameterCal(struct IIR1LPFStruct *P ,INT16 Fc, REAL32 Ts)
{
	REAL32 Tmp;

	Tmp = 2.0f * PI * Fc * Ts;

	P->B0 = Tmp/(2.0f + Tmp);

	if(P->B0 >= 0.5f)
	{
		P->B0 = 0.5f;
	}

	P->B1 = P->B0;
	P->A1 = 1.0f - P->B0 - P->B1;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void FilterIIR2New(struct IIR2Struct *P)
{
	(void)memset (P, 0, sizeof (struct IIR2Struct));
}

/***********************************************************************
 * DESCRIPTION:
//			Y(Z)	  B0*Z^2 + B1*Z + B2
//	H(z) = ------ = ------------------------
//			X(Z)	  A0*Z^2 + A1*Z + A2
 * A0=1
 * RETURNS:
 *
***********************************************************************/
PUBLIC REAL32 FilterIIR2Exec(struct IIR2Struct *P, REAL32 In)
{
	P->Xn = In;
	P->Yn = P->B0 * P->Xn + P->B1 * P->Xn1 + P->B2 * P->Xn2
			               -P->A1 * P->Yn1 - P->A2 * P->Yn2;
	P->Yn2 = P->Yn1;
	P->Yn1 = P->Yn;
	P->Xn2 = P->Xn1;
	P->Xn1 = P->Xn;

	return(P->Yn);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void FilterIIR2ResetState(struct IIR2Struct *P)
{
	P->Xn = 0;
	P->Xn1 = 0;
	P->Xn2 = 0;

	P->Yn = 0;
	P->Yn1 = 0;
	P->Yn2 = 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void FilterFIR8New(struct FIR8Struct *P)
{
	(void)memset (P, 0, sizeof(struct FIR8Struct));
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC REAL32 FilterFIR8Exec(struct FIR8Struct *P, REAL32 In)
{
	INT16 k;
	REAL32 y = 0.0f;

	P->State[0] = In;

	for (k = 7; k >= 0; k--)
	{
		y = P->State[k] * P->Coeff[k] + y;
		P->State[k + 1] = P->State[k];
	}

	return y;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void FilterFIR8SetStates(struct FIR8Struct *P, REAL32 States)
{
	INT16 k;

	for (k = 7; k >= 0; k--)
		P->State[k] = States;
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *   均值滤波器
***********************************************************************/
/*PUBLIC REAL32 AveFilter(struct AverageFilter *p , REAL32 In , INT16 FilterT)//平均值滤波器
{
    REAL32 ave = 0;
    INT16 N = 0;
	p->n = FilterT;
	p->input = In;
    if(p->n < 2)
    {
		ave = p->input;
		p->out = ave;
		return p->out;
	}
	N = (p->n > 2000) ? 2000 : p->n;//MIN(p->n,1024); // N = n  //1024够了吗？

	//p->sum = p->sum  + p->input; //递推公式
	p->sum = p->sum  + (p->input - p->a[p->id]); //递推公式
	p->a[p->id] = p->input;
	
	ave = (p->sum + p->rem) / N;
	p->rem = 0; //  p->sum - ave * n;  无法保证增量位置总量不变 但可对绝对位置进行滤波  于 LPF 类似效果
	p->id = p->id + 1;
	if (p->id >= N) // 0~n-1 n 个历史数据
		p->id = 0;
	p->out = ave;
	return p->out;
}
*/
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
/*PUBLIC void AveFilterResetState(struct AverageFilter *P)
{
	P->a[0] = 0;
	P->n = 0;
	P->id = 0;
	P->rem = 0;

	P->input = 0;
	P->sum = 0;
	P->out = 0;
}*/

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void IIR1HPFParameterCal(struct IIR2Struct *P ,INT16 Fc, REAL32 Ts)
{

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void IIR2LPFParameterCal(struct IIR2Struct *P ,INT16 Fc, REAL32 Ts)
{

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void IIR2HPFParameterCal(struct IIR2Struct *P ,INT16 Fc, REAL32 Ts)
{

}



/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void IIR2BPFParameterCal(struct IIR2Struct *P ,INT16 Fc, REAL32 Ts)
{

}

