/*******************************************************************
 *
 * FILE NAME:  Filter.h
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
#ifndef _FILTER_H_
#define _FILTER_H_

#include "UserDataTypes.h"

struct IIR1LPFStruct
{
	REAL32	A1;			//系数A1
	REAL32	B0;
	REAL32	B1;

	REAL32  Yn;
	REAL32	In;
	REAL32	In1;
};

struct IIR2Struct
{
	REAL32	Yn;
	REAL32	Yn1;
	REAL32	Yn2;

	REAL32	Xn;
	REAL32	Xn1;
	REAL32	Xn2;

	REAL32	A1;
	REAL32	A2;
	REAL32  B0;
	REAL32	B1;
	REAL32	B2;

};

struct FIR8Struct
{
	REAL32  State[9];
	REAL32  Coeff[8];
};

//struct FIRStruct
//{
//	REAL32	xx[6];			//系数A1
//	REAL32	ret;
//};

/*struct AverageFilter
{
	REAL32 a[2000];
    INT16 n;   	// 滤波时间 输入 1
    INT16 id;
    REAL32  rem;
    REAL32  input; 	// 输入 2
    REAL32  sum; 	// 记忆
    REAL32  out;
} ;//均值滤波器
*/
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * (value - (sample)))

extern PUBLIC void FilterIIR2New(struct IIR2Struct *P);
extern PUBLIC REAL32 FilterIIR2Exec(struct IIR2Struct *P, REAL32 In);
extern PUBLIC void FilterIIR2ResetState(struct IIR2Struct *P);
extern PUBLIC void FilterFIR8New(struct FIR8Struct *P);
extern PUBLIC REAL32 FilterFIR8Exec(struct FIR8Struct *P, REAL32 In);
extern PUBLIC void FilterFIR8SetStates(struct FIR8Struct *P, REAL32 States);
extern PUBLIC REAL32 FilterIIR1LPFExec(struct IIR1LPFStruct *P, REAL32 In);

//extern PUBLIC REAL32 AveFilter(struct AverageFilter *p , REAL32 In , INT16 FilterT);//均值滤波
//extern PUBLIC void AveFilterResetState(struct AverageFilter *P);


extern void IIR1LPFParameterCal(struct IIR1LPFStruct *P ,INT16 Fc, REAL32 Ts);
extern void IIR2NotchParameterCal(struct IIR2Struct *P ,INT16 Fc, REAL32 Ts, REAL32 u);

//extern PUBLIC void AveFilterResetState(struct AverageFilter *P);


#endif  // _FILTER_H_
