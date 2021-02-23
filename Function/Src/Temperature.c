/*******************************************************************
 *
 * FILE NAME:  Temperature.c
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2019.02.15
 *
 * AUTHOR:      Denny
 *
 * History:
------------------------------------------------------------------------
15-02-2019 Version 1.00 : Created by Denny
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/
#include "Temperature.h"
#include "HardApi.h"
#include "adc.h"
#include "Param.h"

PRIVATE INT16 ReadMosTemperature(UINT16 AdcValue);
PRIVATE INT16 ReadMotorTemperature(UINT16 AdcValue);

const UINT16 MosTempAdcTable[30]=
{
    3992,  //-20
    3962,  //-15
    3925,  //-10
    3881,  //-5
    3828,  //0
    3766,  //5
    3694,  //10
    3611,  //15
    3517,  //20
    3413,  //25
    3297,  //30
    3173,  //35
    3039,  //40
    2899,  //45
    2753,  //50
    2603,  //55
    2451,  //60
    2299,  //65
    2148,  //70
    2001,  //75
    1859,  //80
    1722,  //85
    1592,  //90
    1469,  //95
    1353,  //100
    1245,  //105
    1145,  //110
    1052,  //115
    967,   //120
    888,   //125
};

const UINT16 MotorTempAdcTable[181]=
{
    4082,   //-55
    4082,   //-54
    4081,   //-53
    4081,   //-52
    4080,   //-51
    4079,   //-50
    4079,   //-49
    4078,   //-48
    4077,   //-47
    4076,   //-46
    4075,   //-45
    4074,   //-44
    4072,   //-43
    4071,   //-42
    4069,   //-41
    4068,   //-40
    4066,   //-39
    4064,   //-38
    4063,   //-37
    4061,   //-36
    4058,   //-35
    4056,   //-34
    4054,   //-33
    4051,   //-32
    4048,   //-31
    4046,   //-30
    4043,   //-29
    4039,   //-28
    4036,   //-27
    4032,   //-26
    4029,   //-25
    4025,   //-24
    4021,   //-23
    4016,   //-22
    4012,   //-21
    4007,   //-20
    4002,   //-19
    3997,   //-18
    3991,   //-17
    3986,   //-16
    3979,   //-15
    3973,   //-14
    3967,   //-13
    3960,   //-12
    3953,   //-11
    3945,   //-10
    3937,   //-9
    3929,   //-8
    3921,   //-7
    3912,   //-6
    3903,   //-5
    3893,   //-4
    3884,   //-3
    3873,   //-2
    3863,   //-1
		3852,   //0
    3840,   //1
    3828,   //2
    3816,   //3
    3803,   //4
    3790,   //5
    3776,   //6
    3762,   //7
    3747,   //8
    3732,   //9
    3716,   //10
    3700,   //11
    3683,   //12
    3666,   //13
    3648,   //14
    3629,   //15
    3610,   //16
    3591,   //17
    3570,   //18
    3550,   //19
    3528,   //20
    3507,   //21
    3484,   //22
    3461,   //23
    3437,   //24
    3413,   //25
    3388,   //26
    3362,   //27
    3336,   //28
    3310,   //29
    3282,   //30
    3254,   //31
    3226,   //32
    3197,   //33
    3167,   //34
    3137,   //35
    3106,   //36
    3075,   //37
    3043,   //38
    3011,   //39
    2978,   //40
    2944,   //41
    2911,   //42
    2876,   //43
    2842,   //44
    2807,   //45
    2771,   //46
    2736,   //47
    2700,   //48
    2663,   //49
    2630,   //50
    2589,   //51
    2552,   //52
    2515,   //53
    2477,   //54
    2440,   //55
    2402,   //56
    2364,   //57
    2326,   //58
    2288,   //59
    2250,   //60
    2212,   //61
    2174,   //62
    2136,   //63
    2098,   //64
    2061,   //65
    2023,   //66
    1986,   //67
    1949,   //68
    1912,   //69
    1875,   //70
    1839,   //71
    1803,   //72
    1767,   //73
    1732,   //74
    1697,   //75
    1662,   //76
    1628,   //77
    1594,   //78
    1560,   //79
    1527,   //80
    1494,   //81
    1462,   //82
    1430,   //83
    1399,   //84
    1368,   //85
    1338,   //86
    1308,   //87
    1279,   //88
    1250,   //89
    1222,   //90
    1194,   //91
    1167,   //92
    1140,   //93
    1114,   //94
    1088,   //95
    1062,   //96
    1038,   //97
    1013,   //98
    990,   //99
    966,   //100
    944,   //101
    921,   //102
    899,   //103
    878,   //104
    857,   //105
    837,   //106
    817,   //107
    797,   //108
    778,   //109
    759,   //110
    741,   //111
    723,   //112
    706,   //113
    689,   //114
    673,   //115
    657,   //116
    641,   //117
    625,   //118
    610,   //119
    596,   //120
    582,   //121
    568,   //122
    554,   //123
    541,   //124
    528,   //125
};

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void TemperatureInit(void)
{

}


/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void TemperatureExec(void)
{
    UINT16 LeftMosAdc = 0;
    UINT16 RightMosAdc = 0;
    UINT16 leftMotorAdc = 0;
    UINT16 rightMotorAdc = 0;
   

    GetMosAdc(&LeftMosAdc, &RightMosAdc);
//    gParam[0].MosTemp0x230B = ReadMosTemperature(LeftMosAdc);
//    gParam[1].MosTemp0x230B = ReadMosTemperature(RightMosAdc);
    gParam[0].MosTemp0x230B = 25;
    gParam[1].MosTemp0x230B = 25;
    
    GetMotorAdc(&leftMotorAdc, &rightMotorAdc);
    gParam[0].MotorTemp0x230C = ReadMotorTemperature(leftMotorAdc);
    gParam[1].MotorTemp0x230C = ReadMotorTemperature(rightMotorAdc);



}

/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE INT16 ReadMosTemperature(UINT16 AdcValue)
{
    INT16  TempMin = 0;
    INT16  TempMax = 29;
    INT16  MidVal = 0;
    INT16  Temperature = 0;
    
    if (AdcValue < MosTempAdcTable[TempMax])
    {
        TempMin = TempMax;
    }
    else
    {
        while(AdcValue >= MosTempAdcTable[TempMax] )
        {
            MidVal = (TempMax + TempMin) / 2;

            if((MidVal == TempMin) || (MidVal == TempMax))
            {
                Temperature = (TempMin-4)*5;
                return Temperature;
            }

            if(AdcValue >= MosTempAdcTable[MidVal])
            {
                    TempMax = MidVal;
            }
            else
            {
                    TempMin = MidVal;
            }
        }    
    }

    Temperature = (TempMin-4)*5;
    return Temperature;
}

/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE INT16 ReadMotorTemperature(UINT16 AdcValue)
{
    INT16  TempMin = 0;
    INT16  TempMax = 180;
    INT16  MidVal = 0;
    INT16  Temperature = 0;
    
    if (AdcValue < MotorTempAdcTable[TempMax])
    {
        TempMin = TempMax;
    }
    else
    {
        while(AdcValue >= MotorTempAdcTable[TempMax] )
        {
            MidVal = (TempMax + TempMin) / 2;

            if((MidVal == TempMin) || (MidVal == TempMax))
            {
                Temperature = TempMin-55;
                return Temperature;
            }

            if(AdcValue >= MotorTempAdcTable[MidVal])
            {
                TempMax = MidVal;
            }
            else
            {
                TempMin = MidVal;
            }
        }    
    }

    Temperature = TempMin-55;
    return Temperature;
}
