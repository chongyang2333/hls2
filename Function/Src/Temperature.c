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

const UINT16 MotorTempAdcTable[30]=
{
    4019,  //-20
    3992,  //-15
    3959,  //-10
    3917,  //-5
    3865,  //0
    3802,  //5
    3726,  //10
    3636,  //15
    3532,  //20
    3413,  //25
    3279,  //30
    3132,  //35
    2973,  //40
    2805,  //45
    2629,  //50
    2450,  //55
    2269,  //60
    2091,  //65
    1917,  //70
    1749,  //75
    1591,  //80
    1442,  //85
    1303,  //90
    1176,  //95
    1059,  //100
    953,   //105
    857,   //110
    770,   //115
    693,   //120
    623,   //125
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
    gParam[0].MosTemp0x230B = ReadMosTemperature(LeftMosAdc);
    gParam[1].MosTemp0x230B = ReadMosTemperature(RightMosAdc);

//    GetMotorAdc(&leftMotorAdc, &rightMotorAdc);
//    gParam[0].MotorTemp0x230C = ReadMotorTemperature(leftMotorAdc);
//    gParam[1].MotorTemp0x230C = ReadMotorTemperature(rightMotorAdc);
    gParam[0].MotorTemp0x230C = 0;
    gParam[1].MotorTemp0x230C = 0;
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
    INT16  TempMax = 29;
    INT16  MidVal = 0;
    INT16  Temperature = 0;

    while(AdcValue >= MotorTempAdcTable[TempMax] )
    {
        MidVal = (TempMax + TempMin) / 2;

        if((MidVal == TempMin) || (MidVal == TempMax))
        {
            Temperature = (TempMin-4)*5;
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

    Temperature = (TempMin-4)*5;
    return Temperature;
}
