/**********************************************************************
 *
 * FILE NAME:  ControlRun.c
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
#include "CurrentSample.h"

extern INT16 ADC2_JDR0_Offset;
extern INT16 ADC2_JDR1_Offset;
extern INT16 ADC2_JDR2_Offset;
extern INT16 ADC1_JDR0_Offset;
extern INT16 ADC1_JDR1_Offset;
extern INT16 ADC1_JDR2_Offset;


PRIVATE void ValueLimit(INT16 ValueTemp,INT16 MinLimit,INT16 MaxLimit);
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CurrentSampleTimeCal(struct CurrentLoopStruct *P,UINT16 AxisID)
{
    UINT16 SampleTime = 0;
    UINT16 MaxDutyTemp = 0;
    UINT16 MidDutyTemp = 0;
    UINT16 CurEffective = 1;
    UINT16 TrigoDir = 0;

    switch(P->Sector)
    {
        case 1:
            MaxDutyTemp = P->TaNumber;
            MidDutyTemp = P->TbNumber;
            if(AxisID == AXIS_RIGHT)
            {
                adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_2,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
                adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_13,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE
            }
            else if(AxisID == AXIS_LEFT)
            {
                adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_11,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
                adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_5,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
            }
            break;
        case 2:
            MaxDutyTemp = P->TbNumber;
            MidDutyTemp = P->TaNumber;
            if(AxisID == AXIS_RIGHT)
            {
                adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_1,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
                adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_13,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE
            }
            else if(AxisID == AXIS_LEFT)
            {
                adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_10,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
                adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_5,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
            }
            break;
        case 3:
            MaxDutyTemp = P->TbNumber;
            MidDutyTemp = P->TcNumber;
            if(AxisID == AXIS_RIGHT)
            {
                adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_1,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
                adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_13,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE
            }
            else if(AxisID == AXIS_LEFT)
            {
                adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_10,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
                adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_5,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
            }
            break;
        case 4:
            MaxDutyTemp = P->TcNumber;
            MidDutyTemp = P->TbNumber;
            if(AxisID == AXIS_RIGHT)
            {
                adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_1,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
                adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_2,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE
            }
            else if(AxisID == AXIS_LEFT)
            {
                adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_10,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
                adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_11,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
            }
            break;
        case 5:
            MaxDutyTemp = P->TcNumber;
            MidDutyTemp = P->TaNumber;
            if(AxisID == AXIS_RIGHT)
            {
                adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_1,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
                adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_2,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE
            }
            else if(AxisID == AXIS_LEFT)
            {
                adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_10,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
                adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_11,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
            }
            break;
        case 6:
            MaxDutyTemp = P->TaNumber;
            MidDutyTemp = P->TcNumber;
            if(AxisID == AXIS_RIGHT)
            {
                adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_2,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
                adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_13,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE
            }
            else if(AxisID == AXIS_LEFT)
            {
                adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_11,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
                adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_5,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
            }
            break;
        default :
					  if(AxisID == AXIS_RIGHT)
            {
                adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_1,ADC_SAMPLETIME_3);    //R_V_I_SAMPLE
                adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_2,ADC_SAMPLETIME_3);   //R_W_I_SAMPLE
            }
            else if(AxisID == AXIS_LEFT)
            {
                adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_10,ADC_SAMPLETIME_3);   //L_V_I_SAMPLE
                adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_11,ADC_SAMPLETIME_3);    //L_W_I_SAMPLE
            }
            break;
    }

    MaxDutyTemp = MaxDutyTemp > PWM_PERIOD_VALUE ? PWM_PERIOD_VALUE : MaxDutyTemp;
    MidDutyTemp = MidDutyTemp > PWM_PERIOD_VALUE ? PWM_PERIOD_VALUE : MidDutyTemp;

    if((MaxDutyTemp + (DEADTIME + NOISETIME)) <= PWM_PERIOD_VALUE)
    {
        SampleTime = PWM_PERIOD_VALUE - 1;
    }
    else  //最大相PWM接近100%占空比
    {
        SampleTime = MaxDutyTemp - MidDutyTemp;
        if(SampleTime > (DEADTIME + NOISETIME + SAMPLINGTIME))     //最大与次大相占空比差大于(DEADTIME_NS+MAX_TNTR_NS+SAMPLING_TIME_NS)
        {
            SampleTime = MaxDutyTemp - SAMPLINGTIME;               //Ts before MaxDutyTemp
        }
        else //最大相与次大相PWM接近且接近100%占空比
        {
            if((2 * PWM_PERIOD_VALUE - MaxDutyTemp - (DEADTIME + NOISETIME) - MidDutyTemp) >= SAMPLINGTIME)
            {
                SampleTime = (2 * PWM_PERIOD_VALUE) - MaxDutyTemp - (DEADTIME + NOISETIME);
                TrigoDir = 1;
            }
            else
            {
                SampleTime = MaxDutyTemp - SAMPLINGTIME;
                CurEffective = 0;
            }
        }
    }

    P->TdNumber = SampleTime;
    P->CurEffective = CurEffective;

		
		if(AxisID == AXIS_RIGHT)
		{
				if(TrigoDir == 0)
				{
						timer_channel_output_mode_config(TIMER7,TIMER_CH_3,TIMER_OC_MODE_PWM1);
				}
				else
				{
						timer_channel_output_mode_config(TIMER7,TIMER_CH_3,TIMER_OC_MODE_PWM0);
				}
		}
		else if(AxisID == AXIS_LEFT)
		{
				if(TrigoDir == 0)
				{
						timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM1);
				}
				else
				{
						timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM0);
				}
		}

}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void GetPhaseCurrentRe(struct AxisCtrlStruct *X)
{
    INT16 CurASample = 0;
    INT16 CurBSample = 0;
    INT16 CurCSample = 0;
    INT16 CurErrDataA = 0;
    INT16 CurErrDataB = 0;
    INT16 CurErrDataC = 0;
    INT16 CurErrDataTemp = 0;
    struct CurrentLoopStruct *P = &X->sCurLoop;
    //A相电流处理
    CurErrDataTemp = P->CurPhaA_Data[0] - P->CurPhaA_Data[1];
    CurErrDataA = abs(CurErrDataTemp);
    CurErrDataTemp = P->CurPhaA_Data[1] - P->CurPhaA_Data[2];
    CurErrDataA += abs(CurErrDataTemp);
    ValueLimit(CurErrDataA,16,2048);
    //B相电流处理
    CurErrDataTemp = P->CurPhaB_Data[0] - P->CurPhaB_Data[1];
    CurErrDataB = abs(CurErrDataTemp);
    CurErrDataTemp = P->CurPhaB_Data[1] - P->CurPhaB_Data[2];
    CurErrDataB += abs(CurErrDataTemp);
    ValueLimit(CurErrDataB,16,2048);
    //C相电流处理
    CurErrDataTemp = P->CurPhaC_Data[0] - P->CurPhaC_Data[1];
    CurErrDataC = abs(CurErrDataTemp);
    CurErrDataTemp = P->CurPhaC_Data[1] - P->CurPhaC_Data[2];
    CurErrDataC += abs(CurErrDataTemp);
    ValueLimit(CurErrDataC,16,2048);
	


    switch(P->Sector)
    {
        case 1:
            if(X->AxisID == AXIS_RIGHT)
            {
                CurCSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1) - ADC2_JDR2_Offset;
            }
            else if(X->AxisID == AXIS_LEFT)
            {
                CurCSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - ADC1_JDR2_Offset;
            }

            if(CurCSample - P->CurPhaC_Data[0] > CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
            }
            else if(CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
            }
            ValueLimit(CurCSample,-2048,2048);

            if(P->CurEffective != 0)
            {
                if(X->AxisID == AXIS_RIGHT)
                {
                    CurBSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0) - ADC2_JDR1_Offset;
                }
                else if(X->AxisID == AXIS_LEFT)
                {
                    CurBSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - ADC1_JDR1_Offset;
                }

                if(CurBSample - P->CurPhaB_Data[0] > CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
                }
                else if(CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
                }
                ValueLimit(CurBSample,-2048,2048);
            }
            else
            {
                CurErrDataTemp = CurCSample - P->CurPhaC_Data[0];
                CurErrDataTemp = abs(CurErrDataTemp);

                if((P->CurPhaB_Data[0] - P->CurPhaB_Data[1] >= 0) && (P->CurPhaB_Data[1] - P->CurPhaB_Data[2] >= 0))
                {
                    CurBSample = P->CurPhaB_Data[0] + CurErrDataTemp;
                }
                else if((P->CurPhaB_Data[0] - P->CurPhaB_Data[1] < 0) && (P->CurPhaB_Data[1] - P->CurPhaB_Data[2] < 0))
                {
                    CurBSample = P->CurPhaB_Data[0] - CurErrDataTemp;
                }
                else if((P->CurPhaB_Data[0] - P->CurPhaB_Data[1] >= 0)&& (P->CurPhaB_Data[1] - P->CurPhaB_Data[2] < 0))
                {
                    CurBSample = (3 * P->CurPhaB_Data[0] + CurErrDataTemp  - P->CurPhaB_Data[1])>>1;
                }
                else
                {
                    CurBSample = (3 * P->CurPhaB_Data[0] - CurErrDataTemp  - P->CurPhaB_Data[1])>>1;
                }

                if(CurBSample - P->CurPhaB_Data[0] >  CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
                }
                else if (CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
                }
                ValueLimit(CurBSample,-2048,2048);
            }
            CurASample = -CurBSample - CurCSample;
            if(CurASample - P->CurPhaA_Data[0] > CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] + CurErrDataA;
            }
            else if(CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] - CurErrDataA;
            }
            ValueLimit(CurASample,-2048,2048);
            break;

        case 2:
            if(X->AxisID == AXIS_RIGHT)
            {
                CurCSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1) - ADC2_JDR2_Offset;
            }
            else if(X->AxisID == AXIS_LEFT)
            {
                CurCSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - ADC1_JDR2_Offset;
            }

            if(CurCSample - P->CurPhaC_Data[0] > CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
            }
            else if(CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
            }
            ValueLimit(CurCSample,-2048,2048);

            if(P->CurEffective != 0)
            {
                if(X->AxisID == AXIS_RIGHT)
                {
                    CurASample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0) - ADC2_JDR0_Offset;
                }
                else if(X->AxisID == AXIS_LEFT)
                {
                    CurASample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - ADC1_JDR0_Offset;
                }

                if(CurASample - P->CurPhaA_Data[0] >  CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] + CurErrDataA;
                }
                else if(CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] - CurErrDataA;
                }
                ValueLimit(CurASample,-2048,2048);
            }
            else
            {
                CurErrDataTemp = CurCSample - P->CurPhaC_Data[0];
                CurErrDataTemp = abs(CurErrDataTemp);
                if((P->CurPhaA_Data[0] - P->CurPhaA_Data[1] >= 0) && (P->CurPhaA_Data[1] - P->CurPhaA_Data[2] >= 0))
                {
                    CurASample = P->CurPhaA_Data[0] + CurErrDataTemp;
                }
                else if((P->CurPhaA_Data[0] - P->CurPhaA_Data[1] < 0) && (P->CurPhaA_Data[1] - P->CurPhaA_Data[2] < 0))
                {
                    CurASample = P->CurPhaA_Data[0] - CurErrDataTemp;
                }
                else if((P->CurPhaA_Data[0] - P->CurPhaA_Data[1] >= 0) && (P->CurPhaA_Data[1] - P->CurPhaA_Data[2] < 0))
                {
                    CurASample = (3 * P->CurPhaA_Data[0] + CurErrDataTemp  - P->CurPhaA_Data[1]) >> 1;
                }
                else
                {
                    CurASample = (3 * P->CurPhaA_Data[0] - CurErrDataTemp  - P->CurPhaA_Data[1]) >> 1;
                }

                if(CurASample - P->CurPhaA_Data[0] >  CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] + CurErrDataA;
                }
                else if(CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] - CurErrDataA;
                }
                ValueLimit(CurASample,-2048,2048);
            }
            // Ib = -Ic-Ia;
            CurBSample = -CurASample - CurCSample;
            if(CurBSample - P->CurPhaB_Data[0] >  CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
            }
            else if(CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
            }
            ValueLimit(CurBSample,-2048,2048);
            break;

        case 3:
            if(X->AxisID == AXIS_RIGHT)
            {
                CurASample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0) - ADC2_JDR0_Offset;
            }
            else if(X->AxisID == AXIS_LEFT)
            {
                CurASample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - ADC1_JDR0_Offset;
            }

            if(CurASample - P->CurPhaA_Data[0] >  CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] + CurErrDataA;
            }
            else if(CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] - CurErrDataA;
            }
            ValueLimit(CurASample,-2048,2048);

            if(P->CurEffective != 0)
            {
                if(X->AxisID == AXIS_RIGHT)
                {
                    CurCSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1) - ADC2_JDR2_Offset;
                }
                else if(X->AxisID == AXIS_LEFT)
                {
                    CurCSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - ADC1_JDR2_Offset;
                }

                if(CurCSample - P->CurPhaC_Data[0] >  CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
                }
                else if(CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
                }
                ValueLimit(CurCSample,-2048,2048);
            }
            else
            {
                CurErrDataTemp = CurASample - P->CurPhaA_Data[0];
                CurErrDataTemp = abs(CurErrDataTemp);
                if((P->CurPhaC_Data[0] - P->CurPhaC_Data[1] >= 0) && (P->CurPhaC_Data[1] - P->CurPhaC_Data[2] >= 0))
                {
                    CurCSample = P->CurPhaC_Data[0] + CurErrDataTemp;
                }
                else if((P->CurPhaC_Data[0] - P->CurPhaC_Data[1] < 0) && (P->CurPhaC_Data[1] - P->CurPhaC_Data[2] < 0))
                {
                    CurCSample = P->CurPhaC_Data[0] - CurErrDataTemp;
                }
                else if((P->CurPhaC_Data[0] - P->CurPhaC_Data[1] >= 0) && (P->CurPhaC_Data[1] - P->CurPhaC_Data[2] < 0))
                {
                    CurCSample = (3 * P->CurPhaC_Data[0] + CurErrDataTemp  - P->CurPhaC_Data[1]) >> 1;
                }
                else
                {
                    CurCSample = (3 * P->CurPhaC_Data[0] - CurErrDataTemp  - P->CurPhaC_Data[1]) >> 1;
                }
                if(CurCSample - P->CurPhaC_Data[0] >  CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
                }
                else if(CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
                }
                ValueLimit(CurCSample,-2048,2048);
            }
            // Ib = -Ic-Ia;
            CurBSample = -CurASample-CurCSample;

            if(CurBSample - P->CurPhaB_Data[0] >  CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
            }
            else if(CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
            }
            ValueLimit(CurBSample,-2048,2048);
            break;

        case 4:
            if(X->AxisID == AXIS_RIGHT)
            {
                CurASample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0) - ADC2_JDR0_Offset;
            }
            else if(X->AxisID == AXIS_LEFT)
            {
                CurASample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - ADC1_JDR0_Offset;
            }

            if(CurASample - P->CurPhaA_Data[0] >  CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] + CurErrDataA;
            }
            else if(CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] - CurErrDataA;
            }
            ValueLimit(CurASample,-2048,2048);

            if(P->CurEffective != 0)
            {
                if(X->AxisID == AXIS_RIGHT)
                {
                    CurBSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1) - ADC2_JDR1_Offset;
                }
                else if(X->AxisID == AXIS_LEFT)
                {
                    CurBSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - ADC1_JDR1_Offset;
                }

                if(CurBSample - P->CurPhaB_Data[0] > CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
                }
                else if(CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
                }
                ValueLimit(CurBSample,-2048,2048);
            }
            else
            {
                CurErrDataTemp = CurASample - P->CurPhaA_Data[0];
                CurErrDataTemp = abs(CurErrDataTemp);
                if((P->CurPhaB_Data[0] - P->CurPhaB_Data[1] >= 0) && (P->CurPhaB_Data[1] - P->CurPhaB_Data[2] >= 0))
                {
                    CurBSample = P->CurPhaB_Data[0] + CurErrDataTemp;
                }
                else if((P->CurPhaB_Data[0] - P->CurPhaB_Data[1] < 0) && (P->CurPhaB_Data[1] - P->CurPhaB_Data[2] < 0))
                {
                    CurBSample = P->CurPhaB_Data[0] - CurErrDataTemp;
                }
                else if((P->CurPhaB_Data[0] - P->CurPhaB_Data[1] >= 0) && (P->CurPhaB_Data[1] - P->CurPhaB_Data[2] < 0))
                {
                    CurBSample = (3 * P->CurPhaB_Data[0] + CurErrDataTemp  - P->CurPhaB_Data[1]) >> 1;
                }
                else
                {
                    CurBSample = (3 * P->CurPhaB_Data[0] - CurErrDataTemp  - P->CurPhaB_Data[1]) >> 1;
                }

                if(CurBSample - P->CurPhaB_Data[0] >  CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
                }
                else if(CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
                {
                    CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
                }
                ValueLimit(CurBSample,-2048,2048);
            }
            // Ic = -Ia-Ib;
            CurCSample = -CurASample - CurBSample;
            if(CurCSample - P->CurPhaC_Data[0] > CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
            }
            else if(CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
            }
            ValueLimit(CurCSample,-2048,2048);
            break;

        case 5:
            if(X->AxisID == AXIS_RIGHT)
            {
                CurBSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1) - ADC2_JDR1_Offset;
            }
            else if(X->AxisID == AXIS_LEFT)
            {
                CurBSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - ADC1_JDR1_Offset;
            }

            if(CurBSample - P->CurPhaB_Data[0] > CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
            }
            else if(CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
            }
            ValueLimit(CurBSample,-2048,2048);

            if(P->CurEffective != 0)
            {
                if(X->AxisID == AXIS_RIGHT)
                {
                    CurASample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0) - ADC2_JDR0_Offset;
                }
                else if(X->AxisID == AXIS_LEFT)
                {
                    CurASample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - ADC1_JDR0_Offset;
                }
                if(CurASample - P->CurPhaA_Data[0] >  CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] + CurErrDataA;
                }
                else if (CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] - CurErrDataA;
                }
                ValueLimit(CurASample,-2048,2048);
            }
            else
            {
                CurErrDataTemp = CurBSample - P->CurPhaB_Data[0];
                CurErrDataTemp = abs(CurErrDataTemp);
                if((P->CurPhaA_Data[0] - P->CurPhaA_Data[1] >= 0) && (P->CurPhaA_Data[1] - P->CurPhaA_Data[2] >= 0))
                {
                    CurASample = P->CurPhaA_Data[0] + CurErrDataTemp;
                }
                else if((P->CurPhaA_Data[0] - P->CurPhaA_Data[1] < 0) && (P->CurPhaA_Data[1] - P->CurPhaA_Data[2] < 0))
                {
                    CurASample = P->CurPhaA_Data[0] - CurErrDataTemp;
                }
                else if((P->CurPhaA_Data[0] - P->CurPhaA_Data[1] >= 0) && (P->CurPhaA_Data[1] - P->CurPhaA_Data[2] < 0))
                {
                    CurASample = (3 * P->CurPhaA_Data[0] + CurErrDataTemp  - P->CurPhaA_Data[1]) >> 1;
                }
                else
                {
                    CurASample = (3 * P->CurPhaA_Data[0] - CurErrDataTemp  - P->CurPhaA_Data[1]) >> 1;
                }

                if(CurASample - P->CurPhaA_Data[0] >  CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] + CurErrDataA;
                }
                else if(CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
                {
                    CurASample = P->CurPhaA_Data[0] - CurErrDataA;
                }
                ValueLimit(CurASample,-2048,2048);
            }
            // Ic = -Ia-Ib;
            CurCSample = -CurASample - CurBSample;
            if(CurCSample - P->CurPhaC_Data[0] >  CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
            }
            else if(CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
            {
                CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
            }
            ValueLimit(CurCSample,-2048,2048);
        break;

        case 6:
            if(X->AxisID == AXIS_RIGHT)
            {
                CurBSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0) - ADC2_JDR1_Offset;
            }
            else if(X->AxisID == AXIS_LEFT)
            {
                CurBSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - ADC1_JDR1_Offset;
            }

            if(CurBSample - P->CurPhaB_Data[0] >  CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] + CurErrDataB;
            }
            else if(CurBSample - P->CurPhaB_Data[0] < -CurErrDataB)
            {
                CurBSample = P->CurPhaB_Data[0] - CurErrDataB;
            }
            ValueLimit(CurBSample,-2048,2048);

            if(P->CurEffective != 0)
            {
                if(X->AxisID == AXIS_RIGHT)
                {
                    CurCSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1) - ADC2_JDR2_Offset;
                }
                else if(X->AxisID == AXIS_LEFT)
                {
                    CurCSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - ADC1_JDR2_Offset;
                }

                if(CurCSample - P->CurPhaC_Data[0] >  CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
                }
                else if(CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
                }
                ValueLimit(CurCSample,-2048,2048);
            }
            else
            {
                CurErrDataTemp = CurBSample - P->CurPhaB_Data[0];
                CurErrDataTemp = abs(CurErrDataTemp);
                if((P->CurPhaC_Data[0] - P->CurPhaC_Data[1] >= 0) && (P->CurPhaC_Data[1] - P->CurPhaC_Data[2] >= 0))
                {
                    CurCSample = P->CurPhaC_Data[0] + CurErrDataTemp;
                }
                else if((P->CurPhaC_Data[0] - P->CurPhaC_Data[1] < 0) && (P->CurPhaC_Data[1] - P->CurPhaC_Data[2] < 0))
                {
                    CurCSample = P->CurPhaC_Data[0] - CurErrDataTemp;
                }
                else if((P->CurPhaC_Data[0] - P->CurPhaC_Data[1] >= 0) && (P->CurPhaC_Data[1] - P->CurPhaC_Data[2] < 0))
                {
                    CurCSample = (3 * P->CurPhaC_Data[0] + CurErrDataTemp  - P->CurPhaC_Data[1]) >> 1;
                }
                else
                {
                    CurCSample = (3 * P->CurPhaC_Data[0] - CurErrDataTemp  - P->CurPhaC_Data[1])>>1;
                }

                if(CurCSample - P->CurPhaC_Data[0] > CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] + CurErrDataC;
                }
                else if (CurCSample - P->CurPhaC_Data[0] < -CurErrDataC)
                {
                    CurCSample = P->CurPhaC_Data[0] - CurErrDataC;
                }
                ValueLimit(CurCSample,-2048,2048);
            }
            // Ia = -Ic -Ib
            CurASample = -CurBSample-CurCSample;
            if(CurASample - P->CurPhaA_Data[0] > CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] + CurErrDataA;
            }
            else if(CurASample - P->CurPhaA_Data[0] < -CurErrDataA)
            {
                CurASample = P->CurPhaA_Data[0] - CurErrDataA;
            }
						ValueLimit(CurASample,-2048,2048);
            break;
        default:
					  if(X->AxisID == AXIS_RIGHT)
            {
                CurASample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0) - ADC2_JDR0_Offset;
            }
            else if(X->AxisID == AXIS_LEFT)
            {
                CurASample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - ADC1_JDR0_Offset;
            }
						
						if(X->AxisID == AXIS_RIGHT)
						{
								CurBSample = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1) - ADC2_JDR1_Offset;
						}
						else if(X->AxisID == AXIS_LEFT)
						{
								CurBSample = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - ADC1_JDR1_Offset;
						}
						
						CurCSample = -CurASample - CurBSample;
            break;
    }

    if(X->PowerFlag == 1)
    {
        P->CurPhaA_Data[2] = P->CurPhaA_Data[1];
        P->CurPhaB_Data[2] = P->CurPhaB_Data[1];
        P->CurPhaC_Data[2] = P->CurPhaC_Data[1];
        P->CurPhaA_Data[1] = P->CurPhaA_Data[0];
        P->CurPhaB_Data[1] = P->CurPhaB_Data[0];
        P->CurPhaC_Data[1] = P->CurPhaC_Data[0];
        P->CurPhaA_Data[0] = CurASample;
        P->CurPhaB_Data[0] = CurBSample;
        P->CurPhaC_Data[0] = CurCSample;
    }
    else
    {
        P->CurPhaA_Data[2] = 0;
        P->CurPhaB_Data[2] = 0;
        P->CurPhaC_Data[2] = 0;
        P->CurPhaA_Data[1] = 0;
        P->CurPhaB_Data[1] = 0;
        P->CurPhaC_Data[1] = 0;
        P->CurPhaA_Data[0] = 0;
        P->CurPhaB_Data[0] = 0;
        P->CurPhaC_Data[0] = 0;
    }
//ADC2_JDR0_GAIN 0.2685547  -
    if(X->AxisID == AXIS_RIGHT)    
    {
					P->Ia = CurASample * ADC2_JDR0_GAIN;
					P->Ib = CurBSample * ADC2_JDR1_GAIN;
					P->Ic = CurCSample * ADC2_JDR2_GAIN;
    }
    else if(X->AxisID == AXIS_LEFT)
    {
					P->Ia = CurASample * ADC1_JDR0_GAIN;
					P->Ib = CurBSample * ADC1_JDR1_GAIN;
					P->Ic = CurCSample * ADC1_JDR2_GAIN;
    }

		


		
    //清楚采样触发标志
    if(X->AxisID == AXIS_RIGHT)
    {
        if(((ADC_STAT(ADC2) & ADC_STAT_STIC)) && ((ADC_STAT(ADC2) & ADC_STAT_EOIC)))
				{
				    ADC_STAT(ADC2)&=(~ADC_STAT_STIC);
					  ADC_STAT(ADC2)&=(~ADC_STAT_EOIC);
					  ADC_STAT(ADC2)&=(~ADC_STAT_EOC);
				}
    }
    else if(X->AxisID == AXIS_LEFT)
    {
        if(((ADC_STAT(ADC1) & ADC_STAT_STIC)) && ((ADC_STAT(ADC1) & ADC_STAT_EOIC)))
				{
				    ADC_STAT(ADC1)&=(~ADC_STAT_STIC);
					  ADC_STAT(ADC1)&=(~ADC_STAT_EOIC);
					  ADC_STAT(ADC1)&=(~ADC_STAT_EOC);
				}	
    }
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ValueLimit(INT16 ValueTemp,INT16 MinLimit,INT16 MaxLimit)
{
    if(ValueTemp < MinLimit)
    {
        ValueTemp = MinLimit;
    }
    else if(ValueTemp > MaxLimit)
    {
        ValueTemp = MaxLimit;
    }
}

