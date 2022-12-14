/*******************************************************************
 *
 * FILE NAME:  Alarm.c
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

#include "HardApi.h"
#include "gpio.h"
#include "Param.h"
#include "ControlRun.h"
#include "StateMachine.h"
#include "CanApp.h"
#include "LedDriver.h"
#include "gd_hal.h"
#include "math.h"

extern struct CanAppStruct    sMyCan;
extern struct AxisCtrlStruct sAxis[MAX_AXIS_NUM];
extern struct PowerManagerStruct sPowerManager;
extern PUBLIC UINT8 ApplicationMode;

#define IDC_MAX_LEVEL

#define PHASE_CURRENT_OVER_TIME  6000  //600ms

#define TEMPERATURE_TIME     (20)  //10ms

#define BEMF_DISCHATGE_TIME  600   // 300ms = 6 *500us
#define VDC_UNDER_TIME       200   // 100ms   
#define MAX_SPD_TIME         20    // 10ms
#define MOTOR_STALL_TIME     400   // 200ms
#define MOTOR_STALL_SPEED    10.0f    // 10rpm
#define HALL_ALARM_MAX_CNT   1

//#define I2T_TIME             500  // 100ms

#define NONE_MOTION_IQ            1.0f
#define IBUS_CUROVER_JUDGE_TIME   100  // 10ms
#define MAX_CONTINUE_IBUS         15.0f
#define MTR_OV_LD_TB_END_ID  9

#define PWMOUT_DETECT_WINDOW      1000 //500ms
const UINT16 invOvLdTb[MTR_OV_LD_TB_END_ID+1]=
{
    4000,   //110%------400s
    2300,	//120%------230s
    1200,	//130%------120s
    900,	//140%------90s
    600, 	//150%------60s
    200, 	//160%------20s
    150, 	//170%------15s
    100,	//180%------10s
    80,		//190%------8s
    80,  	//200%------8s
};

const UINT16 invUnOvLdTb[MTR_OV_LD_TB_END_ID+1]=
{
    80,    //8s
    80,
    100,
    150,
    200,
    600,
    900,
    1200,
    2300,
    4000,  //400s
};
const UINT16 invCurLdTb[MTR_OV_LD_TB_END_ID+1]=
{
    1100,
    1200, 	//120%
    1300, 	//130%
    1400, 	//140%
    1500, 	//150%
    1600, 	//160%
    1700, 	//170%
    1800, 	//180%
    1900,	  //190%
    2000,	  //200%
};

const UINT16 invUnCurLdTb[MTR_OV_LD_TB_END_ID+1]=
{
    100,   //10%
    200, 	//20%
    300, 	//30%
    400, 	//40%
    500, 	//50%
    600, 	//60%
    700, 	//70%
    800, 	//80%
    900,	//90%
    1000,	//100%
};

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AlarmInit(struct AxisCtrlStruct *P)
{
    struct AlarmStruct *pAlarm = &P->sAlarm;

//    pAlarm->I2T_Setpoint = I2T_TIME*(P->sCurLoop.M_PeakI*P->sCurLoop.M_PeakI
//                            - P->sCurLoop.M_RatedI*P->sCurLoop.M_RatedI);

    pAlarm->VdcMax = (float)gParam[P->AxisID].VdcLimitMax0x2004*0.001f;
    pAlarm->VdcMin = (float)gParam[P->AxisID].VdcLimitMin0x2005*0.001f;
    pAlarm->VdcWarn = pAlarm->VdcMax*0.9f;

    pAlarm->MotorMaxSpd = (float)gParam[P->AxisID].MotorMaxSpeed0x2200;
    pAlarm->TempLimitMax = (INT16)gParam[P->AxisID].TempLimitMax0x2006;

    pAlarm->PhaseCurrentLimit = 0.001f*(float)gParam[P->AxisID].CurrentLimit0x2003;

    // ???????????????o??o?????????????3???????????????????????????C
    pAlarm->IuTotle = 0;
    pAlarm->IvTotle = 0;
    pAlarm->IwTotle = 0;
    pAlarm->phaselose_time = 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
//PUBLIC void ExitAlarmExec(void)
//{
//    UINT8 i;
//
//    PwmDisable(0);
//    PwmDisable(1);
//    PwmUpdate(sAxis[0].AxisID, POWER_OFF, 0, 0, 0);
//    PwmUpdate(sAxis[1].AxisID, POWER_OFF, 0, 0, 0);
//
//    if (GetHardOverCurState(0))
//    {
//        sAxis[0].sAlarm.ErrReg.bit.HardCurOver = 1;
//        sAxis[0].PowerEn = 0;
//        sAxis[0].sAlarm.ErrReg.bit.HardCurOver = 1;
//        sAxis[0].PowerEn = 0;
//    }
//}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AlarmExec(struct AxisCtrlStruct *P)
{
    struct AlarmStruct *pAlarm = &P->sAlarm;
    REAL32 Imax,Imin;
    UINT16  Safe_IO = 0;
    // Inner Alarm : sheduler error
    if(GetIsrElapsedTime() > MAX_PWM_ISR_TIME)
    {
        pAlarm->ErrReg.bit.InnerErr = 1;
    }

    if((P->PowerFlag == POWER_ON)
            && ((P->sSpdLoop.SpdFdb > MOTOR_STALL_SPEED) || (P->sSpdLoop.SpdFdb < -MOTOR_STALL_SPEED ))
            //&& ((P->sSpdLoop.SpdRefActul > MOTOR_STALL_SPEED) || (P->sSpdLoop.SpdRefActul < -MOTOR_STALL_SPEED ))
      )
    {
        // out put phase loes  // ok
        if((pAlarm->phaselose_time < 10000.0f) && (pAlarm->ErrReg.bit.OutLosePhase == 0)) // 1s
        {
            pAlarm->phaselose_time ++;
            pAlarm->IuTotle += (P->sCurLoop.Ia > 0.0f ?  P->sCurLoop.Ia : (- P->sCurLoop.Ia));
            pAlarm->IvTotle += (P->sCurLoop.Ib > 0.0f ?  P->sCurLoop.Ib : (- P->sCurLoop.Ib));
            pAlarm->IwTotle += (P->sCurLoop.Ic > 0.0f ?  P->sCurLoop.Ic : (- P->sCurLoop.Ic));
        }
        else
        {
            Imax = (pAlarm->IuTotle > pAlarm->IvTotle) ? pAlarm->IuTotle : pAlarm->IvTotle;
            Imax = (Imax > pAlarm->IwTotle) ? Imax : pAlarm->IwTotle;
            Imin = (pAlarm->IuTotle < pAlarm->IvTotle) ? pAlarm->IuTotle : pAlarm->IvTotle;
            Imin = (Imin < pAlarm->IwTotle) ? Imin : pAlarm->IwTotle;
            Imax = Imax/pAlarm->phaselose_time;
            Imin = Imin/pAlarm->phaselose_time;
            pAlarm->phaselose_time = 0;
            pAlarm->IuTotle = 0;
            pAlarm->IvTotle = 0;
            pAlarm->IwTotle = 0;
            if((Imax > 0.8f) && ((Imax/Imin) > 8.0f))
            {
                pAlarm->ErrReg.bit.OutLosePhase = 1;
            }
        }
    }
    else
    {
        pAlarm->phaselose_time = 0;
        pAlarm->IuTotle = 0;
        pAlarm->IvTotle = 0;
        pAlarm->IwTotle = 0;
    }
    if(P->PowerFlag == POWER_ON)
    {
        if((fabs(P->sCurLoop.VqRef)>0.98f)&&(P->sCurLoop.IValidFdb<1.0f)&&(fabs(P->sSpdLoop.SpdFdb)<30.0f))
        {
            pAlarm->phaselose_cnt++;
        }
        else
        {
            if(pAlarm->phaselose_cnt>0)
                pAlarm->phaselose_cnt--;
        }
        if(pAlarm->phaselose_cnt>1000)//100ms
        {
            pAlarm->ErrReg.bit.OutLosePhase = 1;
            pAlarm->phaselose_cnt = 0;
        }
    }
    else
    {
        pAlarm->phaselose_cnt = 0;
    }

    // hall alarm judgement
    if (P->sEncoder.HallEnable == 1)  //????????????
    {
        if(P->sEncoder.HallState > 6)
        {
            pAlarm->HallErrCnt++;
        }
        else if(pAlarm->HallErrCnt)
        {
            pAlarm->HallErrCnt--;
        }

        // hall alarm
        if(pAlarm->HallErrCnt >= HALL_ALARM_MAX_CNT)
        {
            pAlarm->ErrReg.bit.HallErr = 1;
            pAlarm->HallErrCnt = HALL_ALARM_MAX_CNT;
        }
				
				//??????????????????
				if(P->AxisID)
				{						
						if(pAlarm->RisingCnt > 1000) // PWM????????????
						{
									pAlarm->ErrReg.bit.MotorMatchErr = 1;
									pAlarm->RisingCnt = 0;
						}
				}
				else //left
				{	
						if(pAlarm->RisingCnt > 1000)// PWM????????????
						{
								pAlarm->ErrReg.bit.MotorMatchErr = 1;
								pAlarm->RisingCnt = 0;
						}
				}							
		}			
		else if (P->sEncoder.HallEnable == 2)
		{
				if(P->AxisID) //right
				{
						if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
						{
									pAlarm->PwmOnCnt++;
									pAlarm->PwmOffCnt = 0;
						}
						else
						{
									pAlarm->PwmOffCnt++;
									pAlarm->PwmOnCnt = 0;
						}
						
						if((pAlarm->PwmOffCnt > 40000)||(pAlarm->PwmOnCnt > 40000)) //4s PWM????????????????????? ???????????????????????????
						{
								 	pAlarm->ErrReg.bit.PwmoutBreak = 1;
									pAlarm->PwmOffCnt=0;
									pAlarm->PwmOnCnt = 0;
						}
				}
				else //left
				{
						if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
						{
									pAlarm->PwmOnCnt++;
									pAlarm->PwmOffCnt = 0;
						}
						else
						{
									pAlarm->PwmOffCnt++;
									pAlarm->PwmOnCnt = 0;
						}
						if((pAlarm->PwmOffCnt > 40000)||(pAlarm->PwmOnCnt > 40000))//4s PWM????????????????????? ???????????????????????????
						{
								pAlarm->ErrReg.bit.PwmoutBreak = 1;
								pAlarm->PwmOffCnt=0;
								pAlarm->PwmOnCnt = 0;
						}
				}
				if(P->sEncoder.PosCorrectEnUsingPwmout)
				{
						pAlarm->PwmOffCnt=0;
						pAlarm->PwmOnCnt = 0;
				}
		}
		else
		{
				pAlarm->PwmOnCnt = 0;
				pAlarm->PwmOffCnt = 0;
		}
		
    Safe_IO = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
    if((Safe_IO == 0)&&(pAlarm->ErrReg.bit.StutterStop))
    {
        pAlarm->EmergencyStopRstCnt++;
#define WAIT_SOFT_START_FINISH (3000)
#define SOFT_START_BEGIN (3)

        if (pAlarm->EmergencyStopRstCnt > WAIT_SOFT_START_FINISH)
        {
            pAlarm->ErrReg.bit.StutterStop = 0;
            ClearScramStatus(P->AxisID);
        }
        else if (pAlarm->EmergencyStopRstCnt > SOFT_START_BEGIN)
        {
            if ((!sPowerManager.sBoardPowerInfo.VbusSoftStartFlag) && (!sPowerManager.sBoardPowerInfo.VbusSoftStartEn))
            {
                sPowerManager.sBoardPowerInfo.VbusSoftStartEn = 1;
            }
        }
    }
    else if(Safe_IO == 1)// && (pAlarm->ErrReg.bit.StutterStop == 0))
    {
        pAlarm->ErrReg.bit.StutterStop = 1;
        pAlarm->EmergencyStopRstCnt = 0;
        sPowerManager.sBoardPowerInfo.VbusSoftStartFlag = 0;
        sPowerManager.sBoardPowerInfo.VbusSoftStartEn = 0;
        VbusDisable();
        SetVbusPower(0);
    }

    if (2 == sPowerManager.sBoardPowerInfo.VbusSoftStartFlag)
    {
        sAxis[0].sAlarm.ErrReg.bit.VbusSSMosOpenCircuitFailure = 1;
        sAxis[1].sAlarm.ErrReg.bit.VbusSSMosOpenCircuitFailure = 1;
    }
    else if (1 == sPowerManager.sBoardPowerInfo.VbusSoftStartFlag)
    {
        sAxis[0].sAlarm.ErrReg.bit.VbusSSMosOpenCircuitFailure = 0;
        sAxis[1].sAlarm.ErrReg.bit.VbusSSMosOpenCircuitFailure = 0;
    }
    else
    {
        if ((P->AxisID) && (sPowerManager.sBoardPowerInfo.VbusSoftStartEn))
        {
            VbusSoftStartNoBlock(GetDcVoltageNoFilter());
        }
    }
		
		static UINT8 ASC711PWFlag = 0;
		static UINT8 S_CheckDelayCnt = 0;
		if((GetBatteryVoltage() >= 20.0f) && (ASC711PWFlag == 0))
		{ 
			 ASC711PWFlag = 1;
		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		else if((GetBatteryVoltage() < 16.0f) && (ASC711PWFlag == 1))
		{
			 ASC711PWFlag = 0;
		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		}
		
		if(ASC711PWFlag)
		{
		  S_CheckDelayCnt++;
			if(S_CheckDelayCnt >= 20)
				S_CheckDelayCnt = 20;
		}
		else
			S_CheckDelayCnt = 0;
		
    if((GetIbusOverCurState(P->AxisID)) && (S_CheckDelayCnt >= 10)) 
    {
        pAlarm->ErrReg.bit.BusCurOver = 1;
    }
		
#if 0
    if(((P->sCurLoop.Ia > pAlarm->PhaseCurrentLimit) || (P->sCurLoop.Ia < -pAlarm->PhaseCurrentLimit))
            &&((P->sSpdLoop.SpdFdb < MOTOR_STALL_SPEED) && (P->sSpdLoop.SpdFdb > -MOTOR_STALL_SPEED )))
    {
        pAlarm->IuOverCnt++;
    }
    else if(pAlarm->IuOverCnt)
    {
        pAlarm->IuOverCnt --;
    }
    if(((P->sCurLoop.Ib > pAlarm->PhaseCurrentLimit) || (P->sCurLoop.Ib < -pAlarm->PhaseCurrentLimit))
            &&((P->sSpdLoop.SpdFdb < MOTOR_STALL_SPEED) && (P->sSpdLoop.SpdFdb > -MOTOR_STALL_SPEED )))
    {
        pAlarm->IvOverCnt++;
    }
    else if(pAlarm->IvOverCnt)
    {
        pAlarm->IvOverCnt --;
    }
    if(((P->sCurLoop.Ic > pAlarm->PhaseCurrentLimit) || (P->sCurLoop.Ic < -pAlarm->PhaseCurrentLimit))
            &&((P->sSpdLoop.SpdFdb < MOTOR_STALL_SPEED) && (P->sSpdLoop.SpdFdb > -MOTOR_STALL_SPEED )))
    {
        pAlarm->IwOverCnt++;
    }
    else if(pAlarm->IwOverCnt)
    {
        pAlarm->IwOverCnt --;
    }
    if(  pAlarm->IuOverCnt > PHASE_CURRENT_OVER_TIME
            || pAlarm->IvOverCnt > PHASE_CURRENT_OVER_TIME
            || pAlarm->IwOverCnt > PHASE_CURRENT_OVER_TIME)
    {
        pAlarm->ErrReg.bit.PhaseCurOver = 1;
    }
#endif
    REAL32 tmp = P->sCurLoop.IValidFdb * 1.28f; //IValidFdb????????????????????????????????? 12*1.414/1.28 = 13.3A
    if(tmp > pAlarm->PhaseCurrentLimit)
    {
        pAlarm->IuOverCnt++;
    }
    else if(pAlarm->IuOverCnt)
    {
        pAlarm->IuOverCnt --;
    }
    if( pAlarm->IuOverCnt > PHASE_CURRENT_OVER_TIME) //600ms
    {
        pAlarm->ErrReg.bit.PhaseCurOver = 1;
    }

    if(pAlarm->ErrReg.all>>16)
    {
        pAlarm->ErrReg.bit.FatalErr = 1;
    }
    else
    {
        pAlarm->ErrReg.bit.FatalErr = 0;
    }

    if(pAlarm->ErrReg.all & (gParam[P->AxisID].AlarmSwitch0x2007 | 0xFFFF0000))
    {
        CiA402_LocalError(P->AxisID, pAlarm->ErrReg.all);
    }

    if ((pAlarm->ErrReg.bit.BusCurOver) || (pAlarm->ErrReg.bit.SpdOver) || (pAlarm->ErrReg.bit.StutterStop))
    {
        P->PowerEn = 0;
    }
    if ((!P->sEncoder.InitPosDoneUsingPwmout) && (P->sEncoder.HallEnable == 2))
    {
        P->PowerEn = 0;
    }
}

/***********************************************************************
 * DESCRIPTION:execute every 5 pwm cycles
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AlarmExec_5(struct AxisCtrlStruct *P)
{
    struct AlarmStruct *pAlarm = &P->sAlarm;
    UINT32 m_Data,m_LDeta;
    UINT16 m_Index,i_index,m_Cur;
    INT32  SpdErr_Threshold;

    // Eeprom Alarm
    if(GetEepromErrorCode() & (P->AxisID+1) )
    {
        pAlarm->ErrReg.bit.EepromErr = 1;
    }

    // Current ADC initiation alarm
    if(GetAdcInitState(P->AxisID))
    {
        pAlarm->ErrReg.bit.CurInit = 1;
    }
    // Vdc judgement
    if(P->sCurLoop.Vdc > pAlarm->VdcMax)
    {
        pAlarm->VdcOverCnt = BEMF_DISCHATGE_TIME+1;
    }
    else if(P->sCurLoop.Vdc > pAlarm->VdcWarn && !pAlarm->ErrReg.bit.VdcOver)
    {
        pAlarm->VdcOverCnt++;
    }
    else if(P->sCurLoop.Vdc > pAlarm->VdcMin)
    {
        if(pAlarm->VdcUnderCnt)
        {
            pAlarm->VdcUnderCnt--;
        }

        if(P->sCurLoop.Vdc < pAlarm->VdcWarn -2)
        {
            if(pAlarm->VdcOverCnt)
            {
                pAlarm->VdcOverCnt--;
            }
        }
        else
        {
            pAlarm->VdcOverCnt++;
        }
    }
    else
    {
        pAlarm->VdcOverCnt = 0;
        if (sPowerManager.sBoardPowerInfo.VbusSoftStartFlag)
        {
            pAlarm->VdcUnderCnt++;
        }
    }

    // Vdc Over Alarm
    if(pAlarm->VdcOverCnt >= BEMF_DISCHATGE_TIME)
    {
        pAlarm->ErrReg.bit.VdcOver = 1;
        pAlarm->VdcOverCnt = BEMF_DISCHATGE_TIME;
    }
    // Vdc Under Alarm
    else if((pAlarm->VdcUnderCnt >= VDC_UNDER_TIME)&&(sAxis[0].sAlarm.ErrReg.bit.StutterStop ==0)&&(sAxis[1].sAlarm.ErrReg.bit.StutterStop ==0))
    {
        pAlarm->ErrReg.bit.VdcUnder = 1;
        pAlarm->VdcUnderCnt = VDC_UNDER_TIME;
    }

    //Pwmout Break Alarm
    if (P->sEncoder.HallEnable == 2)
    {
        if (P->sEncoder.PosCorrectEnUsingPwmout)
        {
            pAlarm->PwmoutDisconnectCnt++;
        }
        else
        {
            pAlarm->PwmoutDisconnectCnt = 0;
        }
#define MAX_PWMOUT_DISCONNECT   (1000)
        if (pAlarm->PwmoutDisconnectCnt >= MAX_PWMOUT_DISCONNECT)
        {
            pAlarm->PwmoutDisconnectCnt = MAX_PWMOUT_DISCONNECT;
            pAlarm->ErrReg.bit.PwmoutBreak = 1;
        }
    }

    if(gParam[P->AxisID].MosTemp0x230B > pAlarm->TempLimitMax)
//        || gParam[P->AxisID].MotorTemp0x230C > pAlarm->TempLimitMax)
    {
        pAlarm->MosTempOverCnt++;
    }
    else if(pAlarm->MosTempOverCnt)
    {
        pAlarm->MosTempOverCnt--;
    }

    // Over temperature alarm
    if(pAlarm->MosTempOverCnt >= TEMPERATURE_TIME)
    {
        pAlarm->ErrReg.bit.MosTempOver = 1;
        pAlarm->MosTempOverCnt = TEMPERATURE_TIME;
    }

    if(gParam[P->AxisID].MotorTemp0x230C > pAlarm->TempLimitMax)
    {
        pAlarm->MotorTempOverCnt++;
    }
    else if(pAlarm->MotorTempOverCnt)
    {
        pAlarm->MotorTempOverCnt--;
    }

    // Over temperature alarm
    if(pAlarm->MotorTempOverCnt >= TEMPERATURE_TIME)
    {
//        pAlarm->ErrReg.bit.MotorTempOver = 1;
        pAlarm->MotorTempOverCnt = TEMPERATURE_TIME;
    }

    if(P->PowerFlag == POWER_ON)
    {
        // Over Load Alarm
#if 0
        pAlarm->I2T_accumulator += (P->sCurLoop.IqFdb*P->sCurLoop.IqFdb 		//?a?????????????????????????????????????IQ??????????????????|????????????????????????????????????????????????o?????????o???????o??????o????????
                                    - P->sCurLoop.M_RatedI*P->sCurLoop.M_RatedI);

        if(pAlarm->I2T_accumulator >= pAlarm->I2T_Setpoint)
        {
            pAlarm->ErrReg.bit.MotorIIt = 1;
        }
        else if(pAlarm->I2T_accumulator < 0.0f)
        {
            pAlarm->I2T_accumulator = 0.0f;
        }
#else

        m_Cur = P->sCurLoop.IValidFdb * 1000.0f / P->sCurLoop.M_RatedI;

        if(m_Cur < 1000)
        {
            if(m_Cur < 100)
                m_Data = invUnOvLdTb[0];
            else
            {
                for(i_index = 0; i_index < MTR_OV_LD_TB_END_ID; i_index++)
                {
                    if((m_Cur >= invUnCurLdTb[i_index]) && (m_Cur < invUnCurLdTb[i_index+1]))
                    {
                        m_Index = i_index;
                        m_Data  = (m_Cur - invUnCurLdTb[m_Index])*(invUnOvLdTb[m_Index+1]-invUnOvLdTb[m_Index]);
                        m_Data  = invUnOvLdTb[m_Index] + m_Data / (invUnCurLdTb[m_Index+1] - invUnCurLdTb[m_Index]);
                        break;
                    }
                }
            }
            m_LDeta = (4000UL) / m_Data;
            if(pAlarm->I2T_accumulator>=m_LDeta)
                pAlarm->I2T_accumulator = pAlarm->I2T_accumulator - m_LDeta;		//
            else
                pAlarm->I2T_accumulator = 0;
        }
        else
        {
            if(m_Cur >= 2000)
            {
                m_Data = invOvLdTb[MTR_OV_LD_TB_END_ID];
            }
            else if(m_Cur < 1100)
            {
                m_Data = 4000ul;
            }
            else//1100<= m_Cur <2000
            {
                for(i_index = 0; i_index < MTR_OV_LD_TB_END_ID; i_index++)
                {
                    if((m_Cur >= invCurLdTb[i_index]) && (m_Cur < invCurLdTb[i_index+1]))
                    {
                        m_Index = i_index;
                        m_Data  = (m_Cur - invCurLdTb[m_Index])*(invOvLdTb[m_Index]-invOvLdTb[m_Index+1]);
                        m_Data  = invOvLdTb[m_Index] - m_Data / (invCurLdTb[m_Index+1] - invCurLdTb[m_Index]);
                        break;
                    }
                }
            }
            m_LDeta = (4000UL) / m_Data;
            pAlarm->I2T_accumulator = pAlarm->I2T_accumulator + m_LDeta;		//
            //pAlarm->I2T_accumulator = MIN(temp32U,(32700UL<<16));
        }

        if(pAlarm->I2T_accumulator > 800000UL)
        {
            pAlarm->ErrReg.bit.MotorIIt = 1;
        }
#endif
        // Motor Stall judgement
        /*        float StallCur = P->sCurLoop.M_RatedI*1.414f;
                if((P->sCurLoop.IqFdb > StallCur || P->sCurLoop.IqFdb < -StallCur)
                    && P->sSpdLoop.SpdFdb < MOTOR_STALL_SPEED && P->sSpdLoop.SpdFdb > -MOTOR_STALL_SPEED
                    //&& (P->sSpdLoop.SpdRef > MOTOR_STALL_SPEED || P->sSpdLoop.SpdRef < -MOTOR_STALL_SPEED )
                    && (P->sSpdLoop.SpdErr > MOTOR_STALL_SPEED || P->sSpdLoop.SpdErr < -MOTOR_STALL_SPEED ))//2019.8.14 change by hp
                {
                    pAlarm->StallCnt++;
                }
                else if(pAlarm->StallCnt)
                {
                    pAlarm->StallCnt--;
                }

                // Motor Stall Alarm
                if(pAlarm->StallCnt >= MOTOR_STALL_TIME)//
                {
                    pAlarm->ErrReg.bit.MotorStall = 1;
                    pAlarm->StallCnt = MOTOR_STALL_TIME;
                }
        */
        // Over Speed judgement
        if(P->sSpdLoop.SpdFdb > pAlarm->MotorMaxSpd || P->sSpdLoop.SpdFdb < -pAlarm->MotorMaxSpd) //
        {
            pAlarm->SpdOverCnt++;
        }
        else if(pAlarm->SpdOverCnt)
        {
            pAlarm->SpdOverCnt--;
        }

        // Over Speed Alarm
        if(pAlarm->SpdOverCnt >= MAX_SPD_TIME)
        {
            pAlarm->ErrReg.bit.SpdOver = 1;
            pAlarm->SpdOverCnt = MAX_SPD_TIME;
        }

        if((P->CtrlMode == SPD_CTRL || P->CtrlMode == POS_CTRL) && (P->PowerEn == POWER_ON))
        {
            // Speed Following Error judgement
            SpdErr_Threshold = gParam[P->AxisID].SpeedFollowErrWindow0x2008;
			if(fabs(P->sSpdLoop.SpdErr) > SpdErr_Threshold)
			{
				pAlarm->SpdFollwCnt++;
			}
			else if(pAlarm->SpdFollwCnt)
			{
				pAlarm->SpdFollwCnt--;
			}

            // Speed Following Error Alarm
            UINT16 SpdFollowTime = gParam[P->AxisID].SpeedFollowErrTime0x2009*20; //30r/min 400*20/2 = 4000ms   60r/min 1s
            if(pAlarm->SpdFollwCnt >= SpdFollowTime)
            {
                pAlarm->ErrReg.bit.SpeedFollow = 1;
                pAlarm->SpdFollwCnt = SpdFollowTime;
            }

        } // end if(P->CtrlMode..

    } // end if(P->PowerFlag == POWER_ON)
    else
    {
        pAlarm->I2T_accumulator = 0;
        pAlarm->SpdFollwCnt = 0;
    }
    // can lost alarm
    if(sMyCan.CanLostErr)
    {
        sAxis[0].sAlarm.ErrReg.bit.CanLost = 1;
        sAxis[1].sAlarm.ErrReg.bit.CanLost = 1;
    }

    if(sMyCan.CanBreakErr)
    {
        sAxis[0].sAlarm.ErrReg.bit.CanBreakErr = 1;
        sAxis[1].sAlarm.ErrReg.bit.CanBreakErr = 1;
    }

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ClearAlarm(struct AxisCtrlStruct *P)
{
    struct AlarmStruct *pAlarm = &P->sAlarm;

    pAlarm->ErrReg.all = pAlarm->ErrReg.all & 0xFF870001;
    pAlarm->I2T_accumulator = 0.0f;
    pAlarm->SpdOverCnt = 0;
    pAlarm->StallCnt = 0;
    pAlarm->VdcOverCnt = 0;
    pAlarm->VdcUnderCnt = 0;
    pAlarm->TempOverCnt = 0;
    pAlarm->MosTempOverCnt = 0;
    pAlarm->MotorTempOverCnt = 0;
    pAlarm->SpdFollwCnt = 0;
    pAlarm->HallErrCnt  = 0;
    pAlarm->BusCurOverCnt = 0;
//	pAlarm->EmergencyStopRstCnt = 0;
//    pAlarm->EmergencyStopSetCnt = 0;
    pAlarm->IuOverCnt = 0;
    pAlarm->IvOverCnt = 0;
    pAlarm->IwOverCnt = 0;

    sMyCan.CanLostErr = 0;
    sMyCan.CanBreakErr = 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ClearCanBreakAlarm(void)
{
    sMyCan.CanBreakErr = 0;
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void AlarmLoop(struct AxisCtrlStruct *P)
{


}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void EepromAlarmExec(UINT16 AxisID)
{


}
