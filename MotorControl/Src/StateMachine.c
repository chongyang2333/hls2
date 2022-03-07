/*******************************************************************
 *
 * FILE NAME:  StateMachine.c
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2018.12.28
 *
 * AUTHOR:      Denny
 *
 * History:
------------------------------------------------------------------------
28-12-2018 Version 1.00 : Created by Denny
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/
#define _CiA402_
#include "StateMachine.h"
#undef _CiA402_
#include "ControlRun.h"
#include "Param.h"
#include "ErrorLog.h"

TCiA402Axis       LocalAxes[2]={0};


extern struct AxisCtrlStruct sAxis[MAX_AXIS_NUM];

extern PUBLIC void ClearAlarm(struct AxisCtrlStruct *P);

/***********************************************************************
 * DESCRIPTION: This function initializes the Axes structures
 *
 * RETURNS:
 *
***********************************************************************/
void CiA402_Init(void)
{
    UINT16 AxisCnt = 0;

    for(AxisCnt = 0; AxisCnt < 2 ; AxisCnt++)
    {
        LocalAxes[AxisCnt].bAxisIsActive = TRUE;
        LocalAxes[AxisCnt].bBrakeApplied = TRUE;
        LocalAxes[AxisCnt].bLowLevelPowerApplied = TRUE;
        LocalAxes[AxisCnt].bHighLevelPowerApplied = FALSE;
        LocalAxes[AxisCnt].bAxisFunctionEnabled = FALSE;
        LocalAxes[AxisCnt].bConfigurationAllowed = TRUE;

        LocalAxes[AxisCnt].i16State = STATE_NOT_READY_TO_SWITCH_ON;

    }// for "all active axes"

}

/***********************************************************************
 * DESCRIPTION: This function handles the state machine for devices 
 *              using the CiA402 profile.
 * RETURNS:
 *
***********************************************************************/
void CiA402_StateMachine(void)
{
    TCiA402Axis *pCiA402Axis;
    UINT16 StatusWord = 0;
    UINT16 ControlWord6040 = 0;
    UINT16 counter = 0;

    for(counter = 0; counter < 2;counter++)
    {
        pCiA402Axis = &LocalAxes[counter];
        StatusWord = gParam[counter].StatusWord0x6041;
        ControlWord6040 = gParam[counter].ControlWord0x6040;

        /*clear statusword state and controlword processed complete bits*/
        StatusWord &= ~(STATUSWORD_STATE_MASK | STATUSWORD_REMOTE);

        /*skip transition 1 and 2*/
//      pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON;

        switch(pCiA402Axis->i16State)
        {
        case STATE_NOT_READY_TO_SWITCH_ON:
            StatusWord |= (STATUSWORD_STATE_NOTREADYTOSWITCHON);
            pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 1

            break;
        
        case STATE_SWITCH_ON_DISABLED:
            StatusWord |= (STATUSWORD_STATE_SWITCHEDONDISABLED);
            if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 2
            }
            break;
            
        case STATE_READY_TO_SWITCH_ON:
            StatusWord |= (STATUSWORD_STATE_READYTOSWITCHON);
            if (((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
                || ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
            {
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 7
            } 
            else if (((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_MASK) == CONTROLWORD_COMMAND_SWITCHON) ||
                    ((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION))
            {
                pCiA402Axis->i16State = STATE_SWITCHED_ON;           // Transition 3
            }
            break;
                
        case STATE_SWITCHED_ON:
            StatusWord |= (STATUSWORD_STATE_SWITCHEDON);

            if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 6

            } 
            else if (((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP
                    || (ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
            {
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 10

            } 
            else if ((ControlWord6040 & CONTROLWORD_COMMAND_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_ENABLEOPERATION)
            {
                pCiA402Axis->i16State = STATE_OPERATION_ENABLED;  // Transition 4
                //The Axis function shall be enabled and all internal set-points cleared.
            }
            break;
                
        case STATE_OPERATION_ENABLED:
            StatusWord |= (STATUSWORD_STATE_OPERATIONENABLED);

            if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEOPERATION_MASK) == CONTROLWORD_COMMAND_DISABLEOPERATION)
            {
                pCiA402Axis->i16State = STATE_SWITCHED_ON;           // Transition 5
            } 
            else if ((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
            {
                pCiA402Axis->i16State = STATE_QUICK_STOP_ACTIVE;  // Transition 11
            } 
            else if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {

                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 8

            } 
            else if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
            {
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 9
            }
            break;
            
        case STATE_QUICK_STOP_ACTIVE:
            StatusWord |= STATUSWORD_STATE_QUICKSTOPACTIVE;        
            pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED;

            /*NOTE: it is not recommend to support transition 16 */
            break;
            
        case STATE_FAULT_REACTION_ACTIVE:
            StatusWord |= (STATUSWORD_STATE_FAULTREACTIONACTIVE);

            // Automatic transition
            pCiA402Axis->i16State = STATE_FAULT;// Transition 14
            break;
        
        case STATE_FAULT:
            StatusWord |= (STATUSWORD_STATE_FAULT);

            if ((ControlWord6040 & CONTROLWORD_COMMAND_FAULTRESET_MASK) == CONTROLWORD_COMMAND_FAULTRESET)
            {
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON;// Transition 15
                gParam[counter].ErrorCode0x603F = 0;
                ClearAlarm(&sAxis[counter]);
            }
            break;

        default:    //the sate variable is set to in invalid value => rest Axis
            StatusWord = (STATUSWORD_STATE_NOTREADYTOSWITCHON);
            pCiA402Axis->i16State = STATE_NOT_READY_TO_SWITCH_ON;
            break;

        }// switch(current state)

        /*Update operational functions (the low level power supply is always TRUE*/
        switch(pCiA402Axis->i16State)
        {
        case STATE_NOT_READY_TO_SWITCH_ON:
        case STATE_SWITCH_ON_DISABLED:
        case STATE_READY_TO_SWITCH_ON:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        case STATE_SWITCHED_ON:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        case STATE_OPERATION_ENABLED:
        case STATE_QUICK_STOP_ACTIVE:
        case STATE_FAULT_REACTION_ACTIVE:
            pCiA402Axis->bBrakeApplied = FALSE;
            pCiA402Axis->bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->bAxisFunctionEnabled = TRUE;
            pCiA402Axis->bConfigurationAllowed = FALSE;
            break;
        case STATE_FAULT:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        default:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  FALSE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        }

        if(    pCiA402Axis->bHighLevelPowerApplied == TRUE)
            StatusWord |= STATUSWORD_VOLTAGE_ENABLED;
        else
            StatusWord &= ~STATUSWORD_VOLTAGE_ENABLED;

        /*state transition finished set controlword complete bit and update status object 0x6041*/
        gParam[counter].StatusWord0x6041 = (StatusWord | STATUSWORD_REMOTE);
    }
        
}

/***********************************************************************
 * DESCRIPTION: this function is called if an error was detected
 *             
 * RETURNS:
 *
***********************************************************************/
void CiA402_LocalError(UINT16 NodeID, UINT16 ErrorCode)
{   
    UINT16 PartnerID = NodeID?0:1;
    //if(ErrorCode & 0x3FFF)
    if((ErrorCode & 0xBFFF) && (sAxis[NodeID].sAlarm.ErrReg.bit.StutterStop == 0) && (sAxis[NodeID].sAlarm.ErrReg.bit.TactSwitchSet == 0))
    {
        sAxis[PartnerID].sAlarm.ErrReg.bit.PartnerErr = 1; 
    }
    
    if( (LocalAxes[NodeID].i16State == STATE_FAULT_REACTION_ACTIVE)
            || (LocalAxes[NodeID].i16State == STATE_FAULT) || gParam[NodeID].ErrorCode0x603F == ErrorCode)
    {
    	return;
    }

    LocalAxes[NodeID].i16State = STATE_FAULT_REACTION_ACTIVE;
    gParam[NodeID].ErrorCode0x603F= ErrorCode;
    
    sErrLog.SaveErrLogEn =1;
}

/***********************************************************************
 * DESCRIPTION: this function is called if scram status is cleared
 *             
 * RETURNS:
 *
***********************************************************************/
void ClearScramStatus(UINT16 ScramID)
{
	LocalAxes[ScramID].i16State = STATE_READY_TO_SWITCH_ON;// Transition 15
    gParam[ScramID].ErrorCode0x603F = 0;
	gParam[ScramID].ControlWord0x6040 = 6;
}
/***********************************************************************
 * DESCRIPTION:  
 *             
 * RETURNS:
 *
***********************************************************************/
UINT32 SlowDownCnt[2];
void CiA402_MotionControl(void)
{
    TCiA402Axis *pCiA402Axis;
    struct McRefStruct  McRef;
    UINT16 counter = 0;
    PRIVATE UINT16 PowerEnState[2];

    
    static UINT32 ProfileMoveCnt = 0;
    if(gParam[0].CooperativeEnable0x3000)
    {
        gParam[0].ControlWord0x6040 = gParam[0].CooperativeCW0x3001;
        gParam[1].ControlWord0x6040 = gParam[0].CooperativeCW0x3001;
        
        gParam[0].TargetVelocity0x60FF = gParam[0].CO_ProfileVelocity0x3002;
        gParam[1].TargetVelocity0x60FF = gParam[0].CO_ProfileVelocity0x3002;
        
        if(0 != gParam[0].CO_ProfileVelocity0x3002)
        {
            ProfileMoveCnt++; // for 1ms increment
            if(ProfileMoveCnt > 2*gParam[0].CO_ProfileTime0x3003)
            {
                gParam[0].CO_ProfileVelocity0x3002 = 0;
                ProfileMoveCnt = 0;
            }
        }
        else
        {
            ProfileMoveCnt = 0;
        }
    }
    else
    {
        ProfileMoveCnt = 0;
    }

    for(counter = 0; counter < 2;counter++)
    {
        pCiA402Axis = &LocalAxes[counter];
        McRef.PowerOn = PowerEnState[counter];
        
        if(pCiA402Axis->bAxisFunctionEnabled)
        {
            McRef.PowerOn = POWER_ON;
        }
        else
        {       
            /*增加一个静态变量记录当前的PowerEn状态，在下次切入该函数时，再利用该PowerEn状态作为初始化的值
                之前没有这个问题的原因是：PowerEn的状态完全由当前的bAxisFunctionEnabled的状态决定，
                修改后，如果不记录上一次的PowerEn状态，会导致前一个轴的PowerEn状态会影响到当前轴bAxisFunctionEnabled从1切换为0时的PowerEn状态
                比如当我触发CAN断线故障时（基于右轴单独闭环，左右开环），左轴的McRef.PowerOn会有当bAxisFunctionEnabled从1切换为0时（也就是此时bAxisFunctionEnabled=0时）
                右轴会直接把上一次for循环的McRef.PowerEn=0当做自己的开闭环状态,而不会进入先减速再开环这段操作,所以只需要记录右轴上一次进入该函数的状态并作为下一次进入该函数的初始值即解决该问题
            */         
            //if ErrReg(CanBreak CanLost Bit) is set, slow down
            if (sAxis[counter].sAlarm.ErrReg.all & 0x3000)
            {
                if (POWER_ON == McRef.PowerOn)
                {
                    gParam[counter].TargetVelocity0x60FF = 0;
                    /*341 Inc/s = 5rpm*/
                    if (gParam[counter].ActualVelocity0x606C < 341 || 
                        gParam[counter].ActualVelocity0x606C > -341)
                    {
                        SlowDownCnt[counter]++;
                    }
                    else
                    {
                        SlowDownCnt[counter] = 0;
                    }
                    
                    if (SlowDownCnt[counter] >= 5*1500)
                    {
                        McRef.PowerOn = POWER_OFF;
                        SlowDownCnt[counter] = 0;
                    }
                }
            }
            else
            {
                McRef.PowerOn = POWER_OFF;
            }
        }
        
        PowerEnState[counter] = McRef.PowerOn;
        
        switch(gParam[counter].OperationModeDisplay0x6061)
        {
            case PROFILE_POSITION_MODE:
                McRef.ModeRef = POS_CTRL;
                McRef.PosRef = gParam[counter].TargetPosition0x607A;
                break;
            
            case PROFILE_VELOCITY_MOCE:
                McRef.ModeRef = SPD_CTRL;
                McRef.SpdRef = (float)gParam[counter].TargetVelocity0x60FF*60.0f/((float)gParam[counter].EncoderPPR0x2202);
                break;
            
            case PROFILE_TORQUE_MODE:
                McRef.ModeRef = CUR_CTRL;
                McRef.CurRef = (float)gParam[counter].TargetCurrent0x6071*0.001f;
                break;
            
            case INNER_VELOCITY_MODE:
                McRef.ModeRef = INNER_SPD_CTRL;
                McRef.SpdRef = 0.0f;
                break;
            
            case INNER_TORQUE_MODE:
                McRef.ModeRef = INNER_CUR_CTRL;
                McRef.CurRef = 0.0f;
                break;
                        
            default:
                break;
        }     

        MC_SetReference(&McRef, counter);
        
        if(!pCiA402Axis->bAxisFunctionEnabled)
        {
            /*Accept new mode of operation*/
            gParam[counter].OperationModeDisplay0x6061 = gParam[counter].OperationMode0x6060;
        }
    }
}



/** @} */

