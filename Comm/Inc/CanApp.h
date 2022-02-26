/*******************************************************************
 *
 * FILE NAME:  CanApp.h
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

#ifndef _CAN_APP_H_
#define _CAN_APP_H_

#include "UserDataTypes.h"
#include "BootloaderInfo.h"

#define USE_CAN_APP

#define  STD_CAN_ID  0x02

struct CanAppStruct
{
    UINT8   PcInitDone;
    UINT8   CanLostErr;    
    UINT16  CanLostCnt;
    UINT8   CanBreakErr;
    UINT8   PcCloseLoopEn;
    UINT16  CanLostMaxNum;
    UINT8   CanRxTxState;  //bit0: Tx State(Set Valid), bit1: Rx State(Set Valid
		
		UINT8   Magic_Enable;
		UINT16  MagicThreshold_left;
		UINT16  MagicThreshold_Right;
	
};

typedef enum EN_CAN_CMD
{
	CmdSoftWareReset = 1,
	CmdCheckIapOrApp = 2,
	CmdGetSoftWareVersion = 3,
	CmdOpenOrCloseCan = 4,
	CmdUpdateSize = 5,
	CmdBlockSize = 6,
	CmdUpdateDate = 7,
	CmdJumpApp = 9,
	CmdReadFlash = 10	
}EN_CAN_CMD;

typedef enum EN_RUN_STATE
{
	EN_RUN_IN_APP,
	EN_RUN_IN_IAP
}EN_RUN_STATE;


typedef struct CAN_RX_Message
{
	UINT8                 RxData[8];
}CAN_RX_Message;

extern PUBLIC void CanAppInit(void);
extern PUBLIC void CanAppExec(void);
extern PUBLIC void CanAppDispatch(void);
extern PUBLIC void USB2CAN_RecvDispatch(UINT8 *data,UINT16 datalen);

extern PUBLIC void CanSendSpdFdb(INT16 LeftSpdInc, INT16 RightSpdInc);
extern PUBLIC void CanSendErrorCode(UINT16 LeftErr, UINT16 RightErr);
extern PUBLIC void CanSendErrorCodeHigh(UINT16 LeftErr, UINT16 RightErr);

extern PUBLIC void CanSendLog1Left(INT16 LeftSpdRef, INT16 LeftSpdFdb, INT16 LeftPwmRef, UINT8 LeftStatus);
extern PUBLIC void CanSendLog1Right(INT16 RightSpdRef, INT16 RightSpdFdb, INT16 RightPwmRef, UINT8 RightStatus);

PUBLIC void CanSendLog2Left(UINT16 LeftBusCurrent, INT16 LeftIq, INT16 LeftMosTemp, INT16 LeftMotorTemp);
PUBLIC void CanSendLog2Right(UINT16 RightBusCurrent, INT16 RightIq, INT16 RightMosTemp, INT16 RightMotorTemp);

extern PUBLIC void CanSendGyro(INT16 *gyro, INT16 *accel);
extern PUBLIC void CanSendInfraRed(UINT8 *pData);
void IAPCmdTreatment(CAN_RX_Message* pstCanRxMessage);

void CMDSoftWareTreatment(CAN_RX_Message* CanRxMessage);
void CMDOpenOrCloseCanTreatment(CAN_RX_Message* CanRxMessage);
void CMDJumpAppTreatment(CAN_RX_Message* CanRxMessage);
void CMDCheckIapOrAppTreatment(CAN_RX_Message* CanRxMessage);
void CMDResponseRegular(CAN_RX_Message* pCanRxMessage);
void IAPCmdTreatment(CAN_RX_Message* pstCanRxMessage);
UINT8 GetCheckSum8(UINT8* pData,UINT8 len);
void CmdReadFlashTreatment(CAN_RX_Message* CanRxMessage);
void CMDGetSoftWareVersionTreatment(CAN_RX_Message* CanRxMessage,BootLoaderInfo* pst_bootLoaderInfo);
#define DisableInterupt() __set_FAULTMASK(1);//关闭异常
#define EnableInterrupt()  __set_FAULTMASK(0);
#define EN_IAP_CMD 0x16
#define EN_SELF_ID 0x0d
//#define EN_NO_CHECK_SUM 
//#define EN_NO_CHECK_CRC 

extern PUBLIC void CanSendMagXYZ(INT16 *P,UINT8 addr);

extern PUBLIC void CanSendLedStateFb(UINT8 ledState);
extern PUBLIC void CanSendLidarStateFb(UINT8 lidarState);
extern PUBLIC void CanSendDisinfectionModuleStateFb(UINT8 EnState);
PUBLIC void CanSendBatteryChargeInfo(UINT8 ChargerState, UINT8 batteryLevelRaw, UINT8 batteryLevelOptimized, UINT16 batteryVoltage);
PUBLIC void CanSendBatteryChargeExInfo(UINT16 temperature);
PUBLIC void CanSendSupplyChargeVI_Info(UINT16 ChargeVoltage, INT16 ChargeCur, UINT16 supplyCur);
PUBLIC void CanCarpetModeFdb(UINT16 MotorRatedCurrent0x2209);
PUBLIC void CanSendBatteryInfo(UINT8 index, UINT32 data);
PUBLIC void JumpAppFb(UINT8 RstType);

#endif  
