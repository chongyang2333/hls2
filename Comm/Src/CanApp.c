/*******************************************************************
 *
 * FILE NAME:  CanApp.c
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
#include <string.h>
#include "gd32f4xx.h"
#include "CanApp.h"
#include "Param.h"
#include "Rgb.h"
#include "BootloaderInfo.h"
#include "gpio.h"
#include "HardApi.h"

#include "LedDriver.h"
#include "MachineAdditionalInfo.h"
#include "gd_hal.h"
#include "delay.h"

#define IMITATE_RESPONSE_MCU3_ENABLE   (1)

#define CAN_SLAVE_ID  0x0D

extern struct AxisCtrlStruct sAxis[MAX_AXIS_NUM];
extern const struct ParameterStruct gDefaultParam_Left[10];

struct CanAppStruct  sMyCan={0};
    
// extern CAN_HandleTypeDef hcan1;
// CAN_TxHeaderTypeDef   Can_TxHeader;
// CAN_RxHeaderTypeDef   Can_RxHeader;

extern ST_VersionStruct NowSoftWareVersion;


can_receive_message_struct receive_message;
can_trasnmit_message_struct transmit_message;

/*------------------------- Protype Functions --------------------------*/

PRIVATE void PcRequestHandle(UINT8 *pData);
PRIVATE void CanSendSoftwareVersion(void);
PRIVATE void CanSendMachineInfo(UINT8 *pData);
PRIVATE void CanSendMachineInfoWriteStatus(void);
PRIVATE void CanSendMachineInfoCrcState(void);

PRIVATE void can_tx(UINT8 *pData);
PRIVATE void can_tx_machineinfo(UINT8 *pData);
PRIVATE void CanSetTimeStamp(UINT8 *pData);
PRIVATE void CanSetMotion(UINT8 *pData);
PRIVATE void CanModifyMachineInfo(const UINT8 *pData);
void StorageMessage(CAN_RX_Message* pstCanRxMessage,BootLoaderInfo* pstBootLoaderInfo);
PRIVATE void CanLedExec(void);
PRIVATE void can_tx_no_block(UINT8 *pData);
PRIVATE UINT8 GetSelfId(void);

PRIVATE UINT8 BCC_CheckSum(const UINT8 *buf,UINT8 len);
PRIVATE void CanSendData(UINT8 txd[8], UINT8 len);
extern BootLoaderInfo bootloaderInfo;
UINT8 g_CanTxEnable = 0;

PRIVATE void CMDGetSoftWareVersionTreatmentFromRam(CAN_RX_Message* CanRxMessage,ST_VersionStruct* st_ram_version);
PRIVATE void CMDGetSoftWareVersionTreatmentFromBootloaderInfo(CAN_RX_Message* CanRxMessage,BootLoaderInfo* pst_bootLoaderInfo);
PRIVATE void CarpetModeSet(UINT8 *pData);
PRIVATE void CanSendAcc(UINT8 AccType);
PRIVATE void CanSetAcc(UINT8 *pData);

PRIVATE void CAN_GetRxMessage(CAN_RX_Message* CanRxMessage);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanAppInit(void)
{
    UINT16 Tmp = gParam[0].CanLostErrTime0x200A*4/100;
    
    if(Tmp<4) Tmp =0;
    
    sMyCan.CanLostMaxNum = Tmp;
    sMyCan.CanRxTxState = 0;
}



/***********************************************************************
 * DESCRIPTION:  40HZ
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanAppExec(void)
{
//    if(sMyCan.PcInitDone && sMyCan.PcCloseLoopEn)
    if(sMyCan.PcCloseLoopEn)
    {
        sMyCan.CanLostCnt++;
    }
    else
    {
        sMyCan.CanLostCnt = 0;
    }
    
    if(sMyCan.CanLostCnt > sMyCan.CanLostMaxNum && sMyCan.CanLostMaxNum !=0)  
    {
        sMyCan.CanLostErr = 1;
    }
    
    CanLedExec();
}

void chassis_led_ctrl( uint8_t *buff );

PRIVATE void CAN_GetRxMessage(CAN_RX_Message* CanRxMessage)
{
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &receive_message);
    /* check the receive message */
    can_message_receive(CAN0, CAN_FIFO1, &receive_message);
    
    if ((receive_message.rx_ff   == CAN_FF_STANDARD) &&
        (receive_message.rx_dlen == 8))
    {
        memcpy(CanRxMessage->RxData, receive_message.rx_data, 8);
    }
    else
    {
        memset(CanRxMessage->RxData, 0x00, 8);
    }

}

PRIVATE void CAN_GetRxMessage_Weak(CAN_RX_Message CanRxMessage)
{
//	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can_RxHeader, &CanRxMessage.RxData[0]);//数组参数传递？
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
const unsigned char ucF_IAPorApp = EN_RUN_IN_APP;
PUBLIC void CanAppDispatch(void)
{
    UINT8  CmdType;
		
	CAN_RX_Message CanRxMessage;
    CAN_GetRxMessage(&CanRxMessage);

	CmdType = CanRxMessage.RxData[0];

//    if (!sMyCan.PcInitDone)  
//        return;	

    switch(CmdType)
    {
        case 0x00 :
            PcRequestHandle(CanRxMessage.RxData);
        break;
        
        case 0x08 :
            RgbSetMode(CanRxMessage.RxData[1]);
        break;
				
        case EN_IAP_CMD:
        {
            IAPCmdTreatment(&CanRxMessage);
        }
        break;
				
        case 0x3F :
            CanSetTimeStamp(CanRxMessage.RxData);
        break;
        
        case 0x40 :
						if(!(sAxis[0].sAlarm.ErrReg.all || sAxis[1].sAlarm.ErrReg.all))
						{
							CanSetMotion(CanRxMessage.RxData);
						}
                        else
                        {
                            sMyCan.PcCloseLoopEn = 0;
                        }                        
						sMyCan.CanLostCnt = 0;
        break;
        
        case 0x41 : // clear error
            if(CanRxMessage.RxData[1] == 0x01)
            {
                gParam[0].ControlWord0x6040 = 0x86;
                gParam[1].ControlWord0x6040 = 0x86;
            }
        break;
            
        case 0x45 : // Set Carpet Mode
            CarpetModeSet(CanRxMessage.RxData);
        break;      

        case 0x48: //Set Acc Data
            CanSetAcc(CanRxMessage.RxData);
        break;
        
        case 0x50 :
            if((CanRxMessage.RxData[0]==0x50) && (CanRxMessage.RxData[1]==0x7A) && (CanRxMessage.RxData[2]==0x8A) \
            && (CanRxMessage.RxData[3]==0x85) && (CanRxMessage.RxData[4]==0x75) && (CanRxMessage.RxData[5]==0x5A) \
            && (CanRxMessage.RxData[6]==CAN_SLAVE_ID|| CanRxMessage.RxData[6]==0X02)) 
            {
                gParam[0].SaveParameter0x2401 = 1;
                EnableMachineAddInfoSave();
            }            
        break;
        
        case 0x51 :
            if(CanRxMessage.RxData[6]==CAN_SLAVE_ID || CanRxMessage.RxData[6]==0X02)
            {
                CanModifyMachineInfo(CanRxMessage.RxData);
            }
            break;
            
        case 0x5F:
            if(CanRxMessage.RxData[6]==CAN_SLAVE_ID|| CanRxMessage.RxData[6]==0X02)
            {
				RTC_BKP_Write(EN_RESET_TYPE_BKP_ADDR,EN_RESET_TYPE_SOFT);
                HAL_NVIC_SystemReset();
            }
        break;
						
        case 0x75:
					LedFsmEventHandle(&sLedFsm, LED_EVENT_REMOTE_CONTROL, (LedStateEnum)CanRxMessage.RxData[1], NULL);
					CanSendLedStateFb(sLedFsm.curState);
        break;
        
        case 0x90:
            if( CanRxMessage.RxData[1] == 13 || CanRxMessage.RxData[1] == 14 )
            {
                chassis_led_ctrl( CanRxMessage.RxData );
            }
            break;
				
        case 0x77:	
            LdsPowerCtrl(CanRxMessage.RxData[1], CanRxMessage.RxData[2]);
            CanSendLidarStateFb(ReadLidarPowerState());					
            break;	
        
        case 0x83:	
            DisinfectionModulePowerCtrl(CanRxMessage.RxData[1]);
            CanSendDisinfectionModuleStateFb(CanRxMessage.RxData[1]);					
            break;
        
        default:
            break;
    }

}

void IAPCmdTreatment(CAN_RX_Message* pstCanRxMessage)
{
    UINT8 ucCanCmdType,ucCanCmdId;   
    ucCanCmdType = pstCanRxMessage->RxData[2];   
    ucCanCmdId = pstCanRxMessage->RxData[1];   
    if(ucCanCmdId != GetSelfId())
    {
        //do nothing
    }
    else
    {    
	  switch(ucCanCmdType)
		{
			case CmdSoftWareReset:
			{
				bootloaderInfo.F_Reset = 1;
				StorageMessage(pstCanRxMessage,&bootloaderInfo);
			}
			break;
		
			case CmdCheckIapOrApp:
			{			
				CMDCheckIapOrAppTreatment(pstCanRxMessage);
			}
			break;
			
			case CmdGetSoftWareVersion:
			{
				CMDGetSoftWareVersionTreatmentFromRam(pstCanRxMessage,&NowSoftWareVersion);//
			}
			break;
			
			case CmdOpenOrCloseCan:
			{
				CMDOpenOrCloseCanTreatment(pstCanRxMessage);
			}
			break;
		
			case CmdJumpApp:
			{
				//CMDJumpAppTreatment(pstCanRxMessage);//app程序不处理
			}			
			break;
			
			case CmdReadFlash:
			{
				CmdReadFlashTreatment(pstCanRxMessage);
			}
			break;
		
		}
    }
}


UINT8 CompareCheckSum(UINT8 data1,UINT8 data2)
{
#ifdef	EN_NO_CHECK_SUM
	return 1;
#else
	if(data1 == data2)
	{
		return 1;
	}
	else
	{
		return 0;
	}
#endif
}

UINT8 CompareCRC(UINT8 data1,UINT8 data2)
{
#ifdef	EN_NO_CHECK_CRC
	return 1;
#else
	if(data1 == data2)
	{
		return 1;
	}
	else
	{
		return 0;
	}
#endif
}

void StorageMessage(CAN_RX_Message* pstCanRxMessage,BootLoaderInfo* pstBootLoaderInfo)
{
	UINT8 i;
	for(i=0;i<8;i++)
	{
		pstBootLoaderInfo->CANMessage[i] = pstCanRxMessage->RxData[i];
	}
}
void CMDCheckIapOrAppTreatment(CAN_RX_Message* CanRxMessage)
{
	UINT8 CheckSumCalc = GetCheckSum8(&CanRxMessage->RxData[0],7);
	if( CompareCheckSum(CheckSumCalc,CanRxMessage->RxData[7]))
	{
		
		CanRxMessage->RxData[3] = ucF_IAPorApp;
		CMDResponseRegular(CanRxMessage);
				
	}
}


				
void CMDSoftWareTreatment(CAN_RX_Message* CanRxMessage)
{
	UINT8 CheckSumCalc = GetCheckSum8(&CanRxMessage->RxData[0],7);	
	if( CompareCheckSum(CheckSumCalc,CanRxMessage->RxData[7]))
	{
        extern void EnableFirmwareUpdate( void );
        
		RTC_BKP_Write(EN_RESET_TYPE_BKP_ADDR,EN_RESET_TYPE_SOFT);
		CMDResponseRegular(CanRxMessage);
		g_CanTxEnable = 0;
		delay_ms(10);
		DisableInterupt();
        EnableFirmwareUpdate();
	}
}



PRIVATE void CMDGetSoftWareVersionTreatmentFromRam(CAN_RX_Message* CanRxMessage,ST_VersionStruct* st_ram_version)
{
		UINT8 CheckSumCalc = GetCheckSum8(&CanRxMessage->RxData[0],7);
		if( CompareCheckSum(CheckSumCalc,CanRxMessage->RxData[7]))
		{
			CanRxMessage->RxData[3] = st_ram_version->majorVer;
			CanRxMessage->RxData[4] = st_ram_version->minorVer;
			CanRxMessage->RxData[5] = st_ram_version->patchVer;
			CMDResponseRegular(CanRxMessage);
		}
}

PRIVATE void CMDGetSoftWareVersionTreatmentFromBootloaderInfo(CAN_RX_Message* CanRxMessage,BootLoaderInfo* pst_bootLoaderInfo)
{
		UINT8 CheckSumCalc = GetCheckSum8(&CanRxMessage->RxData[0],7);
		if( CompareCheckSum(CheckSumCalc,CanRxMessage->RxData[7]))
		{
			CanRxMessage->RxData[3] = pst_bootLoaderInfo->SoftwareVersion.majorVer;
			CanRxMessage->RxData[4] = pst_bootLoaderInfo->SoftwareVersion.minorVer;
			CanRxMessage->RxData[5] = pst_bootLoaderInfo->SoftwareVersion.patchVer;
			CMDResponseRegular(CanRxMessage);
		}
}

void CMDOpenOrCloseCanTreatment(CAN_RX_Message* CanRxMessage)
{
//	UINT8 CheckSumCalc = GetCheckSum8(&CanRxMessage->RxData[0],7);
//	if( CompareCheckSum(CheckSumCalc,CanRxMessage->RxData[7]))
//	{
		CMDResponseRegular(CanRxMessage);
		if(CanRxMessage->RxData[3])
		{
			g_CanTxEnable = 1;
		}
		else
		{
			g_CanTxEnable = 0;
		}
//	}
}


UINT16 GetCRC_16_Cal(UINT8 *data,UINT8 num)//八位数组，个数
{
		UINT8 i,j,con1,con2;
		UINT16 CrcR=0xffff, con3=0x00;
		for(i=0;i<num;i++)
		{
				//把第一个8位二进制数据（既通讯信息帧的第一个字节）与16位的CRC寄存器的低
				//8位相异或，把结果放于CRC寄存器，高八位数据不变；
				con1=CrcR&0xff;
				con3=CrcR&0xff00;
				CrcR=con3+data[i]^con1;
				//把CRC寄存器的内容右移一位（朝低位）用0填补最高位，并检查右移后的移出位；
				for(j=0;j<8;j++)
				{
					con2=CrcR&0x0001;
					CrcR=CrcR>>1;
					if(con2==1)
					{
						CrcR=CrcR^0xA001;
					}
				}
		}
		con1=CrcR>>8;//高字节
		con2=CrcR&0xff;//低字节
		CrcR=con2;
		CrcR=(CrcR<<8)+con1;
		return CrcR;
}

void CmdReadFlashTreatment(CAN_RX_Message* CanRxMessage)
{
	UINT32 addr = 0;
	unsigned int CRC_CalResult;
	unsigned char pdata[10];
	UINT8 CheckSumCalc = GetCheckSum8(&CanRxMessage->RxData[0],7);
	if( CompareCheckSum(CheckSumCalc,CanRxMessage->RxData[7]))
	{
		addr = (CanRxMessage->RxData[3]<<24) + (CanRxMessage->RxData[4]<<16)
					+ (CanRxMessage->RxData[5]<<8) + CanRxMessage->RxData[6];
		CanRxMessage->RxData[3] = *(__IO uint32_t*)(addr);
		CanRxMessage->RxData[4] = *(__IO uint32_t*)(addr+1);
		CanRxMessage->RxData[5] = *(__IO uint32_t*)(addr+2);
		
		pdata[0] = (UINT8)(addr >> 24);
		pdata[1] = (UINT8)(addr >> 16);
		pdata[2] = (UINT8)(addr >> 8);
		pdata[3] = (UINT8)(addr &0x00ff);
		pdata[4] = CanRxMessage->RxData[0];
		pdata[5] = CanRxMessage->RxData[1];
		pdata[6] = CanRxMessage->RxData[2];
		pdata[7] = CanRxMessage->RxData[3];
		pdata[8] = CanRxMessage->RxData[4];
		pdata[9] = CanRxMessage->RxData[5];
		CRC_CalResult = GetCRC_16_Cal(&pdata[0],10);
		CanRxMessage->RxData[6] = CRC_CalResult>>8;
		CanRxMessage->RxData[7] = (CRC_CalResult&0x00ff);
		can_tx(&CanRxMessage->RxData[0]);
	}
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void JumpAppFb(UINT8 RstType)
{
	UINT8 data[8];
    data[0] = 0x16;
    data[1] = CAN_SLAVE_ID;
    data[2] = 0x0D;
    data[3] = RstType;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = ~0x16;
	data[7] = GetCheckSum8(&data[0], 7);
    
	can_tx_no_block(&data[0]);
}


UINT8 GetCheckSum8(UINT8* pData,UINT8 len)
{
	UINT8 i;
	UINT8 sum = 0;
	for(i=0;i<len;i++)
	{
		sum += *pData;
		pData++;
	}
	return sum;
}

void CMDResponseRegular(CAN_RX_Message* pCanRxMessage)
{
	UINT8 len = 8;
	pCanRxMessage->RxData[7] = GetCheckSum8(&pCanRxMessage->RxData[0],len-1);
	can_tx_no_block(&pCanRxMessage->RxData[0]);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CarpetModeSet(UINT8 *pData)
{
    if (0x01 == pData[1])
    {
				if(gMachineInfo.motorVersion == 4)
				{
						gParam[0].MotorRatedCurrent0x2209 = 4000;
						gParam[1].MotorRatedCurrent0x2209 = 4000;
				}
				else
				{
					  gParam[0].MotorRatedCurrent0x2209 = 6000;
					  gParam[1].MotorRatedCurrent0x2209 = 6000;
				}
        gParam[0].SaveParameter0x2401 = 1;        
    }
    else if (0 == pData[1])
    {
        gParam[0].MotorRatedCurrent0x2209 = 3000;
        gParam[1].MotorRatedCurrent0x2209 = 3000;
        gParam[0].SaveParameter0x2401 = 1;
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanCarpetModeFdb(UINT16 MotorRatedCurrent0x2209)
{
    UINT8 txd[8];
    INT8 CarpetMode = -1;

    if (3000 == MotorRatedCurrent0x2209)
    {
        CarpetMode = 0;
    }
		else
		{
				if((gMachineInfo.motorVersion == 4)&&(4000 == MotorRatedCurrent0x2209))
				{
						CarpetMode = 1;
				}
				else if(6000 == MotorRatedCurrent0x2209)
				{
						CarpetMode = 1;
				}		
		}
    
    txd[0] = 0x46;
    txd[1] = CarpetMode;
    txd[2] = 0;
    txd[3] = 0;
    txd[4] = 0;
    txd[5] = 0;
    txd[6] = 0;
    txd[7] = BCC_CheckSum(txd,7);
    can_tx(txd);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendSpdFdb(INT16 LeftSpdInc, INT16 RightSpdInc)
{
    static UINT8 _40hzCnt = 0;
    UINT8 datasend[8];
    
    _40hzCnt++;
        
    datasend[0] = 0x01;  //表示电机速度数据上传
    datasend[1] = (LeftSpdInc>>8)&0XFF;
    datasend[2] = (LeftSpdInc)&0XFF;
    datasend[3] = (RightSpdInc>>8)&0XFF;
    datasend[4] = (RightSpdInc)&0XFF;
    //datasend[5] = 0x00;
	datasend[5] = sMyCan.CanLostCnt;
    datasend[6] = _40hzCnt;   //序号
    datasend[7] = BCC_CheckSum(datasend,7);   //校验码
    can_tx(datasend);		   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendErrorCode(UINT16 LeftErr, UINT16 RightErr)
{
    PRIVATE UINT8 ErrCodeCnt=0;
    UINT8 datasend[8];
    
    /*反馈电机fault*/
    datasend[0] = 0x43;
    datasend[1] = (LeftErr>>8)&0XFF;
    datasend[2] = (LeftErr)&0XFF;
    datasend[3] = (RightErr>>8)&0XFF;
    datasend[4] = (RightErr)&0XFF;
    datasend[5] = 0x00;
    datasend[6] = ErrCodeCnt++;   //序号
    datasend[7] = BCC_CheckSum(datasend,7);   //校验码
    can_tx(datasend);	   
}

PUBLIC void CanSendErrorCodeHigh(UINT16 LeftErr, UINT16 RightErr)
{
    PRIVATE UINT8 ErrCodeCnt=0;
    UINT8 datasend[8];
    
    /*反馈电机fault*/
    datasend[0] = 0x47;
    datasend[1] = (LeftErr>>8)&0XFF;
    datasend[2] = (LeftErr)&0XFF;
    datasend[3] = (RightErr>>8)&0XFF;
    datasend[4] = (RightErr)&0XFF;
    datasend[5] = 0x00;
    datasend[6] = ErrCodeCnt++;   //序号
    datasend[7] = BCC_CheckSum(datasend,7);   //校验码
    can_tx(datasend);	   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendLog1Left(INT16 LeftSpdRef, INT16 LeftSpdFdb, INT16 LeftPwmRef, UINT8 LeftStatus)
{
    UINT8 datasend[8];

    datasend[0] = 0x60;
    datasend[1] = (LeftSpdRef>>8) & 0XFF;
    datasend[2] = LeftSpdRef & 0XFF;
    datasend[3] = (LeftSpdFdb>>8) & 0XFF;
    datasend[4] =  LeftSpdFdb & 0XFF;
    datasend[5] = (LeftPwmRef>>8) & 0XFF;
    datasend[6] = LeftPwmRef & 0XFF;
    datasend[7] = LeftStatus;
    can_tx(datasend);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendLog1Right(INT16 RightSpdRef, INT16 RightSpdFdb, INT16 RightPwmRef, UINT8 RightStatus)
{
    UINT8 datasend[8];

    datasend[0] = 0x61;
    datasend[1] = (RightSpdRef>>8) & 0XFF;
    datasend[2] = RightSpdRef & 0XFF;
    datasend[3] = (RightSpdFdb>>8) & 0XFF;
    datasend[4] =  RightSpdFdb & 0XFF;
    datasend[5] = (RightPwmRef>>8) & 0XFF;
    datasend[6] = RightPwmRef & 0XFF;
    datasend[7] = RightStatus;
    can_tx(datasend);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendLog2Left(UINT16 LeftBusCurrent, INT16 LeftIq, INT16 LeftMosTemp, INT16 LeftMotorTemp)
{
    UINT8 datasend[8];

    datasend[0] = 0x64;
    datasend[1] = (LeftBusCurrent>>8) & 0XFF;
    datasend[2] = LeftBusCurrent & 0XFF;
    datasend[3] = (LeftIq>>8) & 0XFF;
    datasend[4] =  LeftIq & 0XFF;
    
    if (LeftMosTemp > 125)
    {
        LeftMosTemp = 125;
    }
    
    if (LeftMotorTemp > 125)
    {
        LeftMotorTemp = 125;
    }
    
    datasend[5] = LeftMosTemp;
    datasend[6] = LeftMotorTemp;
    datasend[7] = 0x00;
    can_tx(datasend);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendSupplyChargeVI_Info(UINT16 ChargeVoltage, INT16 ChargeCur, UINT16 supplyCur)
{
    UINT8 datasend[8];

    datasend[0] = 0x74;
    
    ChargeVoltage = ChargeVoltage/10; 
    datasend[1] = ChargeVoltage>>8;
    datasend[2] = ChargeVoltage;
    
    if(ChargeCur <= 0)
    {
        ChargeCur = 0;
    }
    
    ChargeCur = ChargeCur/10;
    datasend[3] = ChargeCur>>8;
    datasend[4] = ChargeCur;
    
    supplyCur = supplyCur/10;
    datasend[5] = supplyCur>>6;
    datasend[6] = supplyCur;
    
    datasend[7] = BCC_CheckSum(datasend, 7);
    can_tx(datasend); 
}

/***********************************************************************
 * DESCRIPTION:发送电池当前的温度，和充电满所需要的时间。temperature——温度（单位1度）；所需时间（单位秒）。
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendBatteryChargeExInfo(UINT16 temperature)
{
    UINT8 datasend[8] = {0};

    datasend[0] = 0x81;
    datasend[1] = temperature>>8;
    datasend[2] = temperature;
    datasend[3] = 0;
    datasend[4] = 0;
    datasend[5] = 0;
    datasend[7] = BCC_CheckSum(datasend, 7);
    can_tx(datasend);    
}

PUBLIC void CanSendBatteryChargeInfo(UINT8 ChargerState, UINT8 batteryLevelRaw, UINT8 batteryLevelOptimized, UINT16 batteryVoltage)
{
    UINT8 datasend[8] = {0};
    UINT8 batteryLevelState = 0;
    
    datasend[0] = 0x73;
    datasend[1] = ChargerState;

    if (batteryLevelOptimized < 2)
    {
        batteryLevelState = 1;
    }
    datasend[2] = batteryLevelState;
    datasend[3] = batteryLevelRaw;
    datasend[4] = batteryLevelOptimized;
    datasend[5] = 100;
    datasend[6] = batteryVoltage/1000;
    datasend[7] = BCC_CheckSum(datasend, 7);
    can_tx(datasend);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendLog2Right(UINT16 RightBusCurrent, INT16 RightIq, INT16 RightMosTemp, INT16 RightMotorTemp)
{
    UINT8 datasend[8];

    datasend[0] = 0x65;
    datasend[1] = (RightBusCurrent>>8) & 0XFF;
    datasend[2] = RightBusCurrent & 0XFF;
    datasend[3] = (RightIq>>8) & 0XFF;
    datasend[4] =  RightIq & 0XFF;
    
    if (RightMosTemp > 125)
    {
        RightMosTemp = 125;
    }
    
    if (RightMotorTemp > 125)
    {
        RightMotorTemp = 125;
    }
   
    datasend[5] = RightMosTemp;
    datasend[6] = RightMotorTemp;
    datasend[7] = 0x00;
    can_tx(datasend);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendGyro(INT16 *gyro, INT16 *accel)
{
    UINT8 datasend[8]={0};
    
    datasend[0] = 0x10;
    datasend[1] = ((gyro[0])>>8)&0XFF;
    datasend[2] = ((gyro[0]))&0XFF;
    datasend[3] = ((gyro[1])>>8)&0XFF;
    datasend[4] = ((gyro[1]))&0XFF;
    datasend[5] = ((gyro[2])>>8)&0XFF;
    datasend[6] = ((gyro[2]))&0XFF;
    datasend[7] = BCC_CheckSum(datasend,7);
    can_tx(datasend);

    datasend[0] = 0x11;
    datasend[1] = ((accel[0])>>8)&0XFF;
    datasend[2] = ((accel[0]))&0XFF;
    datasend[3] = ((accel[1])>>8)&0XFF;
    datasend[4] = ((accel[1]))&0XFF;
    datasend[5] = ((accel[2])>>8)&0XFF;
    datasend[6] = ((accel[2]))&0XFF;
    datasend[7] = BCC_CheckSum(datasend,7);
//    can_tx(datasend);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendInfraRed(UINT8 *pData)
{
    UINT8 datasend[8]={0};
     
    datasend[0] = 0x06;
    datasend[1] = 255 - pData[0];   
    datasend[2] = 255 - pData[1];  
    datasend[3] = 255 - pData[2]; 
    datasend[4] = 255 - pData[3];   
    datasend[5] = 0;
    datasend[6] = 0;
    datasend[7] = BCC_CheckSum(datasend,7);	
    
    can_tx(datasend);

}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendLedStateFb(UINT8 ledState)
{

    UINT8 datasend[8]={0};
     
    datasend[0] = 0x76;
    datasend[1] = ledState;   
    datasend[2] = 0;  
    datasend[3] = 0; 
    datasend[4] = 0;   
    datasend[5] = 0;
    datasend[6] = 0;
    datasend[7] = BCC_CheckSum(datasend,7);	
    
    can_tx(datasend);	

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendLidarStateFb(UINT8 lidarState)
{
    UINT8 datasend[8]={0};
     
    datasend[0] = 0x78;
    datasend[1] = lidarState;   
    datasend[2] = 0;  
    datasend[3] = 0; 
    datasend[4] = 0;   
    datasend[5] = 0;
    datasend[6] = 0;
    datasend[7] = BCC_CheckSum(datasend,7);	
    
    can_tx(datasend);		
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendDisinfectionModuleStateFb(UINT8 EnState)
{
    UINT8 datasend[8]={0};
     
    datasend[0] = 0x84;
    datasend[1] = EnState;   
    datasend[2] = 0;  
    datasend[3] = 0; 
    datasend[4] = 0;   
    datasend[5] = 0;
    datasend[6] = 0;
    datasend[7] = BCC_CheckSum(datasend,7);	
    
    can_tx(datasend);		
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void CanSendBatteryInfo(UINT8 index, UINT32 data)
{
    UINT8 datasend[8]={0};
     
    datasend[0] = 0x82;
    datasend[1] = index;   
    datasend[2] = data>>24;  
    datasend[3] = data>>16; 
    datasend[4] = data>>8;   
    datasend[5] = data;
    datasend[6] = 0;
    datasend[7] = BCC_CheckSum(datasend, 7);	
    
    can_tx(datasend);		
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PcRequestHandle(UINT8 *pData)
{
    switch(pData[1])
    {
        case 0x13:
            CanSendSoftwareVersion();
        break;
        
        case 0x46:
            CanCarpetModeFdb(gParam[0].MotorRatedCurrent0x2209);
        break;
        
        case 0x49:
            CanSendAcc(pData[2]);
        break;        
        
        case 0x52:
            if(pData[6]==CAN_SLAVE_ID)
            {
                CanSendMachineInfoCrcState();
            }                
        break;
                
        case 0x53:
            if(pData[6]==CAN_SLAVE_ID || pData[6]== 2)
            {
                CanSendMachineInfo(pData);
            }
        break;    

        case 0x54:
            if(pData[6]==CAN_SLAVE_ID)
            {
                CanSendMachineInfoWriteStatus();
            }
        break;
            
            
        default:            
            break;         
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSendSoftwareVersion(void)
{
    UINT8 datasend[8]={0};
    
    datasend[0] = 0x13;
    datasend[1] = NowSoftWareVersion.majorVer;
    datasend[2] = NowSoftWareVersion.minorVer;
    datasend[3] = NowSoftWareVersion.patchVer;
    datasend[4] = 0x00;
    datasend[5] = 0x00;
    datasend[6] = CAN_SLAVE_ID;
    datasend[7] = BCC_CheckSum(datasend,7);	
    can_tx(datasend);	

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSendMachineInfo(UINT8 *pData)
{
    UINT8  datasend[8]={0};
    UINT32 tmp_32bit_data;

    tmp_32bit_data = *((uint32_t *)(&gMachineInfo) + pData[2]);
    
    datasend[0] = 0x53;
    datasend[1] = pData[2];
    datasend[2] = tmp_32bit_data >> 24;
    datasend[3] = tmp_32bit_data >> 16; 
    datasend[4] = tmp_32bit_data >> 8;
    datasend[5] = tmp_32bit_data;
    datasend[6] = 2;
    datasend[7] = BCC_CheckSum(datasend,7);

    can_tx_machineinfo(datasend);
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSendMachineInfoWriteStatus(void)
{
    UINT8 datasend[8];

    datasend[0] = 0x54;
    datasend[1] = gMachineInfo.MachineInfoSaveState;
    datasend[2] = 0x00;
    datasend[3] = 0x00; 
    datasend[4] = 0x00;
    datasend[5] = 0x00;
    datasend[6] = 2;
    datasend[7] = BCC_CheckSum(datasend,7);
    can_tx_machineinfo(datasend);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSendMachineInfoCrcState(void)
{
    UINT8 datasend[8];

    datasend[0] = 0x52;
    datasend[1] = gMachineInfo.CrcState;
    datasend[2] = 0x00;
    datasend[3] = 0x00; 
    datasend[4] = 0x00;
    datasend[5] = 0x00;
    datasend[6] = 2;
    datasend[7] = BCC_CheckSum(datasend,7);
    can_tx_machineinfo(datasend);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void can_tx(UINT8 *pData)
{
    if (!ReadPadPowerState() || sMyCan.CanBreakErr)
    {
        return;
    }

    if (!g_CanTxEnable)
    {
        return;
    }

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
    transmit_message.tx_sfid = CAN_SLAVE_ID;
    transmit_message.tx_ft   = CAN_FT_DATA;
    transmit_message.tx_ff   = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    memcpy(transmit_message.tx_data, pData, 8);

    /* prepare to transmit */
    UINT32 StartTime = ReadTimeStampTimer();
    while (can_message_transmit(CAN0, &transmit_message) == CAN_NOMAILBOX)
    {
        if ((ReadTimeStampTimer() - StartTime) > 27*3000)  // 3ms
        {
            sMyCan.CanBreakErr = 1;
            break;
        }
    }
    
    sMyCan.CanRxTxState |= 0x01;
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void can_tx_machineinfo(UINT8 *pData)
{
    if (!ReadPadPowerState() || sMyCan.CanBreakErr)
    {
        return;
    }

    if (!g_CanTxEnable)
    {
        return;
    }

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
    transmit_message.tx_sfid = 2;
    transmit_message.tx_ft   = CAN_FT_DATA;
    transmit_message.tx_ff   = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    memcpy(transmit_message.tx_data, pData, 8);

    /* prepare to transmit */
    UINT32 StartTime = ReadTimeStampTimer();
    while (can_message_transmit(CAN0, &transmit_message) == CAN_NOMAILBOX)
    {
        if ((ReadTimeStampTimer() - StartTime) > 27*3000)  // 3ms
        {
            sMyCan.CanBreakErr = 1;
            break;
        }
    }
    
    sMyCan.CanRxTxState |= 0x01;
}

PRIVATE void can_tx_weak(UINT8 *pData)
{
//    uint32_t TxMailbox=0;
//    CAN_TxHeaderTypeDef *ptx=&Can_TxHeader;
//	
//    if(!ReadPadPowerState() || sMyCan.CanBreakErr)
//    {
//        return;
//    }
//    
//    if (!g_CanTxEnable)
//    {
//            return;
//    }
//    
//    ptx->StdId = CAN_SLAVE_ID;
//    ptx->RTR = CAN_RTR_DATA;
//    ptx->IDE = CAN_ID_STD;
//    ptx->DLC = 8;
//     
//    UINT32 StartTime = ReadTimeStampTimer();
//    /* Wait transmission complete */
//    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) 
//    {
//        if((ReadTimeStampTimer() - StartTime) > 27*3000)  // 3ms
//        {
//            sMyCan.CanBreakErr = 1;
//            break;
//        }
//    }
//    
//    sMyCan.CanRxTxState |= 0x01;
//      
//    HAL_CAN_AddTxMessage(&hcan1, &Can_TxHeader, pData, &TxMailbox);

}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/

PRIVATE void can_tx_no_block(UINT8 *pData)
{
    if (!ReadPadPowerState() || sMyCan.CanBreakErr) 
    {
        return;
    }

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
    transmit_message.tx_sfid = CAN_SLAVE_ID;
    transmit_message.tx_ft   = CAN_FT_DATA;
    transmit_message.tx_ff   = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    memcpy(transmit_message.tx_data, pData, 8);

    /* prepare to transmit */
    UINT32 StartTime = ReadTimeStampTimer();
    while (can_message_transmit(CAN0, &transmit_message) == CAN_NOMAILBOX)
    {
        if ((ReadTimeStampTimer() - StartTime) > 27*3000)  // 3ms
        {
            sMyCan.CanBreakErr = 1;
            break;
        }
    }
    
    sMyCan.CanRxTxState |= 0x01;
}

PRIVATE void can_tx_no_block_weak(UINT8 *pData)
{
//    uint32_t TxMailbox=0;
//    CAN_TxHeaderTypeDef *ptx=&Can_TxHeader;
//	
//    if(!ReadPadPowerState() || sMyCan.CanBreakErr)
//    {
//        return;
//    }

//    ptx->StdId = CAN_SLAVE_ID;
//    ptx->RTR = CAN_RTR_DATA;
//    ptx->IDE = CAN_ID_STD;
//    ptx->DLC = 8;
//     
//    UINT32 StartTime = ReadTimeStampTimer();
//    /* Wait transmission complete */
//    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) 
//    {
//        if((ReadTimeStampTimer() - StartTime) > 27*3000)  // 3ms
//        {
//            sMyCan.CanBreakErr = 1;
//            break;
//        }
//    }
//    
//    sMyCan.CanRxTxState |= 0x01;
//      
//    HAL_CAN_AddTxMessage(&hcan1, &Can_TxHeader, pData, &TxMailbox);

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSetTimeStamp(UINT8 *pData)
{
    UINT32  value = (pData[1]<<24) + (pData[2]<<16) + (pData[3]<<8) + pData[4];
    gParam[0].TimeStamp0x2500 = value;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSetMotion(UINT8 *pData)
{
    volatile INT16  TmpInt  = 0;
    volatile REAL32 TmpFloat= 0.0f;
    
    TmpInt = (pData[1] << 8) |(pData[2]); 
    TmpFloat = (float)TmpInt*0.0001f;                         // m/s
    TmpFloat = TmpFloat/gMachineInfo.machineWheelPerimeter;   // r/s
    gParam[0].TargetVelocity0x60FF = TmpFloat*(float)gParam[0].EncoderPPR0x2202;   // pulse/s
                         
    TmpInt = (pData[3] << 8) |(pData[4]); 
    TmpFloat = (float)TmpInt*0.0001f;                         // m/s
    TmpFloat = TmpFloat/gMachineInfo.machineWheelPerimeter;   // r/s
    gParam[1].TargetVelocity0x60FF = TmpFloat*(float)gParam[1].EncoderPPR0x2202;   // pulse/s
    
    if(pData[5] & 0x40)
    {
        sMyCan.PcCloseLoopEn = 1;
        gParam[0].ControlWord0x6040 = 0xF;
        gParam[1].ControlWord0x6040 = 0xF;
    }
    else
    {
        sMyCan.PcCloseLoopEn = 0;
        gParam[0].ControlWord0x6040 = 0x6;
        gParam[1].ControlWord0x6040 = 0x6;   
    }
    
    if (pData[5] & 0x80)
    {
        gParam[0].QuickStopEn0x6086 = 1;
        gParam[1].QuickStopEn0x6086 = 1;
    }
    else
    {
        gParam[0].QuickStopEn0x6086 = 0;
        gParam[1].QuickStopEn0x6086 = 0;    
    }
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSetAcc(UINT8 *pData)
{
    volatile UINT32 TmpLong  = 0;
    volatile REAL32 TmpFloat= 0.0f;
    
    TmpLong = (pData[2] << 24) |(pData[3] << 16) |(pData[4] << 8) |(pData[5]); 
    TmpFloat = (float)TmpLong*0.0001f;                         // m/s^2
    TmpFloat = TmpFloat/gMachineInfo.machineWheelPerimeter;   // r/s^2
    TmpFloat = TmpFloat*(float)gParam[0].EncoderPPR0x2202;   // pulse/s^2
    
    //Limit
    if (TmpFloat > gDefaultParam_Left[gMachineInfo.motorVersion].QuickStopDec0x6085*1.6)
    {
        TmpFloat = gDefaultParam_Left[gMachineInfo.motorVersion].QuickStopDec0x6085*1.6;
    }
    
    if (TmpFloat < gDefaultParam_Left[gMachineInfo.motorVersion].QuickStopDec0x6085/20.0f)
    {
        TmpFloat = gDefaultParam_Left[gMachineInfo.motorVersion].QuickStopDec0x6085/20.0f;
    }
    
    switch(pData[1])
    {
        case 0x01:
            gParam[0].ProfileAcc0x6083 = gParam[1].ProfileAcc0x6083 = TmpFloat;
            break;
        
        case 0x02:
            gParam[0].ProfileDec0x6084 = gParam[1].ProfileDec0x6084 = TmpFloat;
            break;
        
        case 0x03:
            gParam[0].QuickStopDec0x6085 = gParam[1].QuickStopDec0x6085 = TmpFloat;
            break;
                
        default:
            break;
    }
    
    CanSendAcc(pData[1]);
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
/*Read Only: Odometer/CumulativeTime */
const UINT8 MachineInfoBanIndex[2] = {25, 26};
PRIVATE void CanModifyMachineInfo(const UINT8 *pData)
{
    UINT8 	key = pData[1];
    UINT8 i;
    for(i=0; i<sizeof(MachineInfoBanIndex); i++)
    {
        if (key == MachineInfoBanIndex[i])
        {
            return;
        }
    }
    
    UINT32  value = (pData[2]<<24) + (pData[3]<<16) + (pData[4]<<8) + pData[5];

    *((uint32_t*)&gMachineInfo + key) = value;    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT8 BCC_CheckSum(const UINT8 *buf,UINT8 len)
{
	UINT8 i;
	UINT8 checksum = 0;
	for(i=0; i<len; i++)
	{
		checksum ^= *buf++;
	}
	return checksum;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanLedExec(void)
{
//    if(!sMyCan.PcInitDone)
//    {
//        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//        return;
//    }
    
    /*Can Rx & Tx State Instruction*/
    static UINT8 schTickCnt = 0;
    #define MAX_SCH_NUM 25
    
    if (MAX_SCH_NUM <= schTickCnt)
    {
        schTickCnt = 0;
        sMyCan.CanRxTxState &= 0xFC;
    }
    
    switch (sMyCan.CanRxTxState & 0x03)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            break;
        case 1:
            if (schTickCnt < 4)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            }
            break;
        /*rx failed, rx Finished*/
        case 2: 
            if (schTickCnt%MAX_SCH_NUM < (MAX_SCH_NUM/2))
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            }
            else 
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            }
            break;            
        /*rx Finished, rx Finished*/
        case 3:
            if ((schTickCnt < 4) || ((schTickCnt > 7) && (schTickCnt < 12)))
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            }
            break;
        
        default:
            break;             
    }
    
    schTickCnt++;
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSendData(UINT8 txd[8], UINT8 len)
{	
//			CAN_TxHeaderTypeDef   TxHeader;
//			UINT32                TxMailbox;
//			UINT8                 waitEmptyMailboxTimeoutCnt = 0;
//		
//			TxHeader.StdId = CAN_SLAVE_ID;
//			TxHeader.RTR = CAN_RTR_DATA;
//			TxHeader.IDE = CAN_ID_STD;
//			TxHeader.DLC = len;
//			TxHeader.TransmitGlobalTime = DISABLE;
	
			if(g_CanTxEnable)//有可能被命令关闭了CAN发送
			{
					/*if waitEmptyMailboxTimeoutCnt >= 50, Should Be Alarm*/
//					while((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) && (waitEmptyMailboxTimeoutCnt < 30))
//					{
//							waitEmptyMailboxTimeoutCnt++;
//							delay_us(100);
//					}
//					
//					if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txd, (unsigned int* )&TxMailbox) != HAL_OK)
//					{
//							/* Transmission request Error */
//							//_Error_Handler(__FILE__, __LINE__);
//					}
			}
			
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 GetSelfId(void)
{
  return EN_SELF_ID;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void CanSendAcc(UINT8 AccType)
{
    volatile UINT32 TmpLong  = 0;
    volatile REAL32 TmpFloat  = 0;
    
    switch(AccType)
    {
        case 0x01:
            TmpFloat = gParam[0].ProfileAcc0x6083;
            break;
        
        case 0x02:
            TmpFloat = gParam[0].ProfileDec0x6084;
            break;
        
        case 0x03:
            TmpFloat = gParam[0].QuickStopDec0x6085;
            break;
        
        default:
            TmpFloat = 0.0f;
            break;
    }
    
    TmpFloat = TmpFloat/(float)gParam[0].EncoderPPR0x2202; //pulse/s^2 -> r/s^2
    TmpFloat = TmpFloat*gMachineInfo.machineWheelPerimeter; //r/s^2 -> m/s^2
    TmpLong = TmpFloat * 10000; //m/s^2 -> 0.1mm/s^2
    
    UINT8 datasend[8];

    datasend[0] = 0x49;
    datasend[1] = AccType;
    datasend[2] = TmpLong >> 24;
    datasend[3] = TmpLong >> 16; 
    datasend[4] = TmpLong >> 8;
    datasend[5] = TmpLong;
    datasend[6] = 0x00;
    datasend[7] = BCC_CheckSum(datasend,7);
    can_tx(datasend);
}
