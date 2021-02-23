/*******************************************************************
 *
 * FILE NAME:  PowerManager.h
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2019.11.16
 *
 * AUTHOR:      DengZhuo  Tel:15008452001  Email:dengzhuo@pudutech.com
 *
 * History:   --DengZhuo  Tel:15008452001  Email:dengzhuo@pudutech.com
                    ->support 圭石 21.6Ah电池 20200218
------------------------------------------------------------------------
16-11-2019 Version 2.00 : Created by DengZhuo
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/
#include "PowerManager.h"
#include "HardApi.h"
#include "Param.h"
#include "gpio.h"
#include "i2c.h"
#include "battery_iic.h"
#include "delay.h"
#include "LedDriver.h"
#include "CanApp.h"
#include "string.h"
#include "math.h"
#include "stm32f7xx_hal.h"
#include "MachineAdditionalInfo.h"

#define ON 1
#define OFF 0

#define A2M7_LIDAR_DEFAULT_SPEED 136

#define VIRTUALFULLSOC     94    // 虚拟电量100% 对应的真实电量
#define VIRTUALZEROSOC     5     // 虚拟电量 0%  对应的真实电量
#define MAXCHARGEREALSOC   96    // 最大充电电量

#define FULLSOC_CHARGING_MAXTIME    36000 // 100MS周期 1*60*60*10 -> 1hour
    
BatteryIICStruct sBatI2C;

struct PowerManagerStruct sPowerManager = {0};

const struct BatteryObjectEntryStruct sBatteryOjectDic[] = {
{0x0000, 10, 16, &sPowerManager.sBatteryInfo[0].FullCapacity},
{0x0001, 10, 16, &sPowerManager.sBatteryInfo[0].RemaingCapacity},
{0x0002, 1, 16, &sPowerManager.sBatteryInfo[0].Voltage},
{0x0003, 10, 16, &sPowerManager.sBatteryInfo[0].SOH},
{0x0004, 10, 16, &sPowerManager.sBatteryInfo[0].CycleCnt},
{0x000B, 10, 16, &sPowerManager.sBatteryInfo[0].SN},
{0x000C, 10, 8, &sPowerManager.sBatteryInfo[0].BMS_icType},
{0x0040, 10, 16, &sPowerManager.sBatteryInfo[0].CellVoltage[0]},
{0x0041, 10, 16, &sPowerManager.sBatteryInfo[0].CellVoltage[1]},
{0x0042, 10, 16, &sPowerManager.sBatteryInfo[0].CellVoltage[2]},
{0x0043, 10, 16, &sPowerManager.sBatteryInfo[0].CellVoltage[3]},
{0x0044, 10, 16, &sPowerManager.sBatteryInfo[0].CellVoltage[4]},
{0x0045, 10, 16, &sPowerManager.sBatteryInfo[0].CellVoltage[5]},
{0x0046, 10, 16, &sPowerManager.sBatteryInfo[0].CellVoltage[6]},
{0x0080, 1, 8,  &sPowerManager.sBatteryInfo[0].SocOptimized},
{0x0081, 1, 8,  &sPowerManager.sBatteryInfo[0].SocRaw},
{0x0082, 1, 16, &sPowerManager.sBatteryInfo[0].ChargeCurrent},
{0x0083, 1, 16, &sPowerManager.sBatteryInfo[0].DischargeCurrent},
{0x0084, 1, 8, &sPowerManager.sBatteryInfo[0].ComFailSet},
{0x0085, 1, 8, &sPowerManager.sBatteryInfo[0].LowPowerSet},
{0x00C0, 1, 32, &sPowerManager.sAlarm.PowerAlarmReg.all},

{0x0100, 10, 16, &sPowerManager.sBatteryInfo[1].FullCapacity},
{0x0101, 10, 16, &sPowerManager.sBatteryInfo[1].RemaingCapacity},
{0x0102, 1, 16, &sPowerManager.sBatteryInfo[1].Voltage},
{0x0103, 10, 16, &sPowerManager.sBatteryInfo[1].SOH},
{0x0104, 10, 16, &sPowerManager.sBatteryInfo[1].CycleCnt},
{0x010B, 10, 16, &sPowerManager.sBatteryInfo[1].SN},
{0x010C, 10, 8, &sPowerManager.sBatteryInfo[1].BMS_icType},
{0x0140, 10, 16, &sPowerManager.sBatteryInfo[1].CellVoltage[0]},
{0x0141, 10, 16, &sPowerManager.sBatteryInfo[1].CellVoltage[1]},
{0x0142, 10, 16, &sPowerManager.sBatteryInfo[1].CellVoltage[2]},
{0x0143, 10, 16, &sPowerManager.sBatteryInfo[1].CellVoltage[3]},
{0x0144, 10, 16, &sPowerManager.sBatteryInfo[1].CellVoltage[4]},
{0x0145, 10, 16, &sPowerManager.sBatteryInfo[1].CellVoltage[5]},
{0x0146, 10, 16, &sPowerManager.sBatteryInfo[1].CellVoltage[6]},
{0x0180, 1, 8,  &sPowerManager.sBatteryInfo[1].SocOptimized},
{0x0181, 1, 8,  &sPowerManager.sBatteryInfo[1].SocRaw},
{0x0182, 1, 16, &sPowerManager.sBatteryInfo[1].ChargeCurrent},
{0x0183, 1, 16, &sPowerManager.sBatteryInfo[1].DischargeCurrent},
{0x0184, 1, 8, &sPowerManager.sBatteryInfo[1].ComFailSet},
{0x0185, 1, 8, &sPowerManager.sBatteryInfo[1].LowPowerSet}
};

PRIVATE void PowerInfoUpdate(void);
PRIVATE void ChargeCurrentBiasCal(void);
PRIVATE void RK3399HeartStateUpdate(void);
PRIVATE void BatteryCoverStateUpdate(void);
PRIVATE void ChargerPlugEventUpdate(struct ChargeManageStruct *chargeInfo);
PRIVATE void KeyEventUpdate(void);
PRIVATE void PowerAlarmExec(void);
PRIVATE void PM_PowerOnOffExec(void);
PRIVATE void ChargeOnOffExec(void);
PRIVATE void ClearPowerAlarmReg(void);
PRIVATE void SpeakerOnOffExec(void);
PRIVATE void ClearKeyEvent(void);
PRIVATE void ClearChargerPlugEvent(void);
PRIVATE void UpdateBatteryPcDisplay(void);

PRIVATE void UploadBatteryInfo(struct BatteryInfoStruct *pBattery);
PRIVATE BOOL ManufacturerBlockAccesRead(UINT8 SlaveAddress, UINT8 Cmd, UINT16 MAC_Cmd, UINT8 *Str, UINT16 Len);
PRIVATE BOOL Battery_Serial_Read(UINT8 SlaveAddress, UINT8 Readaddr, UINT8 *Str,UINT16 Len);
PRIVATE void ReadBatteryInfo(struct BatteryInfoStruct *pBattery);
PRIVATE BOOL BatteryReadSoc(UINT8 *pSoc, UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadTemp(INT16 *pTemp, UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadFcc(UINT32 *pFcc, UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadRc(UINT32 *pRc, UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadBv(UINT32 *pBv, UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadSoh(UINT32 *pSoh, UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadCc(UINT32 *pCc, UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadCellVoltage(UINT16  CellV[7], UINT8 Addr, UINT8 BatteryType);
PRIVATE BOOL BatteryReadCurrent(INT16 *pCurrent, UINT8 Addr, UINT8 BatteryType);

PRIVATE void DisinfectionModulePowerOnOffExec(void);

PRIVATE void VbusSoftStartBlock(UINT8 ApplicationMode);

extern PUBLIC UINT8 ApplicationMode;
extern void HAL_I2C_Reset(I2C_HandleTypeDef* i2cHandle);
PRIVATE struct BatteryObjectEntryStruct* OBJ_GetBatteryObjectHandle(UINT16 index );

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PowerManagerInit(UINT8 ApplicationMode)
{
    MX_I2C3_Init();
       		
	DisableCharge(ApplicationMode);
	LedPowerOn();
    LidarPowerOff(); 

    delay_us(300);
    ResetACS711();
    
    memset(&sPowerManager, 0, sizeof(struct PowerManagerStruct));
    
    VbusSoftStartBlock(ApplicationMode);

	//读取MCU复位原因，判断是硬件复位还是软件复位，当判断是软件复位时，即使判断为插入充电器开机，也不允许关闭头部电源
	sPowerManager.eSysRstState = GetResetType();
	//读取BKP寄存器中电源管理中的Alarm标志位
	sPowerManager.sAlarm.PowerAlarmReg.all = RTC_BKP_Read(EN_BATTERY_PROTECT_BKP_ADDR);

	sPowerManager.ePowerKeyStateLast = sPowerManager.ePowerKeyState = KEY_UP;
    
	sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.ChassisPower = ON;
	sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LedPower = ON;
	sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;
	sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = ON; 
	sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = OFF;
	sPowerManager.sBoardPowerInfo.PoweroffUploadAckFlag = 0;
    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower = OFF;

	sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.ChassisPower = ON;
	sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LedPower = ON;
	sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower = OFF;
	sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower = OFF; 
    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = OFF;
	sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWERON_INIT;
    
    sPowerManager.sBoardPowerInfo.ParallelVoltage = 0;

	sPowerManager.sAlarm.ChargeCurrentMax = 10000;
	sPowerManager.sAlarm.CellTempMax = 450;  //0.1℃
	sPowerManager.sAlarm.ChargerVoltageMax = 31000;
	sPowerManager.sAlarm.BatteryVoltageMax = 30000;
	sPowerManager.sAlarm.LowPowerSoc = 0;

	sPowerManager.sAlarm.ChargeCurrentOverCntMax = 10;
	sPowerManager.sAlarm.ChargerVolateOverCntMax = 10;
	sPowerManager.sAlarm.CellTempOverCntMax = 30;
	sPowerManager.sAlarm.ComFailCntMax = 10;
	sPowerManager.sAlarm.BatteryVolateOverCntMax = 20;
	sPowerManager.sAlarm.LowPowerCntMax = 80;


	sPowerManager.sChargeInfo.ChargerConnectedVoltageThreshold = 27000; //mV
	sPowerManager.sChargeInfo.ChargerConnectedCurrentThreshold = 300; //mA
	sPowerManager.sChargeInfo.ChargeAppState = NOT_CHARGED;
	sPowerManager.sChargeInfo.ChargerConnectJudgeTime = 15;
    sPowerManager.sChargeInfo.ChargerDisconnectJudgeTime = 10;
    sPowerManager.sChargeInfo.charge_over_cnt = 0;
    
    sPowerManager.DevSocOptimized = 0;
    
    for(UINT8 i; i<BATTERY_NUMBERS; i++)
    {
        sPowerManager.sBatteryInfo[i].BMS_icType = NONE_RECOGNIZED;
        sPowerManager.sBatteryInfo[i].FloorSocLimit = VIRTUALZEROSOC;
        sPowerManager.sBatteryInfo[i].TopSocLimit = VIRTUALFULLSOC;        // 虚拟电量100%对应的真实电量
        sPowerManager.sBatteryInfo[i].FullChargeFloorSoc = 94; // 暂不使用
        sPowerManager.sBatteryInfo[i].FullChargeTopSoc = MAXCHARGEREALSOC;   // 最大充电的真实电量
        sPowerManager.sBatteryInfo[i].LowPowerCnt = 0;
        sPowerManager.sBatteryInfo[i].LowPowerSet = 0;
        sPowerManager.sBatteryInfo[i].ComFailCnt = 0;
        sPowerManager.sBatteryInfo[i].ComFailSet = 0;
    }
}


/***********************************************************************
 * DESCRIPTION: 40HZ
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PowerManagerExec(void)
{
    sPowerManager.uTick++;
    
	PowerInfoUpdate();

	//如果powerManage处于Init状态，则先对参数进行校正，在执行真正的电源管理前，需要确保充电关闭，以及外设电源按照合理的状态打开或者关闭
	if(sPowerManager.ePMState == POWER_MANAGE_INIT)
	{
		ChargeCurrentBiasCal();

		//完成电流校正，即退出POWER_MANAGE_INIT状态，执行真正的电源管理
		if (sPowerManager.sChargeInfo.ChargeCurrentBiasCorrectedFlag == 1)
		{
			sPowerManager.ePMState = POWER_MANAGE_CHARGE_KEY_EVENT_DETECT;
		}
		else
		{
			return;
		}
	}
	
	switch(sPowerManager.ePMState)
	{
		case POWER_MANAGE_CHARGE_KEY_EVENT_DETECT:
			//更新Rk3399在线状态
			RK3399HeartStateUpdate();

			//更新电池盖板状态
			BatteryCoverStateUpdate();

			//更新充电器插拔事件:充电器插入 && 充电器拔出
			ChargerPlugEventUpdate(&sPowerManager.sChargeInfo);
        
            //更新按键事件:按键长按 && 按键短按
            KeyEventUpdate();        
			sPowerManager.ePMState = POWER_MANAGE_ALARM_DETECT;
			break;

		case POWER_MANAGE_ALARM_DETECT:
			//电源相关故障Alarm事件:充电器过压、电池过压、充电过流、通讯线脱落、电量与电压失配
			PowerAlarmExec();

			sPowerManager.ePMState = POWER_MANAGE_POWER_ON_OFF_CHARGE_EXEC;
			break;

		case POWER_MANAGE_POWER_ON_OFF_CHARGE_EXEC:
			//外设电源开关管理的状态机跳转
			PM_PowerOnOffExec();

			//充电开关状态机跳转		
			ChargeOnOffExec();

			sPowerManager.ePMState = POWER_MANAGE_STATE_CLEAR;
			break;

		case POWER_MANAGE_STATE_CLEAR:
            
			//清除充电器插入事件、按键等事件
			ClearKeyEvent();
			ClearChargerPlugEvent();

			sPowerManager.ePMState = POWER_MANAGE_CHARGE_KEY_EVENT_DETECT;
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
PUBLIC void BatteryParamLoop(void)
{
    UINT8 Tmp[11];
    
    const UINT8 GF_7S6P_DeviceName[6] = "PD7S6P";
    const UINT8 GF_7S8P_DeviceName[6] = "PD7S8P";
    
    for (UINT8 i = 0; i<BATTERY_NUMBERS; i++)
    {
        //识别多位置电池类型
        if (NONE_RECOGNIZED == sPowerManager.sBatteryInfo[i].BMS_icType)
        {       
            PRIVATE UINT32 readtime[BATTERY_NUMBERS] = {0};
            
            /*update per 1s if battery type has not been recognized*/
            if (sPowerManager.uTick - readtime[i] <= 40)
                return;
            readtime[i] = sPowerManager.uTick;
            
            /*GF 7S6P 7S8P baseAddr = 0x18----first addr:0x18, second addr:0x20, maybe 0x28 is the most suitable third addr for the next code*/
            #define GF_BASEADDR 0x18
            if (Battery_Serial_Read(GF_BASEADDR + 8*i, 0x6C, Tmp, 11))
            {
                sPowerManager.sBatteryInfo[i].id = i;
                sPowerManager.sBatteryInfo[i].Addr = GF_BASEADDR + 8*i;
//                sPowerManager.sBatteryInfo[i].FloorSocLimit = 5;
//                sPowerManager.sBatteryInfo[i].TopSocLimit = 100;
//                sPowerManager.sBatteryInfo[i].FullChargeFloorSoc = 95;
//                sPowerManager.sBatteryInfo[i].FullChargeTopSoc = 100;  
                
                if (!strncmp((INT8 *)&Tmp[1], (INT8 *)GF_7S6P_DeviceName, strlen((INT8 *)GF_7S6P_DeviceName)))
                {                
                    sPowerManager.sAlarm.ChargeCurrentMax = 5500;
                    sPowerManager.sBatteryInfo[i].BMS_icType = (enum BatteryManageSystemType)(GF_7S6P_0x18+32*i);
                    return;
                }
                
                if (!strncmp((INT8 *)&Tmp[1], (INT8 *)GF_7S8P_DeviceName, strlen((INT8 *)GF_7S8P_DeviceName)))
                { 
                    sPowerManager.sAlarm.ChargeCurrentMax = 10000;
                    sPowerManager.sBatteryInfo[i].BMS_icType = (enum BatteryManageSystemType)(GF_7S8P_0x18+32*i);
                    return;
                }
              
            }
        }
        else
        {
            ReadBatteryInfo(&sPowerManager.sBatteryInfo[i]);  
            UploadBatteryInfo(&sPowerManager.sBatteryInfo[i]);       
        }
    }
    
    UpdateBatteryPcDisplay();
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT8 ReadChargeAppState(void)
{
    if (!sPowerManager.sChargeInfo.ChargeAppState)
    {
        return 0;
    }

    return 1;
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void FrameHeader0xB0Parser(UINT8 data[8])
{
	switch(data[1])
	{
		case 2:
			sPowerManager.sBoardPowerInfo.PoweroffUploadAckFlag = 1;
			break;
		
		case 3:
			sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = ON;
			CanLowPowerConsumeMode(sPowerManager.sBoardPowerInfo.LowPowerConsumeMode);
			break;

		case 5:
			sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = OFF;
			CanLowPowerConsumeMode(sPowerManager.sBoardPowerInfo.LowPowerConsumeMode);
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
PRIVATE void PowerInfoUpdate(void)
{
    //这个IO设置为推挽输出，不知道所有机器能否完成读取IO操作
    sPowerManager.sChargeInfo.eChargeMosState = ReadChargeMosState();	
	sPowerManager.sChargeInfo.ChargeCurrent = 1000.0f * GetChargeCurrent() - sPowerManager.sChargeInfo.ChargeCurrentBias;
	sPowerManager.sChargeInfo.ChargeVoltage = 1000.0f * GetChargeVoltage();
    sPowerManager.sBoardPowerInfo.ParallelVoltage = 1000.0f * GetBatteryVoltage();

    for(UINT8 i=0; i<BATTERY_NUMBERS; i++)
    {
        //对电量进行虚拟化
        if(sPowerManager.sBatteryInfo[i].SocRaw <= sPowerManager.sBatteryInfo[i].FloorSocLimit)
        {
            sPowerManager.sBatteryInfo[i].SocOptimized = 0;
        }
        else if(sPowerManager.sBatteryInfo[i].SocRaw >= sPowerManager.sBatteryInfo[i].TopSocLimit)
        {
            sPowerManager.sBatteryInfo[i].SocOptimized = 100;
        }
        else
        {
            sPowerManager.sBatteryInfo[i].SocOptimized = (sPowerManager.sBatteryInfo[i].SocRaw - sPowerManager.sBatteryInfo[i].FloorSocLimit) * 100 / 
                                                            (  sPowerManager.sBatteryInfo[i].TopSocLimit 
                                                                - sPowerManager.sBatteryInfo[i].FloorSocLimit);
        }
    }

	//更新串口工具上的数据显示
    gSensorData.ParallelVoltage0x400A = sPowerManager.sBoardPowerInfo.ParallelVoltage;
	gSensorData.ChargeCurrent0x400C = sPowerManager.sChargeInfo.ChargeCurrent;
	gSensorData.ChargeVoltage0x400F = sPowerManager.sChargeInfo.ChargeVoltage;
	gSensorData.ChargeState0x400E = sPowerManager.sChargeInfo.eChargeMosState;
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ChargeCurrentBiasCal(void)
{
	#define CHARGE_CURRENT_CORRECT_LAUNCH_TIME 0
	#define CHARGE_CURRENT_CORRECT_END_TIME 20
	#define CHARGE_CURRENT_CORRECT_ELAPSED_TIME 20.0f

	static INT16 ChargeCurrentSum = 0;

	//MOS关闭1S后开始采集电流数据累积
	//在随后的250ms内获取到的电流平均值作为采样电流平均值
	//在获取充电电流采样偏置值后，不再重复进入采样电流偏置计算
	if (sPowerManager.uTick > CHARGE_CURRENT_CORRECT_END_TIME)
		return;

	if (sPowerManager.uTick < CHARGE_CURRENT_CORRECT_END_TIME)
	{
		ChargeCurrentSum += sPowerManager.sChargeInfo.ChargeCurrent;
	}
	else
	{
		sPowerManager.sChargeInfo.ChargeCurrentBias = ChargeCurrentSum/CHARGE_CURRENT_CORRECT_ELAPSED_TIME;
		sPowerManager.sChargeInfo.ChargeCurrentBiasCorrectedFlag = 1;
	}
}



/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void BatteryCoverStateUpdate(void)
{
	static UINT8 BatteryNotCoverCnt = 0;

	if (ReadBatteryCoverState())
	{
		BatteryNotCoverCnt++;
	}
	else
	{
		BatteryNotCoverCnt = 0;
		sPowerManager.eBatteryCoverState = COVERED;
	}

	if (BatteryNotCoverCnt >= 3)
	{
		sPowerManager.eBatteryCoverState = NOT_COVERED;
	}
	
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void RK3399HeartStateUpdate(void)
{
	static UINT8 Rk3399HeartOffLineCnt = 0;

	if (!ReadRk3399HeartState())
	{
		Rk3399HeartOffLineCnt++;
	}
	else
	{
		Rk3399HeartOffLineCnt = 0;
		sPowerManager.eRk3399HeartState = ONLINE;
	}

	if (Rk3399HeartOffLineCnt >= 3)
	{
		sPowerManager.eRk3399HeartState = OFFLINE;
	}
	
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
uint8_t test_charge = 0;
PRIVATE void ChargerPlugEventUpdate(struct ChargeManageStruct *chargeInfo)
{
    PRIVATE UINT8 ChargeVoltageDeltaReachCnt = 0;
    PRIVATE UINT8 ChargeVoltageReachCnt = 0;
	PRIVATE UINT8 ChargeVoltageUnreachCnt = 0;
	PRIVATE UINT8 ChargeCurrentUnreachCnt = 0;
    PRIVATE UINT16 LastChargeVoltage = 0;
    
    //更新充电口电压变化
    chargeInfo->ChargerVoltageDecDelta = chargeInfo->ChargeVoltage - LastChargeVoltage;
    LastChargeVoltage = chargeInfo->ChargeVoltage;
    
    // MOS管截止 && 充电口电压高于27V,同时充电口的电压不是一直在下降的，则判断为充电器插入事件
    if ((chargeInfo->ChargeVoltage > chargeInfo->ChargerConnectedVoltageThreshold) && (chargeInfo->eChargeMosState == MOS_OFF))
    {        
        ChargeVoltageReachCnt++;
        
        if (chargeInfo->ChargerVoltageDecDelta < 0)
        {
            ChargeVoltageDeltaReachCnt++;
        }
    }
    else
    {
        ChargeVoltageReachCnt = 0;
        ChargeVoltageDeltaReachCnt = 0;
    }


    if (ChargeVoltageReachCnt >= chargeInfo->ChargerConnectJudgeTime)
    {     
        if (ChargeVoltageDeltaReachCnt >= (chargeInfo->ChargerConnectJudgeTime - 2))
        {    
            //可能充电器的AC头被拔出来了
        }
        else
        {
            chargeInfo->eChargerEvent = INSERT_IN;
            test_charge = chargeInfo->eChargerEvent;
            chargeInfo->eChargerConnectState = CONNECT;    
            chargeInfo->charge_over_cnt = 0;            
        }
        
        ChargeVoltageReachCnt = 0;
        ChargeVoltageDeltaReachCnt = 0;
    }

	//MOS管导通 && 充电电流小于1A，则判断为发生充电器拔出事件,同时更新此时充电器没有连接
	if ((chargeInfo->ChargeCurrent < chargeInfo->ChargerConnectedCurrentThreshold) && (chargeInfo->eChargeMosState == MOS_ON))
	{
        if(ChargeCurrentUnreachCnt < chargeInfo->ChargerDisconnectJudgeTime )
        {
            ChargeCurrentUnreachCnt++;
        }
	}
	else
	{
		ChargeCurrentUnreachCnt = 0;
	}

    // 无充电电流， 判断充电器断开
	if (ChargeCurrentUnreachCnt >= chargeInfo->ChargerDisconnectJudgeTime)
	{
        if(chargeInfo->charge_over_cnt == 0 && is_charger_cur_over() )
        {

            VCS_711_reset();
            ChargeCurrentUnreachCnt = 0;
        } else {
            chargeInfo->eChargerEvent = PULL_OUT;
            test_charge = chargeInfo->eChargerEvent;
            chargeInfo->eChargerConnectState = DISCONNECT;
        }
        if( is_charger_cur_over() )
        {
            chargeInfo->charge_over_cnt++;
        }
	}
    

	//MOS管未导通 && 充电电压低于27V,则判断为充电器拔出事件,同时更新此时充电器没有连接
	if ((chargeInfo->ChargeVoltage < chargeInfo->ChargerConnectedVoltageThreshold) && (chargeInfo->eChargeMosState == MOS_OFF))
	{
		ChargeVoltageUnreachCnt++;
	}
	else
	{
		ChargeVoltageUnreachCnt = 0;
	}

	if (ChargeVoltageUnreachCnt >= chargeInfo->ChargerDisconnectJudgeTime)
	{
		chargeInfo->eChargerEvent = PULL_OUT;
		chargeInfo->eChargerConnectState = DISCONNECT;
        test_charge = chargeInfo->eChargerEvent;
	}
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
UINT8 powerKeyPressCnt = 0;
PRIVATE void KeyEventUpdate(void)
{
    //更新按键历史状态列表
	sPowerManager.ePowerKeyStateLast = sPowerManager.ePowerKeyState;
	sPowerManager.ePowerKeyState = ReadKeyInPinState();
    
	if ((sPowerManager.ePowerKeyStateLast == KEY_UP) && (sPowerManager.ePowerKeyState == KEY_DOWN))
	{
		sPowerManager.ePowerKeyEvent = SHORT_PRESS;
	}

	if (sPowerManager.ePowerKeyState == KEY_DOWN)
	{
		powerKeyPressCnt++;
	}
	else
	{
		powerKeyPressCnt = 0;
	}
    
    //在当前的按键事件判断中，在每次使用完LONG_PRESS这个事件后，就会把按键设置为NULL_KEY_EVENT，防止在按键释放时，此时的LONG_PRESS一直触发状态机循环
    //所以在触发LONG_PRESS事件并且清除后，当按键一直按下时到了10S，会触发LONGLONG_PRESS，并且在触发LONG_PRESS之前一直处于NULL_KEY_EVENT
    //2S
    if ((powerKeyPressCnt >= 20) && (sPowerManager.ePowerKeyEvent == SHORT_PRESS))
    {
        sPowerManager.ePowerKeyEvent = LONG_PRESS;
    }
    
    //10S
    if (powerKeyPressCnt >= 100)
    {
        sPowerManager.ePowerKeyEvent = LONGLONG_PRESS;
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ClearKeyEvent(void)
{
    //整个电源架构中，事件的清除机制是有问题，在此处利用特殊条件对事件进行清除，初步满足需求
    if(sPowerManager.ePowerKeyState == KEY_UP)
    {
        sPowerManager.ePowerKeyEvent = NULL_KEY_EVENT;
    }
    
    if(sPowerManager.ePowerKeyEvent == LONG_PRESS)
    {
        sPowerManager.ePowerKeyEvent = NULL_KEY_EVENT;
    }
    
    if(sPowerManager.ePowerKeyEvent == LONGLONG_PRESS)
    {
        sPowerManager.ePowerKeyEvent = NULL_KEY_EVENT;
    }    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
//持续检查电池容量跟电池电压的配对关系，则认为报告异常
PRIVATE void PowerAlarmExec(void)
{
	//打开MOS前检查充电器是否过压
	//当收到充电器插入事件的时候，开始进行充电器电压检查，1S后获取检查结果，如果没有过压，则检查结果为CHECK_APPROVED
	//当收到充电器拔出事件的时候，清空充电器电压检查结果，等下一次插入充电器时，重新进行一次检查
	if ((sPowerManager.sChargeInfo.eChargerEvent == INSERT_IN) && (sPowerManager.sChargeInfo.sChargerCheck.eCheckState == CHECK_APPENDING))
	{
		sPowerManager.sChargeInfo.sChargerCheck.eCheckState = CHECK_START;
		sPowerManager.sChargeInfo.sChargerCheck.CheckStartTime = sPowerManager.uTick;
	}
	else if (sPowerManager.sChargeInfo.eChargerEvent == PULL_OUT)
	{
		sPowerManager.sChargeInfo.sChargerCheck.eCheckState = CHECK_APPENDING;
        sPowerManager.sAlarm.ChargerVolateOverCnt = 0;		
	}

	//如果检查到1S内有125ms发现过压，则判定为过压
	if ((sPowerManager.uTick - sPowerManager.sChargeInfo.sChargerCheck.CheckStartTime <= 40) && (sPowerManager.sChargeInfo.sChargerCheck.eCheckState == CHECK_START))
	{
		if (sPowerManager.sChargeInfo.ChargeVoltage > sPowerManager.sAlarm.ChargerVoltageMax)
		{
			sPowerManager.sAlarm.ChargerVolateOverCnt++;
		}
		else
		{
			sPowerManager.sAlarm.ChargerVolateOverCnt = 0;
		}
        
        if (sPowerManager.uTick - sPowerManager.sChargeInfo.sChargerCheck.CheckStartTime == 40)
        {
            if (sPowerManager.sAlarm.ChargerVolateOverCnt > 5)
            {
                sPowerManager.sChargeInfo.sChargerCheck.eCheckState = CHECK_FAILED;
                sPowerManager.sAlarm.PowerAlarmReg.bit.ChargerVoltageOver = 1;
            }
            else
            {
                sPowerManager.sChargeInfo.sChargerCheck.eCheckState = CHECK_APPROVED;
            }	
        }
	}
    
    //检查是否存在过流充电
	if (sPowerManager.sChargeInfo.ChargeCurrent > sPowerManager.sAlarm.ChargeCurrentMax)
	{
		sPowerManager.sAlarm.ChargeCurrentOverCnt++;
	}
	else
	{
		sPowerManager.sAlarm.ChargeCurrentOverCnt = 0;
	}

	if (sPowerManager.sAlarm.ChargeCurrentOverCnt >= sPowerManager.sAlarm.ChargeCurrentOverCntMax)
	{
		sPowerManager.sAlarm.PowerAlarmReg.bit.ChargeCurrentOVer = 1;
		sPowerManager.sAlarm.ChargeCurrentOverCnt = sPowerManager.sAlarm.ChargeCurrentOverCntMax;
	}
    
    //检查电路板电压是否过高(实际上也是检查电池是否被过冲)是否被过充
    if (sPowerManager.sBoardPowerInfo.ParallelVoltage > sPowerManager.sAlarm.BatteryVoltageMax)
    {
        sPowerManager.sAlarm.BoardVoltageOverCnt++;
    }
    else if(sPowerManager.sAlarm.BoardVoltageOverCnt)
    {
        sPowerManager.sAlarm.BoardVoltageOverCnt--;
    }

    if (sPowerManager.sAlarm.BoardVoltageOverCnt >= sPowerManager.sAlarm.BatteryVolateOverCntMax)
    {
        sPowerManager.sAlarm.BoardVoltageOverCnt = sPowerManager.sAlarm.BatteryVolateOverCntMax;
        sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryVoltageOver = 1;
    }    
    
    sPowerManager.sAlarm.ComFailCombinedSet = 0;
    sPowerManager.sAlarm.LowPowerCombinedSet = 0;

    for(UINT8 i=0; i<BATTERY_NUMBERS; i++)
    {
        //检查电池是否被过充
//        if (sPowerManager.sBatteryInfo[i].Voltage > sPowerManager.sAlarm.BatteryVoltageMax)
//        {
//            sPowerManager.sAlarm.BatteryVolateOverCnt[i]++;
//            over_cnt++;
//        }
//        else if(sPowerManager.sAlarm.BatteryVolateOverCnt[i])
//        {
//            sPowerManager.sAlarm.BatteryVolateOverCnt[i]--;
//        }

        if (sPowerManager.sAlarm.BatteryVolateOverCnt[i] >= sPowerManager.sAlarm.BatteryVolateOverCntMax)
        {
            sPowerManager.sAlarm.BatteryVolateOverCnt[i] = sPowerManager.sAlarm.BatteryVolateOverCntMax;
            sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryVoltageOver = 1;
        }
        
        //检查电芯温度
//        if (sPowerManager.sBatteryInfo[i].CellTemp > sPowerManager.sAlarm.CellTempMax)
//        {
//            sPowerManager.sAlarm.CellTempOverCnt[i]++;
//        }
//        else if(sPowerManager.sAlarm.CellTempOverCnt[i])
//        {
//            sPowerManager.sAlarm.CellTempOverCnt[i]--;
//        }

        if (sPowerManager.sAlarm.CellTempOverCnt[i] >= sPowerManager.sAlarm.CellTempOverCntMax)
        {
            sPowerManager.sAlarm.PowerAlarmReg.bit.CellTempOver = 1;
            sPowerManager.sAlarm.CellTempOverCnt[i] = sPowerManager.sAlarm.CellTempOverCntMax;
        }
        
        //检查电池通讯是否脱落
        if (sPowerManager.sBatteryInfo[i].ComFailCnt >= sPowerManager.sAlarm.ComFailCntMax)
        {
            sPowerManager.sBatteryInfo[i].ComFailCnt = sPowerManager.sAlarm.ComFailCntMax;
            sPowerManager.sBatteryInfo[i].ComFailSet = 1;
        }
        else if(!sPowerManager.sBatteryInfo[i].ComFailCnt)
        {
            sPowerManager.sBatteryInfo[i].ComFailSet = 0;
        }
        sPowerManager.sAlarm.ComFailCombinedSet += sPowerManager.sBatteryInfo[i].ComFailSet;
        
        
        //检查电量是否过低
        //if ((sPowerManager.sBatteryInfo[i].SocOptimized <= sPowerManager.sAlarm.LowPowerSoc) && 
        if ((sPowerManager.DevSocOptimized <= sPowerManager.sAlarm.LowPowerSoc) && 
            (!sPowerManager.sBatteryInfo[i].ComFailSet) &&
            (NONE_RECOGNIZED != sPowerManager.sBatteryInfo[i].BMS_icType))
        {
            sPowerManager.sBatteryInfo[i].LowPowerCnt++;
        }
        else if(sPowerManager.sBatteryInfo[i].LowPowerCnt)
        {
            sPowerManager.sBatteryInfo[i].LowPowerCnt--;
        }
        
//        if( NONE_RECOGNIZED == sPowerManager.sBatteryInfo[i].BMS_icType
//            sPowerManager.sBatteryInfo[i].ComFailSet != ComFail
//        )
//        {
//            sPowerManager.sBatteryInfo[i]
//        }

        if (sPowerManager.sBatteryInfo[i].LowPowerCnt >= sPowerManager.sAlarm.LowPowerCntMax)
        {
            sPowerManager.sBatteryInfo[i].LowPowerSet = 1;
            sPowerManager.sBatteryInfo[i].LowPowerCnt = sPowerManager.sAlarm.LowPowerCntMax;
        }else if (!sPowerManager.sBatteryInfo[i].LowPowerCnt)
        {
            sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower = 0;
        }
        sPowerManager.sAlarm.LowPowerCombinedSet += sPowerManager.sBatteryInfo[i].LowPowerSet;
    }
    
    /*两块电池都通讯失败时,才判定为通讯失败*/
    if (sPowerManager.sAlarm.ComFailCombinedSet >= BATTERY_NUMBERS)
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.ComFail = 1;
    }
    else
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.ComFail = 0;        
    }
    
    /*两块电池都电量过低时,才判定为电量过低*/
    if (sPowerManager.sAlarm.LowPowerCombinedSet == BATTERY_NUMBERS)
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower = 1;
    }
    else
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower = 0;
    }
    
    
	//如果发现充电器被拔出，则清除相关的alarm标志位，同时清除充电器检查OK的标志位
	if(sPowerManager.sChargeInfo.eChargerEvent == PULL_OUT)
	{
		ClearPowerAlarmReg();
        sPowerManager.sChargeInfo.sChargerCheck.eCheckState = CHECK_APPENDING;
	}

	//需确认BKP寄存器本质是RAM? 无擦写寿命？
	RTC_BKP_Write(EN_BATTERY_PROTECT_BKP_ADDR, sPowerManager.sAlarm.PowerAlarmReg.all);
}




/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PM_PowerOnOffExec(void)
{
	switch(sPowerManager.sBoardPowerInfo.PowerOnOnffAppState)
	{
		case POWERON_INIT:
			//MCU被软件复位时，即使刚复位时检测到充电器插入，也不允许对头部平板进行关机
			if ((sPowerManager.sChargeInfo.ChargeVoltage > sPowerManager.sChargeInfo.ChargerConnectedVoltageThreshold) && (sPowerManager.eSysRstState != EN_RESET_TYPE_SOFT))
			{
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = ON;
                
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = PAD_POWEROFF;
			}
            //增加判断条件sPowerManager.ePowerKeyState == KEY_UP是为了防止按键启动的时候松动了一下开机按钮，导致按下刚开机就直接关机了
			else if(sPowerManager.ePowerKeyState == KEY_UP)
			{
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = ON;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = ON;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = ON;
                sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = OFF;

				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = NORMAL_POWERON;
			}

            PadPowerOnInit(sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower);	
            sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.PadPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower;
			break;

		case NORMAL_POWERON:      
			//在NORMAL_POWERON状态中允许使能音推、雷达等
			if (sPowerManager.ePowerKeyEvent == LONG_PRESS)
			{
                    if(sPowerManager.sChargeInfo.eChargerConnectState == CONNECT)
                    {
                        if (!(sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK))
                        {
                            sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF_UPLOAD;
                            sPowerManager.uPowerOffKeyPressTimestamp = sPowerManager.uTick;
                        }
                    }
                    else
                    {
                        sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF_UPLOAD;
                        sPowerManager.uPowerOffKeyPressTimestamp = sPowerManager.uTick;
                    }
			}

			//当在非充电状态中,低电量的时候跳转入低电量上报状态
			if (sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower)
			{
				if ((sPowerManager.sChargeInfo.eChargerConnectState != CONNECT) && (!sPowerManager.sAlarm.PowerAlarmReg.bit.ComFail))
				{
					sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF_UPLOAD;
                    sPowerManager.uPowerOffKeyPressTimestamp = sPowerManager.uTick;
				}
			}

			//如果发生打开电池仓盖的动作，无论在任何情况，则应该直接执行关闭整机电源
			if (sPowerManager.eBatteryCoverState == NOT_COVERED)
			{
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;                            
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = OFF;                
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                sPowerManager.BatteryCoverLeadtoPoweroffSet = 1;
                EnableMachineAddInfoSave();
			}
            
			break;

		case PAD_POWEROFF:
			//当检测故障(非低电量故障)时，应该进入正常POWERON状态，打开头部平板，方便上位机做声光提示
			if (sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK)
			{
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = ON;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = ON;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = ON;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = A2M7_LIDAR_DEFAULT_SPEED;
                sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = OFF;

				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = NORMAL_POWERON;
			}
			//由于插上充电器导致开机的情形下，当长按按钮时，应该跳转进入Normal_POWERON状态
			else if (sPowerManager.ePowerKeyEvent == LONG_PRESS)
			{
				//跳转入normalPowerOn模式时，应该首先默认打开speaker使能、Vbus电源、雷达电源
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = ON;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = ON;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = ON;
				sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = A2M7_LIDAR_DEFAULT_SPEED;
                sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = OFF;
				
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = NORMAL_POWERON;
			}
			//由于插上充电器导致开机的情形下，当拔出充电器时应该跳转进入关机状态
			else if(sPowerManager.sChargeInfo.eChargerEvent == PULL_OUT)
			{
				sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                EnableMachineAddInfoSave();
			}

			//如果发生打开电池仓盖的动作，无论在任何情况，则应该直接执行关闭整机电源
			if (sPowerManager.eBatteryCoverState == NOT_COVERED)
			{
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;                            
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = OFF;  
                sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = OFF;                  
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                sPowerManager.BatteryCoverLeadtoPoweroffSet = 1;
                EnableMachineAddInfoSave();
			}
			
			break;

		case POWEROFF_UPLOAD:
            {
                static UINT8 UploadPoweroffCnt = 0;
                static UINT32 uploadPoweroffTimestamp = 0;

                //每1S给上位机上传一次关闭平板提示
                if ((sPowerManager.uTick - uploadPoweroffTimestamp >= 40) && (sPowerManager.uTick - sPowerManager.uPowerOffKeyPressTimestamp <= 400))
                {
                    UploadPoweroffCnt++;
                    uploadPoweroffTimestamp = sPowerManager.uTick;
					sPowerManager.sBoardPowerInfo.PoweroffUploadAckFlag = 0;

                    if (sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower)
                    {
                        CanUploadLowPoweroffEvent();
                    }
                    else
                    {
                        CanUploadKeyPoweroffEvent();
                    }
                }
            }

			//如果收到应答，此时应该跳转入POWEROFF_ACKED等待上位机关机后执行关机操作
			if (sPowerManager.sBoardPowerInfo.PoweroffUploadAckFlag)
			{
				sPowerManager.sBoardPowerInfo.PoweroffUploadAckFlag = 0;
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF_UPLOAD_ACKED;
                sPowerManager.uPoweroffCmdFeedbackTimestamp = sPowerManager.uTick;
			}
			else
			{
				//上报即将关机信息后，10s内上位机仍然一直无应答，判定为上位机关机信息反馈超时
				if (sPowerManager.uTick - sPowerManager.uPowerOffKeyPressTimestamp > 400)
				{
					sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF_UPLOAD_ACK_TIMEOUT;
				}
			}
            
			//如果上位机反馈信息超时
			if ((sPowerManager.sBoardPowerInfo.PowerOnOnffAppState == POWEROFF_UPLOAD_ACK_TIMEOUT) && (!sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower))
			{
                sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = NORMAL_POWERON;
			}            
            
			//或者如果按键持续按下10S，执行强制关闭上位机或者关闭整机电源
            //或者如果此时低电量时同时上报消息无应答时，执行强制关闭上位机或者关闭整机电源
			if ((sPowerManager.ePowerKeyEvent == LONGLONG_PRESS) || ((sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower) && (sPowerManager.sBoardPowerInfo.PowerOnOnffAppState == POWEROFF_UPLOAD_ACK_TIMEOUT)))
			{
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;                            
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = OFF;  
                sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = ON;
                sPowerManager.uPoweroffCmdFeedbackTimestamp = sPowerManager.uTick;
                
				//如果此时充电器未连接，跳转进入POWEROFF状态
				if (sPowerManager.sChargeInfo.eChargerConnectState == DISCONNECT)
				{
                    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = OFF;
					sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                    EnableMachineAddInfoSave();
				}
				//如果此时充电器连接，跳转进入PAD_POWEROFF状态
				else
				{
					sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = PAD_POWEROFF;					
				}

			}

			//如果发生打开电池仓盖的动作，无论在任何情况，则应该直接执行关闭整机电源
			if (sPowerManager.eBatteryCoverState == NOT_COVERED)
			{
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;                            
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = OFF;    
                sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = OFF;                  
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                sPowerManager.BatteryCoverLeadtoPoweroffSet = 1;
                EnableMachineAddInfoSave();

			}
			
			break;   

		case POWEROFF_UPLOAD_ACKED:
        {
            UINT8 RK3399HeartDeadTimeOut = 0;
            
            //给了应答后3min仍然没有完成安卓关机，则判断为HeartDeadTimeout
            if ((sPowerManager.uTick - sPowerManager.uPoweroffCmdFeedbackTimestamp >= 3*60*40) && sPowerManager.eRk3399HeartState)
            {
                RK3399HeartDeadTimeOut = 1;
            }
        
			//读取上位机RK3399_HEART状态状态，连续读取到3次低电平时，则跳转入平板关机状态或者整机关机状态
			//或者如果按键持续按下10S，执行强制关闭上位机或者关闭整机电源
            //或者如果Rk3399 Heart Dead Timeout
			if (!sPowerManager.eRk3399HeartState || sPowerManager.ePowerKeyEvent == LONGLONG_PRESS || RK3399HeartDeadTimeOut)
			{
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;                            
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = OFF;  
                sPowerManager.sBoardPowerInfo.LowPowerConsumeMode = ON;
                
				//如果此时充电器未连接，跳转进入POWEROFF状态
				if (sPowerManager.sChargeInfo.eChargerConnectState == DISCONNECT)
				{
                    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = OFF;
					sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                    EnableMachineAddInfoSave();
				}
				//如果此时充电器连接，跳转进入PAD_POWEROFF状态
				else
				{
					sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = PAD_POWEROFF;					
				}

			}

			//如果发生打开电池仓盖的动作，无论在任何情况，则应该直接执行关闭整机电源
			if (sPowerManager.eBatteryCoverState == NOT_COVERED)
			{
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;                            
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;  
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = OFF;   
                sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = OFF;                  
				sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                sPowerManager.BatteryCoverLeadtoPoweroffSet = 1;
                EnableMachineAddInfoSave();
			}
			
			break; 
        }            

        case POWEROFF:
			//执行关机操作
            LedFsmEventHandle(&sLedFsm, LED_EVENT_MCU_POWEROFF, LED_STATE_CLOSE, NULL);
            if( IsMachineAddInfoSaveOK() )
			{
                McuPowerOff();
            }
        
            //当插着充电器进行更换电池重新盖上电池舱门时，则重新进行电源初始化
            //上述表述的条件是充电器仍然插着，电池舱门处于盖上状态，同时处于状态机的POWEROFF装填下
            //增加变量sPowerManager.BatteryCoverOpenSet 的原因用于修复在电池电压高于27情况下，关机情况下插入充电器，头部直接开机的问题
			if ((sPowerManager.eBatteryCoverState == COVERED) && 
                (sPowerManager.sChargeInfo.ChargeVoltage > sPowerManager.sChargeInfo.ChargerConnectedVoltageThreshold) && 
                 sPowerManager.BatteryCoverLeadtoPoweroffSet)
			{     
                //Reset
                HAL_NVIC_SystemReset();
			}
           
			break;

		default:
			break;
	}
    
    if (sPowerManager.sBoardPowerInfo.LowPowerConsumeMode)
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;                            
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower = OFF;
    }
    else
    {    
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = ON;                            
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = ON;         
    }

	//更新执行 雷达电源  音响电源  头部平板电源使能动作
	lidarPowerOnOffExec();
	SpeakerOnOffExec();	
    DisinfectionModulePowerOnOffExec();

	if (sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.PadPower != sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower)
	{
		sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.PadPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower;
		if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower)
		{
			PadPowerOn();
		}
		else
		{
			PadPowerOff();
		}
	}
    
    //更新Vbus状态以及使能禁能动作
    if (sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.VbusPower != sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower)
    {
        if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower)
        {
            __nop();
        }
        else
        {
            VbusDisable();
            sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.VbusPower = OFF;          
        }
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ChargeOnOffExec(void)
{
    PRIVATE uint32_t charge_timeout = 0; // 防止电量无法达到期望值导致一直充电的问题
    
	switch(sPowerManager.sChargeInfo.ChargeAppState)
	{
		case NOT_CHARGED:
			if (sPowerManager.eBatteryCoverState == NOT_COVERED)
			{
				break;
			}

			if (sPowerManager.sChargeInfo.eChargerEvent == INSERT_IN)
			{
				sPowerManager.sChargeInfo.ChargeAppState = CHECK_BEFORE_CHARGING;
			}
            else if(sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK)
            {
                sPowerManager.sChargeInfo.ChargeAppState = POWER_ALARM;
            }
			break;

		case CHECK_BEFORE_CHARGING:
			//充电器输出电压验证低于报警值，可以进入充电状态
			if (sPowerManager.sChargeInfo.sChargerCheck.eCheckState == CHECK_APPROVED)
			{
                UINT8 UnFullCharge = 0;
                for (UINT8 i=0; i<BATTERY_NUMBERS; i++)
                {
                    /*如果某块并联中的电池通讯失败*/
                    if  (sPowerManager.sBatteryInfo[i].ComFailSet)
                        continue;
                    UnFullCharge += (sPowerManager.sBatteryInfo[i].SocRaw <= sPowerManager.sBatteryInfo[i].TopSocLimit)?1:0;
                }
                
                /*并联电池组中任意一个电池pack低于FullChargeFloorSoc，都会进入充电状态*/
				if (UnFullCharge)
				{
					sPowerManager.sChargeInfo.ChargeAppState = CHARGING;
                    charge_timeout = 0;
				}	
			}
			//监测电池或者充电异常（非低电量异常），跳转入ALARM状态
			if (sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK)
			{
				sPowerManager.sChargeInfo.ChargeAppState = POWER_ALARM;
			}

			//充电器被拔出时，跳转入非充电状态
			if (sPowerManager.sChargeInfo.eChargerEvent == PULL_OUT)
			{
				sPowerManager.sChargeInfo.ChargeAppState = NOT_CHARGED;
			}

			//如果发生打开电池仓盖的动作，无论在任何情况，则应该直接执行跳转到非充电状态下
			if (sPowerManager.eBatteryCoverState == NOT_COVERED)
			{
				sPowerManager.sChargeInfo.ChargeAppState = NOT_CHARGED;
			}

			break;

		case CHARGING:
            {
                UINT8 OverTopSocLimit = 0;
                UINT8 OverVirtualFullSoc = 0;
            
                for (UINT8 i=0; i<BATTERY_NUMBERS; i++)
                {
                    if (sPowerManager.sBatteryInfo[i].ComFailSet)
                    {
                        OverTopSocLimit += 1;
                        OverVirtualFullSoc += 1;
                        continue;
                    }
                    OverTopSocLimit += (sPowerManager.sBatteryInfo[i].SocRaw >= sPowerManager.sBatteryInfo[i].FullChargeTopSoc)?1:0;
                    // 虚拟电量已满，用于计算超时
                    OverVirtualFullSoc += (sPowerManager.sBatteryInfo[i].SocRaw >= sPowerManager.sBatteryInfo[i].TopSocLimit)?1:0;
                }
            
                //监测电池或者充电异常(非低电量异常)，跳转入ALARM状态
                if (sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK)
                {
                    sPowerManager.sChargeInfo.ChargeAppState = POWER_ALARM;
                }
                //充电器被拔出时，跳转入非充电状态
                else if (sPowerManager.sChargeInfo.eChargerEvent == PULL_OUT)
                {
                    sPowerManager.sChargeInfo.ChargeAppState = NOT_CHARGED;
                }
                //并联电池组中两个电池电量都高于TopSocLimit时，跳转入CHECK_BEFORE_CHARGING
                else if (OverTopSocLimit == BATTERY_NUMBERS)
                {
                    sPowerManager.sChargeInfo.ChargeAppState = CHECK_BEFORE_CHARGING;
                }
                
                // 并联电池组中两个电池的虚拟电量都达到了100%
                if(OverVirtualFullSoc == BATTERY_NUMBERS)
                {
                    charge_timeout++;
                    // 超时判定，如果虚拟电量冲满的情况下，持续充电超过FULLSOC_CHARGING_MAXTIME时间自动关闭mos
                    if(charge_timeout >= FULLSOC_CHARGING_MAXTIME) 
                    {
                        charge_timeout = 0;
                        sPowerManager.sChargeInfo.ChargeAppState = CHECK_BEFORE_CHARGING;
                    }
                }
                else
                {
                    charge_timeout = 0;
                }

                //如果发生打开电池仓盖的动作，无论在任何情况，则应该直接执行跳转到非充电状态下
                if (sPowerManager.eBatteryCoverState == NOT_COVERED)
                {
                    sPowerManager.sChargeInfo.ChargeAppState = NOT_CHARGED;
                }
            }
			break;

		case POWER_ALARM:
            //当故障清除时，应该选择让机器重新判定是否进入充电状态
            if (!(sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK))
            {
                sPowerManager.sChargeInfo.ChargeAppState = NOT_CHARGED;
            }                
			break;
		
		default:
			break;
	}

	//执行MOS开关动作
	if (sPowerManager.sChargeInfo.ChargeAppState == CHARGING)
	{
		//确认在该状态中MOS管处于导通状态
		if (sPowerManager.sChargeInfo.eChargeMosState != MOS_ON)
		{
			EnableCharge(ApplicationMode);
		}		
	}
	else
	{
		//确认在该状态中MOS管处于截止状态
		if (sPowerManager.sChargeInfo.eChargeMosState != MOS_OFF)
		{
			DisableCharge(ApplicationMode);
		}
	}

	//根据充电状态，切换灯带状态
	if (sPowerManager.sChargeInfo.ChargeAppState == CHARGING)// || (sPowerManager.sChargeInfo.ChargeAppState == CHECK_BEFORE_CHARGING))
	{
        UINT16 AveSoc = 0;
        UINT8  ConnectedBatteryCnt = 0;
        for(UINT8 i=0; i<BATTERY_NUMBERS; i++)
        {
            if (!sPowerManager.sBatteryInfo[i].ComFailSet)
            {
                AveSoc += sPowerManager.sBatteryInfo[i].SocOptimized;
                ConnectedBatteryCnt ++;
            }
        }
        
        if (ConnectedBatteryCnt)
        {
            AveSoc = AveSoc/ConnectedBatteryCnt;
        }
        else
        {
            AveSoc = 0;        
        }   
        
        
		//插上充电后，即显示充电呼吸效果
		LedFsmEventHandle(&sLedFsm, LED_EVENT_CHARGE_ING, GetBatteryLevelForLed(AveSoc), NULL);
	}
	else if(sPowerManager.sChargeInfo.ChargeAppState == NOT_CHARGED || (sPowerManager.sChargeInfo.ChargeAppState == CHECK_BEFORE_CHARGING))
	{
		//拔掉充电器后，进入正常的蓝色待机状态
		LedFsmEventHandle(&sLedFsm, LED_EVENT_CHARGE_OUT, LED_STATE_AWAIT, NULL);
	}
	else if(sPowerManager.sChargeInfo.ChargeAppState == POWER_ALARM)
	{
        LedFsmEventHandle(&sLedFsm, LED_EVENT_CHARGE_OUT, LED_STATE_AWAIT, NULL);
		//当出现故障时，接收上位机控制，提示红色闪烁故障灯
		LedFsmEventHandle(&sLedFsm, LED_EVENT_REMOTE_CONTROL, LED_STATE_ERROR, NULL);
	}
	
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ClearPowerAlarmReg(void)
{
	//清除IIC通信失败 & 电池电量过低以外的故障
	sPowerManager.sAlarm.PowerAlarmReg.all &= 0x101;

	sPowerManager.sAlarm.ChargeCurrentOverCnt = 0;
	sPowerManager.sAlarm.ChargerVolateOverCnt = 0;
    
    for (UINT8 i=0; i<BATTERY_NUMBERS; i++)
    {
        sPowerManager.sAlarm.CellTempOverCnt[i] = 0;
        sPowerManager.sAlarm.BatteryVolateOverCnt[i] = 0;   
    }
}



/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void lidarPowerOnOffExec(void)
{
    if(sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower)
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = A2M7_LIDAR_DEFAULT_SPEED;
    }
    
	//在雷达打开状态下不能进行调速，必须先关闭雷达，再重新打开
	if (sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower != sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower ||
        sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarPowerCmdSet)
	{
        sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarPowerCmdSet = 0;
        
		if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower)
		{
            LidarPowerOn();
			if ((gMachineInfo.ldsSensorVersion == 9) || (gMachineInfo.ldsSensorVersion == 10))
			{
                A2M7_CtrlOn(sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed);
			}
            sPowerManager.sBoardPowerInfo.PowerOnState.lidarSpeed = sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed;			
		}
		else
		{
			LidarPowerOff();
			if ((gMachineInfo.ldsSensorVersion == 9) || (gMachineInfo.ldsSensorVersion == 10))
			{
				A2M7_CtrlOff();
			}
            sPowerManager.sBoardPowerInfo.PowerOnState.lidarSpeed = sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = 0;
		}
        
        sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower;
	}
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void DisinfectionModulePowerOnOffExec(void)
{
	if (sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower != sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower ||
        sPowerManager.sBoardPowerInfo.PowerOnConfig.disinfectionModulePowerCmdSet)
	{
        sPowerManager.sBoardPowerInfo.PowerOnConfig.disinfectionModulePowerCmdSet = 0;
        
		if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower)
		{	
            DisinfectionModulePowerOn();
		}
		else
		{
            DisinfectionModulePowerOff();
		}
        
        sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower;
	}
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void LdsPowerCtrl(UINT8 En, UINT8 Speed)
{
    if (En)
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = ON;
        if (0 == Speed)
        {
            Speed = A2M7_LIDAR_DEFAULT_SPEED;
        }
        sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = Speed;
    }
    else
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;
    }
    sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarPowerCmdSet = 1;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void DisinfectionModulePowerCtrl(UINT8 En)
{
    if (En)
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower = ON;
    }
    else
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower = OFF;
    }
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SpeakerOnOffExec(void)
{
	//上电后3S钟才打开音推使能，避免音响开机爆音问题
	if (sPowerManager.uTick < 3*40)
		return;

	if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower != sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower)
	{
		sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower;
		if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower)
		{
			YDA138_MuteOff();
		}
		else
		{
			YDA138_MuteOn();
		}
	}
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ClearChargerPlugEvent(void)
{
	sPowerManager.sChargeInfo.eChargerEvent = NULL_CHARGER_EVENT;
}

uint32_t over_cnt = 0;
/***********************************************************************
 * DESCRIPTION: 10HZ
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ReadBatteryInfo(struct BatteryInfoStruct *pBattery)
{
    UINT8 tmp=0;
    INT16 tmp1=0;
    UINT32 tmp2 = 0;
    UINT16 tmp3[7] = {0};
    UINT8 ret = 0;
    PRIVATE UINT32 _300ms_ReadEndtime[BATTERY_NUMBERS] = {0};
    PRIVATE UINT32 _10s_ReadEndtime[BATTERY_NUMBERS] = {0};
    uint8_t b_id = 0;
    
     
    for( b_id =0; b_id < BATTERY_NUMBERS; b_id++ )
    {
        if( pBattery == &( sPowerManager.sBatteryInfo[b_id] ) )
        {
            break;
        }
    }
    
	if (sPowerManager.uTick - _300ms_ReadEndtime[pBattery->id] >= 12)
    {
        #define BAT_Soc_DEBOUNCE  4
        
        /*Soc*/
        if(BatteryReadSoc(&tmp, pBattery->Addr, pBattery->BMS_icType))
        {
            if(tmp != pBattery->SocRaw)
            {
                if(tmp == pBattery->SocBuffer)
                {
                    if(pBattery->SocDebounceCnt >= BAT_Soc_DEBOUNCE)
                    {
                        // 用于滤掉电池电量偶尔出现0的情况，如果这个滤波还无法解决问题，换用 labs(pBattery->SocRaw - tmp > 5)判定
                        if( tmp == 0 && pBattery->SocRaw > 5)
                        {
                            // 故障不操作，保持上一个状态，等待重新获取数据
                        }
                        else
                        {
                            pBattery->SocRaw = pBattery->SocBuffer;
                        }
                        pBattery->SocDebounceCnt = 0;
                    }
                    else
                    {
                        pBattery->SocDebounceCnt++;
                    }
                }
                else
                {
                    pBattery->SocBuffer = tmp;
                    pBattery->SocDebounceCnt = 0;
                }
            }
            else
            {
                pBattery->SocBuffer = 0;
            }

            pBattery->ComFailCnt = 0;
        }
        else
        {
            pBattery->ComFailCnt++;
        }
        
        
        /*Read Cell Temp*/
        if(BatteryReadTemp(&tmp1, pBattery->Addr, pBattery->BMS_icType))
        {
            if( tmp1 > sPowerManager.sAlarm.CellTempMax )
            {
                sPowerManager.sAlarm.CellTempOverCnt[b_id]++;
            } else {
                if( sPowerManager.sAlarm.CellTempOverCnt[b_id] > 0 )
                {
                    sPowerManager.sAlarm.CellTempOverCnt[b_id]--;
                }
            }
            pBattery->CellTemp = tmp1;  
        }
        
        /*Read DisCharge & Charge current*/
        if(BatteryReadCurrent(&tmp1, pBattery->Addr, pBattery->BMS_icType))
        {
            if (tmp1 > 0)
            {
                pBattery->ChargeCurrent = tmp1;
                pBattery->DischargeCurrent = 0;
            }
            else if (tmp1 < 0)
            {
                pBattery->DischargeCurrent = -tmp1;
                pBattery->ChargeCurrent = 0;
            }
            else
            {
                pBattery->ChargeCurrent = pBattery->DischargeCurrent = 0;
            }
              
        }   
        
        /*battery voltage iic*/
        ret = BatteryReadBv(&tmp2, pBattery->Addr, pBattery->BMS_icType);
        if (ret || tmp2 <= 45000 )
        {
            pBattery->Voltage = tmp2;
            if(tmp2 > sPowerManager.sAlarm.BatteryVoltageMax )
            {
                sPowerManager.sAlarm.BatteryVolateOverCnt[b_id]++;
                over_cnt++;
            } else {
                if(sPowerManager.sAlarm.BatteryVolateOverCnt[b_id] > 0 )
                {
                    sPowerManager.sAlarm.BatteryVolateOverCnt[b_id]--;
                }
            }
        }
        
        /*Read Protect*/
        
        /*Read Fault*/
        
        _300ms_ReadEndtime[pBattery->id] = sPowerManager.uTick;
    }
    
    
    /*update fcc, remaining capacity, battery voltage, soh, cycle, CellVoltage, IC_type Per 10s*/
    if (sPowerManager.uTick - _10s_ReadEndtime[pBattery->id] >= 400)
    {
        /*full charge capacity*/
        ret = BatteryReadFcc(&tmp2, pBattery->Addr, pBattery->BMS_icType);
        if (ret)
        {
            pBattery->FullCapacity = tmp2;
        }
        
        
        /*remaining capacity*/
        ret = BatteryReadRc(&tmp2, pBattery->Addr, pBattery->BMS_icType);
        if (ret)
        {
            pBattery->RemaingCapacity = tmp2;
        }
        
        
        /*state of health*/
        ret = BatteryReadSoh(&tmp2, pBattery->Addr, pBattery->BMS_icType);
        if (ret)
        {
            pBattery->SOH = tmp2;
        }
        
        
        /*cycle count*/
        ret = BatteryReadCc(&tmp2, pBattery->Addr, pBattery->BMS_icType);
        if (ret)
        {
            pBattery->CycleCnt = tmp2;
        }
        
        
        /*Cell voltage*/
        ret = BatteryReadCellVoltage(tmp3, pBattery->Addr, pBattery->BMS_icType);
        if (ret)
        {
            memcpy(pBattery->CellVoltage, tmp3, 16);
        }

        _10s_ReadEndtime[pBattery->id] = sPowerManager.uTick;
    }
}

/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void UploadBatteryInfo(struct BatteryInfoStruct *pBattery)
{
    PRIVATE UINT32 _1s_ReadEndtime[BATTERY_NUMBERS] = {0};
    PRIVATE UINT32 _10s_ReadEndtime[BATTERY_NUMBERS] = {0};
    
    struct BatteryObjectEntryStruct* p_OBJ = NULL;
    UINT32 ObjData;
    UINT16 ObjLen;
    
    if (sPowerManager.uTick - _10s_ReadEndtime[pBattery->id] >= 80)
    {   
        #define _10s_SubIndex_SIZE 13
        const UINT8 SubIndex[_10s_SubIndex_SIZE] = {0x00, 0x01, 0x03, 0x04, 0x0B, 0x0C, 
                                                    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46};         
        for(UINT8 i=0; i<_10s_SubIndex_SIZE; i++)
        {
            p_OBJ = OBJ_GetBatteryObjectHandle((pBattery->id<<8) + SubIndex[i]);
            if (p_OBJ != NULL)
            {
                ObjLen = p_OBJ->BitLength >> 3;
                ObjData = 0;
                
                memcpy(&ObjData, (UINT8 *)p_OBJ->pVarPtr, ObjLen);
                
                CanSendBatteryInfo(p_OBJ->Index, ObjData);
                delay_ms(5);
            }            
        }
        
        CanSendMergedBatteryInfo( &sPowerManager);
        _10s_ReadEndtime[pBattery->id] = sPowerManager.uTick;
    }
    
    if (sPowerManager.uTick - _1s_ReadEndtime[pBattery->id] >= 40)
    {   
        #define _1s_SubIndex_SIZE 7
        const UINT8 SubIndex[_1s_SubIndex_SIZE] = {0x02, 0x80, 0x81, 0x82, 0x83, 0x84,
                                                   0xC0};
        
        for(UINT8 i=0; i<_1s_SubIndex_SIZE; i++)
        {
            p_OBJ = OBJ_GetBatteryObjectHandle((pBattery->id<<8) + SubIndex[i]);
            if (p_OBJ != NULL)
            {
                ObjLen = p_OBJ->BitLength >> 3;
                ObjData = 0;
                
                memcpy(&ObjData, (UINT8 *)p_OBJ->pVarPtr, ObjLen);
                
                CanSendBatteryInfo(p_OBJ->Index, ObjData);
                delay_ms(5);                
            }            
        }
        
        _1s_ReadEndtime[pBattery->id] = sPowerManager.uTick;
    }
}

/***********************************************************************
 * DESCRIPTION: 
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void UpdateBatteryPcDisplay(void)
{
    PRIVATE UINT32 op_time = 0;
    
    if (sPowerManager.uTick - op_time <= 40)
        return;
    op_time = sPowerManager.uTick;
    
    gSensorData.PowerManageAlarm0x4011 = sPowerManager.sAlarm.PowerAlarmReg.all;
    gSensorData.No1_Type0x4012 = sPowerManager.sBatteryInfo[0].BMS_icType;
    gSensorData.No2_Type0x4026 = sPowerManager.sBatteryInfo[1].BMS_icType;
    
    UINT32 len = (UINT32)(&gSensorData.No1_LowPowerSet0x4025) - (UINT32)(&gSensorData.No1_PackVoltage0x4013);
    memcpy(&gSensorData.No1_PackVoltage0x4013, &sPowerManager.sBatteryInfo[0].Voltage, len);
    memcpy(&gSensorData.No2_PackVoltage0x4027, &sPowerManager.sBatteryInfo[1].Voltage, len);
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE struct BatteryObjectEntryStruct* OBJ_GetBatteryObjectHandle(UINT16 index )
{
    #define BATTERY_OBJ_NUM (sizeof(sBatteryOjectDic)/sizeof(struct BatteryObjectEntryStruct))
    UINT16 TempMin = 0;
    UINT16 TempMax = BATTERY_OBJ_NUM;
    UINT16 MidVal = 0;
    struct BatteryObjectEntryStruct *pObjEntry = NULL;

    while(TempMax >= TempMin)
    {     
        MidVal = (TempMax + TempMin) / 2;
        pObjEntry = (struct BatteryObjectEntryStruct*)&sBatteryOjectDic[MidVal]; 
        
        if(index < pObjEntry->Index)
        {
            TempMax =  MidVal-1;
        }
        else if((index > pObjEntry->Index))
        {
            TempMin = MidVal+1;
        }
        else
        {
            return pObjEntry;
        }     
    }
    
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadSoc(UINT8 *pSoc, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
            
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x0D, pSoc, 1);
            break;
            
        default:
            break;
    }
    
    if (ret)
    {
        return 1;
    }
    
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadTemp(INT16 *pTemp, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
            
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x08, (UINT8 *)pTemp, 2);
            *pTemp = *pTemp - 2730;
            break;            
            
        default:
            break;
    }
    
    if (ret)
    {
        return 1;
    } 
    return 0;    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadFcc(UINT32 *pFcc, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
            
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x10, (UINT8 *)pFcc, 2);
            break;            
              
        default:
            break;
    }
    
    
    if (ret)
    {
        return 1;
    } 
    
    return 0;    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadRc(UINT32 *pRc, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
            
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x0F, (UINT8 *)pRc, 2);
            break;            
            
        default:
            break;
    }
    
    if (ret)
    {
        return 1;
    }  
    
    return 0;   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadBv(UINT32 *pBv, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;

        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x09, (UINT8 *)pBv, 2);
            break;            
            
        default:
            break;
    }
    
    if (ret)
    {
        return 1;
    } 
    return 0;       
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadSoh(UINT32 *pSoh, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
           
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x7C, (UINT8 *)pSoh, 1);
            
            if (ret)
            {
                *pSoh = *pSoh & 0xFF;
            }  
            break;            
            
        default:
            break;
    }
    
    if (ret)
    {
        return 1;
    }
    
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadCc(UINT32 *pCc, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
            
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x7B, (UINT8 *)pCc, 2);            
            break;             
            
        default:
            break;
    }
    
    if (ret)
    {
        return 1;
    }  
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadCellVoltage(UINT16  CellV[7], UINT8 Addr, UINT8 BatteryType)
{ 
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
        
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20: 
            for(UINT8 i=0; i<7; i++)
            {
                ret = Battery_Serial_Read(Addr, 0x30+i, (UINT8 *)CellV+2*i, 2);
                
                if(!ret)
                {
                    return 0;
                }
            }
            
            break;           
            
        default:
            break;
    }
    
    return 1;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadCurrent(INT16 *pCurrent, UINT8 Addr, UINT8 BatteryType)
{
    UINT8 ret = 0;
    
    switch(BatteryType)
    {
        case NONE_RECOGNIZED:
            break;
            
        case GF_7S6P_0x18:
        case GF_7S8P_0x18:
        case GF_7S6P_0x20:
        case GF_7S8P_0x20:  
            ret = Battery_Serial_Read(Addr, 0x0A, (UINT8 *)pCurrent, 2);            
            break;             
            
        default:
            break;
    }
    
    if (ret)
    {
        return 1;
    }  
    return 0;
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void SetVbusPower(UINT8 State)
{
    if (!State)
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = OFF;
        sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.VbusPower = OFF;
    }
    else
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.VbusPower = ON;
        sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.VbusPower = ON;        
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
uint32_t i2c_total = 0;
uint32_t i2c_err_cnt = 0;
PRIVATE BOOL Battery_Serial_Read(UINT8 SlaveAddress, UINT8 Readaddr, UINT8 *Str,UINT16 Len)
{
    HAL_StatusTypeDef ret;
    
    ret = HAL_I2C_Mem_Read(&hi2c3, SlaveAddress, Readaddr, I2C_MEMADD_SIZE_8BIT, Str, Len, 1000);
    i2c_total++;
    if (ret != HAL_OK)
    {
        i2c_err_cnt++;
        HAL_I2C_Reset(&hi2c3);
        return 0;
    }

    return 1;
}

/***********************************************************************
 * DESCRIPTION:only support TI bms
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL ManufacturerBlockAccesRead(UINT8 SlaveAddress, UINT8 Cmd, UINT16 macCode, UINT8 *Str, UINT16 Len)
{ 
    UINT8 writeList[3] = {0x02, 0x00, 0x00};
    
    writeList[1] = macCode&0xFF;
    writeList[2] = (macCode>>8)&0xFF;
    
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Write(&hi2c3, SlaveAddress, Cmd, I2C_MEMADD_SIZE_8BIT, writeList, 3, 500);
    
    if (ret != HAL_OK)
    {
        HAL_I2C_Reset(&hi2c3);
        return 0;
    }
    
    ret = HAL_I2C_Mem_Read(&hi2c3, SlaveAddress, Cmd, I2C_MEMADD_SIZE_8BIT, Str, Len, 1000);
    
    if (ret != HAL_OK)
    {
        HAL_I2C_Reset(&hi2c3);
        return 0;
    }
    
    return 1; 
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
#if (DEBUG_VBUS_SOFT_START == 1)
REAL32 Vbus_Debug[200];
REAL32 Vbus_climb_Slope_Debug[200];
#endif
PRIVATE void VbusSoftStartBlock(UINT8 ApplicationMode)
{ 
    UINT8 BlockSoftStartTick = 0;
    UINT8 Vbus_Judge_Cnt = 0;
    REAL32 Vbus_tmp = 0.0f;
    REAL32 Vbus_tmp_last = 0.0f;
    REAL32 Vbus_delta_tmp = 0.0f;
    
    //Ensure 12V PowerOn completely
    DrvPwEnable();
    delay_ms(50);
    
    VbusBufferEnable();
    sPowerManager.sBoardPowerInfo.VbusSoftStartFlag = 0;
    while(1)
    {
        if ((Vbus_delta_tmp < VBUS_VOLTAGE_CLIMB_SLOPE_CONDITION) && (Vbus_tmp > VBUS_VOLTAGE_CONDITION))
        {
            Vbus_Judge_Cnt++;
        }
        else if (Vbus_Judge_Cnt)
        {
            Vbus_Judge_Cnt--;
        }
        
        if (Vbus_Judge_Cnt >= 10)
        {
            VbusEnable();
            VbusBufferDisable();
            SetVbusPower(1);
            sPowerManager.sBoardPowerInfo.VbusSoftStartFlag = 1;
            
#if (DEBUG_VBUS_SOFT_START == 1)            
            memset(Vbus_Debug, 0, 200);
            memset(Vbus_climb_Slope_Debug, 0, 200);
#endif
            
            break;        
        }
        
        if (BlockSoftStartTick >= 200)
        {
            SetVbusPower(0);
            VbusBufferDisable();
            sPowerManager.sBoardPowerInfo.VbusSoftStartFlag = 2;
            break;
        }
        
        AdcSampleStart();
        AdcSampleClearFlag();
        
        Vbus_tmp_last = Vbus_tmp;
        if (!ApplicationMode)
        {
            Vbus_tmp = ADC1->JDR2*ADC1_JDR2_GAIN;
        }
        else
        {
            Vbus_tmp = ADC2->JDR4*ADC2_JDR4_GAIN;
        }        

        Vbus_delta_tmp = Vbus_tmp - Vbus_tmp_last;
#if (DEBUG_VBUS_SOFT_START == 1)
        if (BlockSoftStartTick < 200)
        {
            Vbus_Debug[BlockSoftStartTick] = Vbus_tmp;
            Vbus_climb_Slope_Debug[BlockSoftStartTick] = Vbus_delta_tmp;
        }
#endif        
        
        delay_ms(1);
        BlockSoftStartTick++;
    }
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void VbusSoftStartNoBlock(REAL32 VbusAdcValue)
{
    //100 us/tick
    PRIVATE UINT32 Tick = 0;
    PRIVATE UINT32 Tick_Last = 0;
    PRIVATE REAL32 AdcValueLast = 0.0f;
    PRIVATE UINT8  VbusJudgeCnt = 0;
    PRIVATE UINT8 _1ms_tickCnt = 0;
    
    if (Tick > 2000)
    {
        VbusBufferDisable();
        sPowerManager.sBoardPowerInfo.VbusSoftStartFlag = 2;
        sPowerManager.sBoardPowerInfo.VbusSoftStartEn = 0;
    }
    else if (Tick > 100)
    {
        //Check Per 1ms
        if (Tick - Tick_Last >= 10)
        {
            REAL32 VbusDcDelta = VbusAdcValue - AdcValueLast;
            AdcValueLast = VbusAdcValue;
            Tick_Last = Tick;
            
#if (DEBUG_VBUS_SOFT_START == 1)
            if (_1ms_tickCnt < 200)
            {
                Vbus_Debug[_1ms_tickCnt] = VbusAdcValue;
                Vbus_climb_Slope_Debug[_1ms_tickCnt] = VbusDcDelta;
            }
#endif              
            if ((VbusDcDelta < VBUS_VOLTAGE_CLIMB_SLOPE_CONDITION) &&
                (VbusAdcValue > VBUS_VOLTAGE_CONDITION))
            {
                VbusJudgeCnt++;
            }
            else if (VbusJudgeCnt)
            {
                VbusJudgeCnt--;
            }
            
            if (VbusJudgeCnt >= 10)
            {
                VbusEnable();
                VbusBufferDisable();
                SetVbusPower(1);
                sPowerManager.sBoardPowerInfo.VbusSoftStartFlag = 1;
                sPowerManager.sBoardPowerInfo.VbusSoftStartEn = 0;             
            }
            
            _1ms_tickCnt++;
        }
    }
    else if (!Tick)
    {
        VbusBufferEnable();
    }

    Tick++;
    
    if (!sPowerManager.sBoardPowerInfo.VbusSoftStartEn)
    {
        Tick = 0;
        Tick_Last = 0;
        AdcValueLast = 0.0f;
        VbusJudgeCnt = 0;
        _1ms_tickCnt = 0;     
    }
    

}

/*可能存在的问题
(1)复位起来的时候，可能会突然关掉头部电源，原因是识别为插入充电器
(2)在normal poweron 模式下，插入充电器，然后进入充电模式后，拔出充电器，再执行关机按钮，可能需要较长时间才能关机
(3)	YDA138_MuteOff();  包含延时，需要优化
*/

