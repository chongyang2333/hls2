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
#include "LedDriver.h"
#include "CanApp.h"
#include "string.h"
#include "math.h"
#include "gd32f4xx.h"
#include "MachineAdditionalInfo.h"
#include "gd_hal.h"
#include "delay.h"


#define ON 1
#define OFF 0
#define A2M7_LIDAR_DEFAULT_SPEED  136
#define TI_BQ34Z100_IIC_ADDR      0xAA
#define GSA7S_IIC_ADDR            0x16
#define GF_7S6P_7S8P_IIC_ADDR     0x18
#define DESAY_7S5P_7S8P_IIC_ADDR  0x22

BatteryIICStruct sBatI2C;

struct PowerManagerStruct sPowerManager = {0};
PRIVATE void PowerInfoUpdate(void);
PRIVATE void ChargeCurrentBiasCal(void);
PRIVATE void ChargerPlugEventUpdate(struct ChargeManageStruct *chargeInfo);
PRIVATE void KeyEventUpdate(void);
PRIVATE void PowerAlarmExec(void);
PRIVATE void PM_PowerOnOffExec(void);
PRIVATE void ChargeOnOffExec(enum BatteryManageSystemType BatteryType);
PRIVATE void ClearPowerAlarmReg(void);
PRIVATE void SpeakerOnOffExec(void);
PRIVATE void PowerManageInfoUploadingExec(void);
PRIVATE void ClearKeyEvent(void);
PRIVATE void ClearChargerPlugEvent(void);
PRIVATE UINT8 GetFrame0x73Header(const union PowerAlarmReg alarmRegData, const enum ChargeManageState chargerManageState, const UINT8 batteryLevelOptimized);
PRIVATE BOOL ManufacturerBlockAccesRead(UINT8 SlaveAddress, UINT8 Cmd, UINT16 MAC_Cmd, UINT8 *Str, UINT16 Len);
PRIVATE BOOL Battery_Serial_Read(UINT8 SlaveAddress, UINT8 Readaddr, UINT8 *Str,UINT16 Len, UINT8 i2c_type);
PRIVATE void BatteryI2cReadSOC_Temp(void);
PRIVATE void BatteryI2cReadInternalInfo(void);
PRIVATE BOOL BatteryReadSoc(UINT8 *pSoc);
PRIVATE BOOL BatteryReadTemp(INT16 *pTemp);
PRIVATE BOOL BatteryReadFcc(UINT32 *pFcc);
PRIVATE BOOL BatteryReadRc(UINT32 *pRc);
PRIVATE BOOL BatteryReadBv(UINT32 *pBv);
PRIVATE BOOL BatteryReadSoh(UINT32 *pSoh);
PRIVATE BOOL BatteryReadCc(UINT32 *pCc);
PRIVATE void DisinfectionModulePowerOnOffExec(void);
PRIVATE BOOL BatteryReadSN(UINT32 *pSN);
PRIVATE BOOL BatteryReadCellVoltage(UINT16  CellV[7]);
PRIVATE BOOL BatteryReadSafetyAlert(UINT32 *pSafetyAlert);
PRIVATE BOOL BatteryReadSafetyStatus(UINT32 *pSafetyStatus);
PRIVATE BOOL BatteryReadPFAlert(UINT32 *pPFAlert);
PRIVATE BOOL BatteryReadPFStatus(UINT32 *pPFStatus);
PRIVATE BOOL BatteryReadOperationStatus(UINT32 *pOperationStatus);
PRIVATE BOOL BatteryReadChargingStatus(UINT32 *pChargingStatus);
PRIVATE BOOL BatteryReadLifetimeData(struct BatteryLifeTimeDataBlock *pData);
PRIVATE void VbusSoftStartBlock(UINT8 ApplicationMode);

extern PUBLIC UINT8 ApplicationMode;
extern void HAL_I2C_Reset(uint32_t i2c_periph);
extern PUBLIC void MutePowerAnswer(UINT8 Data);
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void PowerManagerInit(UINT8 ApplicationMode)
{
    sBatI2C.SCL_WritePin = Battery_SCL_WritePin;
    sBatI2C.SDA_WritePin = Battery_SDA_WritePin;
    sBatI2C.SDA_ReadPinState = Battery_SDA_ReadPin;
    sBatI2C.SCL_ReadPinState = Battery_SCL_ReadPin;

    //DisableCharge(ApplicationMode);
    //LedPowerOn();
    //LidarPowerOff();  //初始化已经关了
    //LidarPowerOff();

    delay_us(300);
    ResetACS711(); //todo
    memset(&sPowerManager, 0, sizeof(struct PowerManagerStruct));
    VbusSoftStartBlock(ApplicationMode);

    //读取MCU复位原因，判断是硬件复位还是软件复位，当判断是软件复位时，即使判断为插入充电器开机，也不允许关闭头部电源
    sPowerManager.eSysRstState = GetResetType();
    //读取BKP寄存器中电源管理中的Alarm标志位
    sPowerManager.sAlarm.PowerAlarmReg.all = RTC_BKP_Read(EN_BATTERY_PROTECT_BKP_ADDR);

    sPowerManager.ePowerKeyStateLast = sPowerManager.ePowerKeyState = KEY_UP;

    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.ChassisPower            = ON;
    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LedPower                = ON;
    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower              = OFF;
    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower                = ON;
    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower            = ON;
    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower = OFF;
    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.ChassisPower             = ON;
    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LedPower                 = ON;
    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower               = OFF;
    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.PadPower                 = ON;
    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower             = OFF;
    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower  = OFF;
    sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWERON_INIT;

    sPowerManager.sBatteryInfo.BatteryFloorLevelLimit      = 10;
    sPowerManager.sBatteryInfo.BatteryTopLevelLimit        = 95;
    sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel = 93;
    sPowerManager.sBatteryInfo.BatteryFullChargeTopLevel   = 95;

    sPowerManager.sAlarm.ChargeCurrentMax        = 5500;
    sPowerManager.sAlarm.ChargeTempMax           = 450;  //0.1单位
    sPowerManager.sAlarm.ChargeTempMin           = 10;
    sPowerManager.sAlarm.ChargerVoltageMax       = 31000;
    sPowerManager.sAlarm.BatteryVoltageMax       = 30000;
    sPowerManager.sAlarm.BatteryLowPowerLevel    = 0;
    sPowerManager.sAlarm.ChargeCurrentOverCntMax = 10;
    sPowerManager.sAlarm.ChargerVolateOverCntMax = 10;
    sPowerManager.sAlarm.ChargeTempOverCntMax    = 30;
    sPowerManager.sAlarm.ChargeTempUnderCntMax   = 30;
    sPowerManager.sAlarm.ComDisconnectCntMax     = 10;
    sPowerManager.sAlarm.BatteryVolateOverCntMax = 10;
    sPowerManager.sAlarm.BatteryLowPowerCntMax   = 80;

    sPowerManager.sChargeInfo.ChargerConnectedVoltageThreshold = 27000; //mV
    sPowerManager.sChargeInfo.ChargerConnectedCurrentThreshold = 300; //mA
    sPowerManager.sChargeInfo.ChargeAppState                   = NOT_CHARGED;
    sPowerManager.sChargeInfo.ChargerConnectJudgeTime          = 15;
    sPowerManager.sChargeInfo.ChargerDisconnectJudgeTime       = 10;

    sPowerManager.sBatteryInfo.BMS_icType = NONE_RECOGNIZED;
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
    //PowerInfoUpdate();
    sPowerManager.ePMState = POWER_MANAGE_CHARGE_KEY_EVENT_DETECT;
    //如果powerManage处于Init状态，则先对参数进行校正，在执行真正的电源管理前，需要确保充电关闭，以及外设电源按照合理的状态打开或者关闭
    if(sPowerManager.ePMState == POWER_MANAGE_INIT)
    {
        //ChargeCurrentBiasCal();
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
        //更新充电器插拔事件:充电器插入 && 充电器拔出
        //ChargerPlugEventUpdate(&sPowerManager.sChargeInfo);
        //更新按键事件:按键长按 && 按键短按
        //KeyEventUpdate();
        sPowerManager.ePMState = POWER_MANAGE_ALARM_DETECT;
        break;

    case POWER_MANAGE_ALARM_DETECT:
        //电源相关故障Alarm事件:充电器过压、电池过压、充电过流、通讯线脱落、电量与电压失配
        //PowerAlarmExec();
        sPowerManager.ePMState = POWER_MANAGE_POWER_ON_OFF_CHARGE_EXEC;
        break;

    case POWER_MANAGE_POWER_ON_OFF_CHARGE_EXEC:
        //外设电源开关管理的状态机跳转
        //PM_PowerOnOffExec();
        //充电开关状态机跳转
        //ChargeOnOffExec(sPowerManager.sBatteryInfo.BMS_icType);
        lidarPowerOnOffExec();
        SpeakerOnOffExec();
        sPowerManager.ePMState = POWER_MANAGE_INFO_UPLOAD;
        break;

    case POWER_MANAGE_INFO_UPLOAD:
        //can消息上报
        //PowerManageInfoUploadingExec();
        //清除充电器插入事件、按键等事件
        //ClearKeyEvent();
        //ClearChargerPlugEvent();
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
PRIVATE void PowerInfoUpdate(void)
{
    //这个IO设置为推挽输出，不知道所有机器能否完成读取IO操作
    sPowerManager.sChargeInfo.eChargeMosState = ReadChargeMosState();
    sPowerManager.sChargeInfo.ChargeCurrent = 1000.0f * GetChargeCurrent() - sPowerManager.sChargeInfo.ChargeCurrentBias;
    sPowerManager.sChargeInfo.ChargeVoltage = 1000.0f * GetChargeVoltage();
    sPowerManager.sBatteryInfo.BatteryCurrent = 1000.0f * GetBatteryCurrent();
    sPowerManager.sBatteryInfo.BatteryVoltage = 1000.0f * GetBatteryVoltage();
    sPowerManager.sBatteryInfo.BatteryLevelRaw = gSensorData.BatteryLevel0x400D;
    sPowerManager.sBatteryInfo.BatteryTemp = gSensorData.BatteryTemp0x4010;

    //对电量进行虚拟化
    if(sPowerManager.sBatteryInfo.BatteryLevelRaw <= sPowerManager.sBatteryInfo.BatteryFloorLevelLimit)
    {
        sPowerManager.sBatteryInfo.BatteryLevelOptimized = 0;
    }
    else if(sPowerManager.sBatteryInfo.BatteryLevelRaw >= sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel)
    {
        sPowerManager.sBatteryInfo.BatteryLevelOptimized = 100;
    }
    else
    {
        sPowerManager.sBatteryInfo.BatteryLevelOptimized = (sPowerManager.sBatteryInfo.BatteryLevelRaw - sPowerManager.sBatteryInfo.BatteryFloorLevelLimit) * 100 / (sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel - sPowerManager.sBatteryInfo.BatteryFloorLevelLimit);
    }

    //更新串口工具上的数据显示
    gSensorData.ChargeCurrent0x400C = sPowerManager.sChargeInfo.ChargeCurrent;
    gSensorData.BatteryCurrent0x400B = sPowerManager.sBatteryInfo.BatteryCurrent;
    gSensorData.ChargeVoltage0x400F = sPowerManager.sChargeInfo.ChargeVoltage;
    gSensorData.ChargeState0x400E = sPowerManager.sChargeInfo.eChargeMosState;
    gSensorData.BatteryVoltage0x400A = gParam[0].Vdc0x2309;
    gMachineInfo.batteryVersion = sPowerManager.sBatteryInfo.BMS_icType;
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
    if ((sPowerManager.uTick < CHARGE_CURRENT_CORRECT_LAUNCH_TIME) || (sPowerManager.uTick > CHARGE_CURRENT_CORRECT_END_TIME))
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
            chargeInfo->eChargerConnectState = CONNECT;
        }

        ChargeVoltageReachCnt = 0;
        ChargeVoltageDeltaReachCnt = 0;
    }

    //MOS管导通 && 充电电流小于1A，则判断为发生充电器拔出事件,同时更新此时充电器没有连接
    if ((chargeInfo->ChargeCurrent < chargeInfo->ChargerConnectedCurrentThreshold) && (chargeInfo->eChargeMosState == MOS_ON))
    {
        ChargeCurrentUnreachCnt++;
    }
    else
    {
        ChargeCurrentUnreachCnt = 0;
    }

    if (ChargeCurrentUnreachCnt >= chargeInfo->ChargerDisconnectJudgeTime)
    {
        chargeInfo->eChargerEvent = PULL_OUT;
        chargeInfo->eChargerConnectState = DISCONNECT;
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

    if ((sPowerManager.ePowerKeyStateLast == KEY_UP) && (sPowerManager.ePowerKeyState == KEY_DOWN) && (sPowerManager.ePowerKeyEvent == NULL_KEY_EVENT))
    {
        sPowerManager.ePowerKeyEvent = SHORT_PRESS;
    }

    if ((sPowerManager.ePowerKeyEvent == SHORT_PRESS) && (sPowerManager.ePowerKeyState == KEY_DOWN))
    {
        powerKeyPressCnt++;
    }
    else
    {
        powerKeyPressCnt = 0;
    }

    //2S
    if (powerKeyPressCnt >= 20)
    {
        powerKeyPressCnt = 0;
        sPowerManager.ePowerKeyEvent = LONG_PRESS;
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


    //检查电池电压是否被过充
    if (sPowerManager.sBatteryInfo.BatteryVoltage > sPowerManager.sAlarm.BatteryVoltageMax)
    {
        sPowerManager.sAlarm.BatteryVolateOverCnt++;
    }
    else if(sPowerManager.sAlarm.BatteryVolateOverCnt)
    {
        sPowerManager.sAlarm.BatteryVolateOverCnt--;
    }

    if (sPowerManager.sAlarm.BatteryVolateOverCnt >= sPowerManager.sAlarm.BatteryVolateOverCntMax)
    {
        sPowerManager.sAlarm.BatteryVolateOverCnt = sPowerManager.sAlarm.BatteryVolateOverCntMax;
        sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryVoltageOver = 1;
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

    //检查电池温度
    if (sPowerManager.sBatteryInfo.BatteryTemp > sPowerManager.sAlarm.ChargeTempMax)
    {
        sPowerManager.sAlarm.ChargeTempOverCnt++;
    }
    else if(sPowerManager.sAlarm.ChargeTempOverCnt)
    {
        sPowerManager.sAlarm.ChargeTempOverCnt--;
    }

    if (sPowerManager.sAlarm.ChargeTempOverCnt >= sPowerManager.sAlarm.ChargeTempOverCntMax)
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.ChargeTempOver = 1;
        sPowerManager.sAlarm.ChargeTempOverCnt = sPowerManager.sAlarm.ChargeTempOverCntMax;
    }
    //检查电池充电低温
    if ((INT16)sPowerManager.sBatteryInfo.BatteryTemp < sPowerManager.sAlarm.ChargeTempMin)
    {
        sPowerManager.sAlarm.ChargeTempUnderCnt++;
    }
    else if(sPowerManager.sAlarm.ChargeTempUnderCnt)
    {
        sPowerManager.sAlarm.ChargeTempUnderCnt--;
    }

    if (sPowerManager.sAlarm.ChargeTempUnderCnt >= sPowerManager.sAlarm.ChargeTempUnderCntMax)
    {
        //sPowerManager.sAlarm.PowerAlarmReg.bit.ChargeTempUnder = 1;
        sPowerManager.sAlarm.ChargeTempUnderCnt = sPowerManager.sAlarm.ChargeTempUnderCntMax;
    }
    //检查电池通讯是否脱落
    if (sPowerManager.sAlarm.ComDisconnectCnt >= sPowerManager.sAlarm.ComDisconnectCntMax)
    {
        sPowerManager.sAlarm.ComDisconnectCnt = sPowerManager.sAlarm.ComDisconnectCntMax;
        sPowerManager.sAlarm.PowerAlarmReg.bit.ComDisconnet = 1;
    }
    else if(!sPowerManager.sAlarm.ComDisconnectCnt)
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.ComDisconnet = 0;
    }

    //检查电量是否过低
    if ((sPowerManager.sBatteryInfo.BatteryLevelOptimized <= sPowerManager.sAlarm.BatteryLowPowerLevel) &&
            (!sPowerManager.sAlarm.PowerAlarmReg.bit.ComDisconnet) &&
            (NONE_RECOGNIZED != sPowerManager.sBatteryInfo.BMS_icType))
    {
        sPowerManager.sAlarm.BatteryLowPowerCnt++;
    }
    else if(sPowerManager.sAlarm.BatteryLowPowerCnt)
    {
        sPowerManager.sAlarm.BatteryLowPowerCnt--;
    }

    if (sPowerManager.sAlarm.BatteryLowPowerCnt >= sPowerManager.sAlarm.BatteryLowPowerCntMax)
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower = 1;
        sPowerManager.sAlarm.BatteryLowPowerCnt = sPowerManager.sAlarm.BatteryLowPowerCntMax;
    } else if (!sPowerManager.sAlarm.BatteryLowPowerCnt)
    {
        sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower = 0;
    }

    //如果发现充电器被拔出，则清除相关的alarm标志位，同时清除充电器检查OK的标志位
    if(sPowerManager.sChargeInfo.eChargerEvent == PULL_OUT)
    {
        ClearPowerAlarmReg();
        sPowerManager.sChargeInfo.sChargerCheck.eCheckState = CHECK_APPENDING;
    }

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

            sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = CHARGE_POWERON;
        }
        //增加判断条件sPowerManager.ePowerKeyState == KEY_UP是为了防止按键启动的时候松动了一下开机按钮，导致按下刚开机就直接关机了
        else if(sPowerManager.ePowerKeyState == KEY_UP)
        {
            sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = ON;
            sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = ON;
            sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = ON;

            sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = NORMAL_POWERON;
        }
        break;

    case NORMAL_POWERON:
        //在NORMAL_POWERON状态中允许使能音推、雷达等
        if (sPowerManager.ePowerKeyEvent == LONG_PRESS)
        {
            if(sPowerManager.sChargeInfo.eChargerConnectState == CONNECT)
            {
                if (!(sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK))
                {
                    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;
                    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                    sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                    sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = CHARGE_POWERON;
                }
            }
            else
            {
                //关机前先关闭音推使能，防止关机时出现爆音问题
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
                sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                EnableMachineAddInfoSave();
            }
        }

        //当在非充电状态中,低电量的时候跳转入POWEROFF
        if (sPowerManager.sAlarm.PowerAlarmReg.bit.BatteryLowPower)
        {
            if ((sPowerManager.sChargeInfo.eChargerConnectState != CONNECT) && (!sPowerManager.sAlarm.PowerAlarmReg.bit.ComDisconnet))
            {
                sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;

                sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
                EnableMachineAddInfoSave();
            }
        }
        break;

    case CHARGE_POWERON:
        //当检测故障(非低电量故障)时，应该进入正常POWERON状态，打开头部平板，方便上位机做声光提示
        if (sPowerManager.sAlarm.PowerAlarmReg.all & ALARM_LOW_POWER_MASK)
        {
            sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower = ON;
            sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.PadPower = ON;
            sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = ON;
            sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = A2M7_LIDAR_DEFAULT_SPEED;

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

            sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = NORMAL_POWERON;
        }
        //由于插上充电器导致开机的情形下，当拔出充电器时应该跳转进入关机状态
        else if(sPowerManager.sChargeInfo.eChargerEvent == PULL_OUT)
        {
            sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower = OFF;
            sPowerManager.sBoardPowerInfo.PowerOnOnffAppState = POWEROFF;
            EnableMachineAddInfoSave();
        }

        break;

    case POWEROFF:
        //执行关机操作
        LedFsmEventHandle(&sLedFsm, LED_EVENT_MCU_POWEROFF, LED_STATE_CLOSE, NULL);
        if( IsMachineAddInfoSaveOK() )
        {
            McuPowerOff();
        }
        break;

    default:
        break;
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
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ChargeOnOffExec(enum BatteryManageSystemType BatteryType)
{
    if (BatteryType == NONE_RECOGNIZED)
    {
        return;
    }

    switch(sPowerManager.sChargeInfo.ChargeAppState)
    {
    case NOT_CHARGED:
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
            if (sPowerManager.sBatteryInfo.BatteryLevelRaw <= sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel)
            {
                sPowerManager.sChargeInfo.ChargeAppState = CHARGING;
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

        break;

    case CHARGING:
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
        //电量高于95%时，跳转入CHECK_BEFORE_CHARGING
        else if (sPowerManager.sBatteryInfo.BatteryLevelRaw >= sPowerManager.sBatteryInfo.BatteryTopLevelLimit)
        {
            sPowerManager.sChargeInfo.ChargeAppState = CHECK_BEFORE_CHARGING;
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
    if ((sPowerManager.sChargeInfo.ChargeAppState == CHARGING) || (sPowerManager.sChargeInfo.ChargeAppState == CHECK_BEFORE_CHARGING))
    {
        //插上充电后，即显示充电呼吸效果
        LedFsmEventHandle(&sLedFsm, LED_EVENT_CHARGE_ING, GetBatteryLevelForLed(sPowerManager.sBatteryInfo.BatteryLevelOptimized), NULL);
    }
    else if(sPowerManager.sChargeInfo.ChargeAppState == NOT_CHARGED)
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
    sPowerManager.sAlarm.ChargeTempOverCnt = 0;
    sPowerManager.sAlarm.ChargeTempUnderCnt = 0;
    sPowerManager.sAlarm.BatteryVolateOverCnt = 0;
}



/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void lidarPowerOnOffExec(void)
{
    static UINT16 leds_cnt = 0;
    if(sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower)
    {
        sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = A2M7_LIDAR_DEFAULT_SPEED;
    }

    //在雷达打开状态下不能进行调速，必须先关闭雷达，再重新打开
    if ((sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower != sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower) ||
            sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarPowerCmdSet)
    {
        if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower)
        {
            LidarPowerOn();
            if ((gMachineInfo.ldsSensorVersion == 9) || (gMachineInfo.ldsSensorVersion == 10))
            {
                if(leds_cnt++>20)
                {
                    leds_cnt = 20;
                    A2M7_CtrlOn(sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed);
                    sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower;
                    sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarPowerCmdSet = 0;
                }
            }
            else
            {
                sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower;
                sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarPowerCmdSet = 0;
            }
            sPowerManager.sBoardPowerInfo.PowerOnState.lidarSpeed = sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed;
        }
        else
        {
            leds_cnt = 0;
            LidarPowerOff();
            if ((gMachineInfo.ldsSensorVersion == 9) || (gMachineInfo.ldsSensorVersion == 10))
            {
                A2M7_CtrlOff();
            }
            sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarPowerCmdSet = 0;
            sPowerManager.sBoardPowerInfo.PowerOnState.lidarSpeed = sPowerManager.sBoardPowerInfo.PowerOnConfig.lidarSpeed = 0;
            sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.LidarPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.LidarPower;
        }

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
    if (sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower != sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower)
    {
        sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.DisinfectionModulePower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower;
        if (sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.DisinfectionModulePower)
        {
            DisinfectionModulePowerOn();
        }
        else
        {
            DisinfectionModulePowerOff();
        }
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
            MusicPwEnable();
            MuteDisable();
            //YDA138_MuteOff();
        }
        else
        {
            MuteEnable();
            MusicPwDisable();
            //YDA138_MuteOn();
        }
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PowerManageInfoUploadingExec(void)
{
    //为了填之前协议的坑，需要对0x73帧头的数据做特殊处理
    UINT8 dataFrame0x73Header = GetFrame0x73Header(sPowerManager.sAlarm.PowerAlarmReg, sPowerManager.sChargeInfo.ChargeAppState, sPowerManager.sBatteryInfo.BatteryLevelOptimized);
    //上传电量、异常等相关信息
    CanSendBatteryChargeInfo(dataFrame0x73Header, sPowerManager.sBatteryInfo.BatteryLevelOptimized, sPowerManager.sBatteryInfo.BatteryLevelRaw, sPowerManager.sBatteryInfo.BatteryVoltage);
    //上传电流
    CanSendSupplyChargeVI_Info(sPowerManager.sChargeInfo.ChargeVoltage, sPowerManager.sChargeInfo.ChargeCurrent, sPowerManager.sBatteryInfo.BatteryLevelOptimized);
    //上传电池温度等信息
    CanSendBatteryChargeExInfo(sPowerManager.sBatteryInfo.BatteryTemp);
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


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 GetFrame0x73Header(const union PowerAlarmReg alarmRegData, const enum ChargeManageState chargerManageState, const UINT8 batteryLevelOptimized)
{
    if (alarmRegData.bit.BatteryVoltageOver || alarmRegData.bit.ChargerVoltageOver)
    {
        return 6;
    }
    else if (alarmRegData.bit.ChargeCurrentOVer)
    {
        return 7;
    }
    else if(alarmRegData.bit.ChargeTempOver)
    {
        return 8;
    }
    else if(alarmRegData.bit.ComDisconnet)
    {
        return 5;
    }
    else if(alarmRegData.bit.ChargeTempUnder)
    {
        return 9;
    }
    else if((chargerManageState == CHECK_BEFORE_CHARGING) || ((chargerManageState == CHARGING)))
    {
        if(batteryLevelOptimized >= 100)
        {
            return 2; //充满
        }
        else
        {
            return 1; //正在充电
        }
    }
    else if(chargerManageState == NOT_CHARGED)
    {
        return 0; //未接充电器
    }

    return 0xFF;
}

/***********************************************************************
 * DESCRIPTION: 10HZ
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void BatteryI2cReadSOC_Temp(void)
{
#define BAT_LEVLE_DEBOUNCE_AMOUNT  6
    static UINT8 batteryBuffer = 0;
    static UINT8 batteryDebounceCnt = 0;

    static UINT32 ReadOperationTime = 0;

    static UINT8 step = 0;
    UINT8 tmp=0;
    INT16 tmp1=0;
    static INT16  batterry_temp[4]= {100,100,100,100};

    //300ms刷新一次电池电量以及温度信息
    if (sPowerManager.uTick - ReadOperationTime <= 4)  //25ms*4 = 100ms
        return;

    switch(step) {
    case 0:
        if(BatteryReadSoc(&tmp))
        {
            if(tmp != gSensorData.BatteryLevel0x400D)
            {
                if(tmp == batteryBuffer)
                {
                    if(batteryDebounceCnt >= BAT_LEVLE_DEBOUNCE_AMOUNT)
                    {
                        gSensorData.BatteryLevel0x400D = batteryBuffer;
                        batteryDebounceCnt = 0;
                    }
                    else
                    {
                        batteryDebounceCnt++;
                    }
                }
                else
                {
                    batteryBuffer = tmp;
                    batteryDebounceCnt = 0;
                }
            }
            else
            {
                batteryDebounceCnt = 0;
            }

            sPowerManager.sAlarm.ComDisconnectCnt = 0;
        }
        else
        {
            sPowerManager.sAlarm.ComDisconnectCnt++;
        }

        break;

    case 1: //第一次读取温度
        tmp = BatteryReadTemp(&tmp1);
        if(tmp == 1)
        {
            batterry_temp[0]= tmp1 - 2730;
        }
        else
        {
            batterry_temp[0] = 300; /*30 鈩�*/
        }
        ReadOperationTime = sPowerManager.uTick;
        break;
    case 2: //第二次读取温度
        tmp = BatteryReadTemp(&tmp1);
        if(tmp == 1)
        {
            batterry_temp[1]= tmp1 - 2730;
        }
        else
        {
            batterry_temp[1] = 300; /*30 鈩�*/
        }
        if(batterry_temp[0]>batterry_temp[1])
        {
            tmp1 = batterry_temp[0];
            batterry_temp[0] = batterry_temp[1];
            batterry_temp[1] = tmp1;
        }
        ReadOperationTime = sPowerManager.uTick;
        break;
    case 3: //第三次读取温度
        tmp = BatteryReadTemp(&tmp1);
        if(tmp == 1)
        {
            batterry_temp[2]= tmp1 - 2730;
        }
        else
        {
            batterry_temp[2] = 300; /*30 ℃*/
        }
        if(batterry_temp[1]>batterry_temp[2])
        {
            tmp1 = batterry_temp[1];
            batterry_temp[2] = batterry_temp[1];
            batterry_temp[1] = tmp1;
        }
        ReadOperationTime = sPowerManager.uTick;
        break;
    case 4://第四次读取温度
        tmp = BatteryReadTemp(&tmp1);
        if(tmp == 1)
        {
            batterry_temp[3]= tmp1 - 2730;
        }
        else
        {
            batterry_temp[3] = 300; /*30 ℃*/
        }
        if(batterry_temp[2]>batterry_temp[3])
        {
            tmp1 = batterry_temp[2];
            batterry_temp[2] = batterry_temp[3];
            batterry_temp[3] = tmp1;
        } // batterry_temp[3]为最大的值
        //再筛出最小值
        for(tmp = 0; tmp<1; tmp++)
        {
            if(batterry_temp[tmp]<batterry_temp[tmp+1])
            {
                tmp1 = batterry_temp[tmp];
                batterry_temp[tmp] = batterry_temp[tmp+1];
                batterry_temp[tmp+1] = tmp1;
            }
        }// batterry_temp[2]为最小值，去除最大和最小值，剩余两个中间值取平均
        gSensorData.BatteryTemp0x4010 =(batterry_temp[0]+batterry_temp[1])>>1;
        ReadOperationTime = sPowerManager.uTick;

        break;
    }
    (step >= 4) ? (step = 0) : step++;
}


/***********************************************************************
 * DESCRIPTION: 0.01HZ
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void BatteryI2cReadLifetimeDataBlock(struct BatteryLifeTimeDataBlock *pData)
{
    static UINT32 ReadStartTime = 0;
    UINT8 i = 0;
    UINT8 ret = 0;
    UINT8 index = 0x80;
    UINT16 *pCur16 = (UINT16 *)pData;
    UINT8 *pCur8;

    //100s刷新一次LifeTime Data等信息
    if (sPowerManager.uTick - ReadStartTime <= 400)
        return;

    ret = BatteryReadLifetimeData(pData);

    if(ret)
    {
        for (i=0; pCur16 <= &pData->MaxAvgDsgPower; i++)
        {
            CanSendBatteryInfo(index+i, *pCur16);
            pCur16 += 1;
        }

        for(pCur8 = (UINT8 *)pCur16; pCur8 <= &pData->NoOfWDTResets; i++)
        {
            CanSendBatteryInfo(index+i, *pCur8);
            pCur8 += 1;
        }

        for(pCur16 = (UINT16 *)pCur8; pCur16 <= &pData->LastRaDisable; i++)
        {
            CanSendBatteryInfo(index+i, *pCur16);
            pCur16 += 1;
        }
    }

    ReadStartTime = sPowerManager.uTick;
}

/***********************************************************************
 * DESCRIPTION: 0.1HZ
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void BatteryI2cReadInternalInfo(void)
{
    static UINT32 ReadStartTime = 0;

    static UINT8 step = 0;
    UINT32 tmp = 0;
    UINT8 i = 0;
    UINT8	ret = 0;
    UINT8   ret1 = 0;
    UINT32 Tmp2 = 0;
    UINT8 start_index = 0x40;

    //10s刷新一次电池SOH、容量、cycle等信息
    if (sPowerManager.uTick - ReadStartTime <= 400)
        return;


    switch(step)
    {
    case 0:
        /*full charge capacity*/
        ret = BatteryReadFcc(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.BatteryFullCapacity = tmp;
        }
        break;

    case 1:
        /*remaining capacity*/
        ret = BatteryReadRc(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.BatteryRemaingCapacity = tmp;
        }
        break;

    case 2:
        /*battery voltage iic*/
        ret = BatteryReadBv(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.BatteryVoltageIIC = tmp;
        }
        break;

    case 3:
        /*state of health*/
        ret = BatteryReadSoh(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.SOH = tmp;
        }
        break;

    case 4:
        /*cycle count*/
        ret = BatteryReadCc(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.CycleCnt = tmp;
        }
        break;

    case 5:
        /*Safety Alert*/
//			ret = BatteryReadSafetyAlert(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.SafetyAlert = tmp;
        }
        break;

    case 6:
        /*Safety Status*/
//			ret = BatteryReadSafetyStatus(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.SafetyStatus = tmp;
        }
        break;

    case 7:
        /*PF Alert*/
//			ret = BatteryReadPFAlert(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.PFAlert = tmp;
        }
        break;

    case 8:
        /*PF Status*/
//			ret = BatteryReadPFStatus(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.PFStatus = tmp;
        }
        break;

    case 9:
        /*Operation Status*/
//			ret = BatteryReadOperationStatus(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.OperationStatus = tmp;
        }
        break;

    case 10:
        /*Charging Status*/
//			ret = BatteryReadChargingStatus(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.ChargingStatus = tmp;
        }
        break;

    case 11:
        /*serial number*/
        ret = BatteryReadSN(&tmp);
        if (ret)
        {
            sPowerManager.sBatteryInfo.BatterySN = tmp;
        }
        break;

    case 12:
        /*battery type*/
        ret = 1;
        tmp = sPowerManager.sBatteryInfo.BMS_icType;

        break;

    case 13:
        /*cell voltage*/
        ret1 = BatteryReadCellVoltage(sPowerManager.sBatteryInfo.CellVoltage);
        ReadStartTime = sPowerManager.uTick;
        break;

    default:
        break;
    }

    /*upload normal status*/
    if (ret)
    {
        CanSendBatteryInfo(step, tmp);
        delay_us(2000);
    }

    /*upload cell voltage*/
    if (ret1)
    {
        for (i=0; i<7; i++)
        {
            CanSendBatteryInfo(start_index+i, *(sPowerManager.sBatteryInfo.CellVoltage+i));
            delay_us(2000);
        }
    }

    (step >= 14) ? (step = 0) : step++;
}

PRIVATE HAL_StatusTypeDef I2C2_Mem_Read(
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint8_t *pData,
    uint16_t Size,
    uint32_t Timeout
)
{
    if (!i2c_mem_read(I2C2, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout))
    {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL Battery_Serial_Read(UINT8 SlaveAddress, UINT8 Readaddr, UINT8 *Str,UINT16 Len, UINT8 i2c_type)
{
    HAL_StatusTypeDef ret;

    if (i2c_type)
    {
        ret = I2C2_Mem_Read(SlaveAddress, Readaddr, I2C_MEMADD_SIZE_8BIT, Str, Len, 50000);

        if (ret != HAL_OK)
        {
            HAL_I2C_Reset(I2C2);
            return 0;
        }

        return 1;
    }
    else
    {
        battery_iic_start(&sBatI2C);
        battery_iic_WriteByte(&sBatI2C, SlaveAddress);
        if(!battery_iic_waitack(&sBatI2C))
            return 0;
        battery_iic_WriteByte(&sBatI2C,Readaddr);
        if(!battery_iic_waitack(&sBatI2C))
            return 0;

        battery_iic_start(&sBatI2C);
        battery_iic_WriteByte(&sBatI2C, SlaveAddress+1);
        if(!battery_iic_waitack(&sBatI2C))
            return 0;

        while(Len)
        {
            if(!battery_iic_ReadByte(&sBatI2C, Str))
                return 0;

            if(Len==1)
            {
                battery_iic_noack(&sBatI2C);
            }
            else
            {
                battery_iic_ack(&sBatI2C);
            }

            Str++;
            Len--;
        }
        battery_iic_stop(&sBatI2C);

        return 1;
    }
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

    if (!i2c_mem_write(I2C2, SlaveAddress, Cmd, I2C_MEMADD_SIZE_8BIT, writeList, 3, 5000))
    {
        HAL_I2C_Reset(I2C2);
        return 0;
    }

    if (!i2c_mem_read(I2C2, SlaveAddress, Cmd, I2C_MEMADD_SIZE_8BIT, Str, Len, 5000))
    {
        HAL_I2C_Reset(I2C2);
        return 0;
    }

    return 1;
}

PRIVATE BOOL ManufacturerBlockAccesRead_Weak(UINT8 SlaveAddress, UINT8 Cmd, UINT16 macCode, UINT8 *Str, UINT16 Len)
{
//    UINT8 writeList[3] = {0x02, 0x00, 0x00};
//
//    writeList[1] = macCode&0xFF;
//    writeList[2] = (macCode>>8)&0xFF;
//
//    HAL_StatusTypeDef ret;
//    // ret = HAL_I2C_Mem_Write(&hi2c3, SlaveAddress, Cmd, I2C_MEMADD_SIZE_8BIT, writeList, 3, 500);
//
//    if (ret != HAL_OK)
//    {
//         HAL_I2C_Reset(&hi2c3);
//        return 0;
//    }
//
//     ret = HAL_I2C_Mem_Read(&hi2c3, SlaveAddress, Cmd, I2C_MEMADD_SIZE_8BIT, Str, Len, 1000);
//
//    if (ret != HAL_OK)
//    {
//         HAL_I2C_Reset(&hi2c3);
//        return 0;
//    }

    return 1;
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
PUBLIC void BatteryInfoReadLoop(void)
{
    //识别电池类型
    if (NONE_RECOGNIZED == sPowerManager.sBatteryInfo.BMS_icType)
    {
        PRIVATE UINT32 ReadOperationTime = 0;
        UINT8 Tmp;
        UINT8 Tmp1[11];

        const UINT8 GF_7S6P_DeviceName[6] = "PD7S6P";
        const UINT8 GF_7S8P_DeviceName[6] = "PD7S8P";

        const UINT8 GSA7S147_DeviceName[8] = "GSA7S147";
        const UINT8 GSA7S119_DeviceName[8] = "GSA7S119";
        const UINT8 GSA7S139_DeviceName[8] = "GSA7S139";
        const UINT8 GSA7S140_DeviceName[8] = "GSA7S140";
        const UINT8 GSA7S141_DeviceName[8] = "GSA7S141";
        const UINT8 DESAY7S8P_DeviceName[6]= "DESAY";

        //100ms刷新一次电池类型识别
        if (sPowerManager.uTick - ReadOperationTime <= 4)
            return;

        ReadOperationTime = sPowerManager.uTick;

        i2c_deinit(I2C2);

        battery_I2C_GPIO_Init();
        if (Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x02, &Tmp, 1, 0))
        {
            sPowerManager.sBatteryInfo.BatteryFloorLevelLimit = 10;
            sPowerManager.sBatteryInfo.BatteryTopLevelLimit = 95;
            sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel = 93;
            sPowerManager.sBatteryInfo.BatteryFullChargeTopLevel = 95;
            sPowerManager.sAlarm.ChargeCurrentMax = 5500;
            sPowerManager.sBatteryInfo.BMS_icType = TI_BQ34Z100;

            return;
        }

//        MX_I2C3_Init();
        if (Battery_Serial_Read(GSA7S_IIC_ADDR, 0x21, Tmp1, 11, 0))
        {
            sPowerManager.sBatteryInfo.BatteryFloorLevelLimit = 5;
            sPowerManager.sBatteryInfo.BatteryTopLevelLimit = 100;
            sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel = 94;
            sPowerManager.sBatteryInfo.BatteryFullChargeTopLevel = 100;

            UINT8 lenCmp = 8;

            if (!strncmp((INT8 *)&Tmp1[1], (INT8 *)GSA7S147_DeviceName, lenCmp))
            {
                sPowerManager.sAlarm.ChargeCurrentMax = 5500;
                sPowerManager.sBatteryInfo.BMS_icType = GSA7S147;
                return;
            }

            if (!strncmp((INT8 *)&Tmp1[1], (INT8 *)GSA7S119_DeviceName, lenCmp))
            {
                sPowerManager.sAlarm.ChargeCurrentMax = 7500;
                sPowerManager.sBatteryInfo.BMS_icType = GSA7S119;
                return;
            }

            if (!strncmp((INT8 *)&Tmp1[1], (INT8 *)GSA7S139_DeviceName, lenCmp))
            {
                sPowerManager.sAlarm.ChargeCurrentMax = 5500;
                sPowerManager.sBatteryInfo.BMS_icType = GSA7S139;
                return;
            }

            if (!strncmp((INT8 *)&Tmp1[1], (INT8 *)GSA7S140_DeviceName, lenCmp))
            {
                sPowerManager.sAlarm.ChargeCurrentMax = 7500;
                sPowerManager.sBatteryInfo.BMS_icType = GSA7S140;
                return;
            }

            if (!strncmp((INT8 *)&Tmp1[1], (INT8 *)GSA7S141_DeviceName, lenCmp))
            {
                sPowerManager.sAlarm.ChargeCurrentMax = 10000;
                sPowerManager.sBatteryInfo.BMS_icType = GSA7S141;
                return;
            }
        }

        if (Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x6C, Tmp1, 11, 0))
        {
            sPowerManager.sBatteryInfo.BatteryFloorLevelLimit = 5;
            sPowerManager.sBatteryInfo.BatteryTopLevelLimit = 100;
            sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel = 95;
            sPowerManager.sBatteryInfo.BatteryFullChargeTopLevel = 100;

            if (!strncmp((INT8 *)&Tmp1[1], (INT8 *)GF_7S6P_DeviceName, strlen((INT8 *)GF_7S6P_DeviceName)))
            {
                sPowerManager.sAlarm.ChargeCurrentMax = 5500;
                sPowerManager.sBatteryInfo.BMS_icType = GF_7S6P;
                return;
            }

            if (!strncmp((INT8 *)&Tmp1[1], (INT8 *)GF_7S8P_DeviceName, strlen((INT8 *)GF_7S8P_DeviceName)))
            {
                sPowerManager.sAlarm.ChargeCurrentMax = 10000;
                sPowerManager.sBatteryInfo.BMS_icType = GF_7S8P;
                return;
            }
        }
        if (Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0x6B, Tmp1, 6, 0))
        {
            sPowerManager.sBatteryInfo.BatteryFloorLevelLimit = 10;
            sPowerManager.sBatteryInfo.BatteryTopLevelLimit = 100;
            sPowerManager.sBatteryInfo.BatteryFullChargeFloorLevel = 95;
            sPowerManager.sBatteryInfo.BatteryFullChargeTopLevel = 100;

            sPowerManager.sAlarm.ChargeCurrentMax = 10000;
            sPowerManager.sBatteryInfo.BMS_icType = DESAY_7S8P;
            return;
        }
    }

    BatteryI2cReadSOC_Temp();
    BatteryI2cReadInternalInfo();
//    BatteryI2cReadLifetimeDataBlock(&sPowerManager.sBatteryInfo.sLifetimeData);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadSN(UINT32 *pSN)
{
    UINT8 ret;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:

        break;

    case TI_BQ34Z100:

        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x1C, (UINT8 *)pSN, 2, 0);
        break;

    case GF_7S6P:
    case GF_7S8P:
        break;
    case DESAY_7S8P:

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
PRIVATE BOOL BatteryReadSoc(UINT8 *pSoc)
{
    UINT8 ret = 0;
    UINT16 soc_tmp =0;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
        break;

    case TI_BQ34Z100:
        ret = Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x02, pSoc, 1, 0);
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x0D, pSoc, 1, 0);
        break;

    case GF_7S6P:
    case GF_7S8P:
        ret = Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x0D, pSoc, 1, 0);
        break;

    case DESAY_7S8P:
        ret = Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0x0D, (UINT8 *)&soc_tmp, 2, 0);
        *pSoc = soc_tmp/100;
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
PRIVATE BOOL BatteryReadTemp(INT16 *pTemp)
{
    UINT8 ret = 0;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
        break;

    case TI_BQ34Z100:
//            ret = Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x0C, (UINT8 *)pTemp, 2, 0);
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x08, (UINT8 *)pTemp, 2, 0);
        break;

    case GF_7S6P:
    case GF_7S8P:
//            ret = Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x08, (UINT8 *)pTemp, 2, 0);
        break;

    case DESAY_7S8P:
//			ret = Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0x08, (UINT8 *)pTemp, 2,0);
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
PRIVATE BOOL BatteryReadFcc(UINT32 *pFcc)
{
    UINT8 ret = 0;
    UINT32 Fcc_Temp = 0;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
        break;

    case TI_BQ34Z100:
        ret = Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x06, (UINT8 *)pFcc, 2, 0);
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x10, (UINT8 *)pFcc, 2,0);

        if (ret)
        {
            *pFcc = *pFcc *2;
        }
        break;

    case GF_7S6P:
    case GF_7S8P:
        ret = Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x10, (UINT8 *)pFcc, 2, 0);
        break;

    case DESAY_7S8P:
        ret = Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0x10, (UINT8 *)&Fcc_Temp, 2, 0);
        *pFcc = Fcc_Temp<<3;
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
PRIVATE BOOL BatteryReadRc(UINT32 *pRc)
{
    UINT8 ret = 0;
    UINT32 Rcc_Temp = 0;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
        break;

    case TI_BQ34Z100:
        ret = Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x04, (UINT8 *)pRc, 2, 0);
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x0F, (UINT8 *)pRc, 2, 0);

        if (ret)
        {
            *pRc = *pRc * 2;
        }
        break;

    case GF_7S6P:
    case GF_7S8P:
        ret = Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x0F, (UINT8 *)pRc, 2, 0);
        break;

    case DESAY_7S8P:
        ret = Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0xf, (UINT8 *)&Rcc_Temp, 2, 0);
        *pRc = Rcc_Temp<<3;
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
PRIVATE BOOL BatteryReadBv(UINT32 *pBv)
{
    UINT8 ret = 0;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
        break;

    case TI_BQ34Z100:
        ret = Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x08, (UINT8 *)pBv, 2, 0);
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x09, (UINT8 *)pBv, 2, 0);
        break;

    case GF_7S6P:
    case GF_7S8P:
        ret = Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x09, (UINT8 *)pBv, 2, 0);
        break;

    case DESAY_7S8P:
        ret = Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0x09, (UINT8 *)pBv, 2, 0);
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
PRIVATE BOOL BatteryReadSoh(UINT32 *pSoh)
{
    UINT8 ret = 0;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
        break;

    case TI_BQ34Z100:
        ret = Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x2E, (UINT8 *)pSoh, 2, 0);
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x4F, (UINT8 *)pSoh, 1, 0);

        if (ret)
        {
            *pSoh = *pSoh & 0xFF;
        }
        break;

    case GF_7S6P:
    case GF_7S8P:
        ret = Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x7C, (UINT8 *)pSoh, 1, 0);

        if (ret)
        {
            *pSoh = *pSoh & 0xFF;
        }
        break;

    case DESAY_7S8P:
        ret = Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0x70, (UINT8 *)pSoh, 2, 0);
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
PRIVATE BOOL BatteryReadCc(UINT32 *pCc)
{
    UINT8 ret = 0;

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
        break;

    case TI_BQ34Z100:
        ret = Battery_Serial_Read(TI_BQ34Z100_IIC_ADDR, 0x2C, (UINT8 *)pCc, 2, 0);
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = Battery_Serial_Read(GSA7S_IIC_ADDR, 0x17, (UINT8 *)pCc, 2, 0);
        break;

    case GF_7S6P:
    case GF_7S8P:
        ret = Battery_Serial_Read(GF_7S6P_7S8P_IIC_ADDR, 0x7B, (UINT8 *)pCc, 2, 0);
        break;

    case DESAY_7S8P:
        ret = Battery_Serial_Read(DESAY_7S5P_7S8P_IIC_ADDR, 0x17, (UINT8 *)pCc, 2, 0);
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
PRIVATE BOOL BatteryReadCellVoltage(UINT16  CellV[7])
{
    UINT8 ret;
    UINT8 ret1;
    UINT8 Tmp[35];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;


    case GSA7S147:
    case GSA7S119:
        /*Bq78350 DAstatus1*/
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x71, Tmp, 17);

        if (ret)
        {
            memcpy((UINT8 *)CellV, Tmp+3, 14);
            return 1;
        }
        break;

    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        /*Bq34Z80 DAstatus1*/
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x71, Tmp, 35);
        if (ret)
        {
            memcpy((UINT8 *)CellV, Tmp+3, 8);
        }

        /*Bq34Z80 DAstatus3*/
        ret1 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x7B, Tmp, 21);
        if (ret1)
        {
            memcpy((UINT8 *)(CellV+4), Tmp+3, 2);
            memcpy((UINT8 *)(CellV+5), Tmp+9, 2);
            memcpy((UINT8 *)(CellV+6), Tmp+15, 2);
        }

        if (ret & ret1)
        {
            return 1;
        }
        break;

    default:
        break;
    }
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadSafetyAlert(UINT32 *pSafetyAlert)
{
    UINT8 ret;

    UINT8 Tmp[7];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x50, Tmp, 7);

        if (ret)
        {
            memcpy((UINT8 *)pSafetyAlert, Tmp+3, 4);
            return 1;
        }

        break;

    default:
        break;
    }
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadSafetyStatus(UINT32 *pSafetyStatus)
{
    UINT8 ret;

    UINT8 Tmp[7];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x51, Tmp, 7);

        if (ret)
        {
            memcpy((UINT8 *)pSafetyStatus, Tmp+3, 4);
            return 1;
        }

        break;

    default:
        break;
    }
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadPFAlert(UINT32 *pPFAlert)
{
    UINT8 ret;

    UINT8 Tmp[7];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x52, Tmp, 7);

        if (ret)
        {
            memcpy((UINT8 *)pPFAlert, Tmp+3, 4);
            return 1;
        }

        break;

    default:
        break;
    }
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadPFStatus(UINT32 *pPFStatus)
{
    UINT8 ret;

    UINT8 Tmp[7];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x53, Tmp, 7);

        if (ret)
        {
            memcpy((UINT8 *)pPFStatus, Tmp+3, 4);
            return 1;
        }

        break;

    default:
        break;
    }
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadOperationStatus(UINT32 *pOperationStatus)
{
    UINT8 ret;

    UINT8 Tmp[7];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x54, Tmp, 7);

        if (ret)
        {
            memcpy((UINT8 *)pOperationStatus, Tmp+3, 4);
            return 1;
        }

        break;

    default:
        break;
    }
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadChargingStatus(UINT32 *pChargingStatus)
{
    UINT8 ret;

    UINT8 Tmp[7];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;

    case GSA7S147:
    case GSA7S119:
    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x55, Tmp, 7);

        if (ret)
        {
            memcpy((UINT8 *)pChargingStatus, Tmp+3, 3);
            return 1;
        }

        break;

    default:
        break;
    }
    return 0;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE BOOL BatteryReadLifetimeData(struct BatteryLifeTimeDataBlock *pData)
{
    UINT8 ret_lifetimeDataBlock1;
    UINT8 ret_lifetimeDataBlock2;
    UINT8 ret_lifetimeDataBlock3;
    UINT8 ret_lifetimeDataBlock4;
    UINT8 ret_lifetimeDataBlock5;
    UINT8 ret_lifetimeDataBlock6;
    UINT8 ret_lifetimeDataBlock7;

    UINT8 Tmp[35];

    switch(sPowerManager.sBatteryInfo.BMS_icType)
    {
    case NONE_RECOGNIZED:
    case TI_BQ34Z100:
    case GF_7S6P:
    case GF_7S8P:

    case DESAY_7S8P:
        return 0;
        break;

    case GSA7S147:
    case GSA7S119:
        ret_lifetimeDataBlock1 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x60, Tmp, 17);
        if (ret_lifetimeDataBlock1)
        {
            memcpy((UINT8 *)(&(pData->MaxCellVoltage1)), Tmp+3, 14);
        }

        ret_lifetimeDataBlock2 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x61, Tmp, 17);
        if (ret_lifetimeDataBlock2)
        {
            memcpy((UINT8 *)(&(pData->MinCellVoltage1)), Tmp+3, 14);
        }

        ret_lifetimeDataBlock3 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x62, Tmp, 17);
        if (ret_lifetimeDataBlock3)
        {
            memcpy((UINT8 *)(&(pData->MaxDeltaCellVoltage)), Tmp+3, 13);
            memcpy((UINT8 *)(&(pData->MaxTempFET)), Tmp+163, 1);
        }

        ret_lifetimeDataBlock4 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x63, Tmp, 4);
        if (ret_lifetimeDataBlock4)
        {
            memcpy((UINT8 *)(&(pData->NoOfShutdowns)), Tmp+3, 1);
        }

        ret_lifetimeDataBlock5 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x64, Tmp, 5);
        if (ret_lifetimeDataBlock5)
        {
            memcpy((UINT8 *)(&(pData->TotalFWRuntime)), Tmp+3, 2);
        }

        ret_lifetimeDataBlock6 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x65, Tmp, 35);
        if (ret_lifetimeDataBlock6)
        {
            memcpy((UINT8 *)(&(pData->NoOfCOVEvents)), Tmp+3, 12);
            memcpy((UINT8 *)(&(pData->NoOfOCC1Events)), Tmp+15, 4);
            memcpy((UINT8 *)(&(pData->NoOfOAOLDEvents)), Tmp+19, 16);
        }

        ret_lifetimeDataBlock7 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x66, Tmp, 17);
        if (ret_lifetimeDataBlock7)
        {
            memcpy((UINT8 *)(&(pData->NoOfOTFEvents)), Tmp+3, 14);
        }

        if (ret_lifetimeDataBlock1 && ret_lifetimeDataBlock2 && ret_lifetimeDataBlock3 && ret_lifetimeDataBlock4 && ret_lifetimeDataBlock5 && ret_lifetimeDataBlock6 && ret_lifetimeDataBlock7)
        {
            return 1;
        }
        break;

    case GSA7S139:
    case GSA7S140:
    case GSA7S141:
        ret_lifetimeDataBlock1 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x60, Tmp, 33);
        if (ret_lifetimeDataBlock1)
        {
            memcpy((UINT8 *)(&(pData->MaxCellVoltage1)), Tmp+3, 30);
        }

        ret_lifetimeDataBlock2 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x61, Tmp, 28);
        if (ret_lifetimeDataBlock2)
        {
            memcpy((UINT8 *)(&(pData->MaxChargeCurrent)), Tmp+3, 18);
        }

        ret_lifetimeDataBlock3 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x62, Tmp, 19);
        if (ret_lifetimeDataBlock3)
        {
            memcpy((UINT8 *)(&(pData->TotalFWRuntime)), Tmp+3, 2);
        }

        ret_lifetimeDataBlock4 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x63, Tmp, 35);
        if (ret_lifetimeDataBlock4)
        {
            memcpy((UINT8 *)(&(pData->NoOfCOVEvents)), Tmp+3, 32);
        }

        ret_lifetimeDataBlock5 = ManufacturerBlockAccesRead(GSA7S_IIC_ADDR, 0x44, 0x64, Tmp, 35);
        if (ret_lifetimeDataBlock5)
        {
            memcpy((UINT8 *)(&(pData->NoOfASCCEvents)), Tmp+3, 32);
        }

        if (ret_lifetimeDataBlock1 && ret_lifetimeDataBlock2 && ret_lifetimeDataBlock3 && ret_lifetimeDataBlock4 && ret_lifetimeDataBlock5)
        {
            return 1;
        }

        break;

    default:
        break;
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
#define ADC0_JDR0_GAIN  0.01859225f    // Dc Voltage coff MT_BUS
PRIVATE float GetVbusVoltage(void)
{
    float temp = 0;
    temp = ADC_IDATA0(ADC0)*ADC0_JDR0_GAIN;
    return temp;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void SpeakerOnExec(void)
{
    static UINT16 S_DelayCnt = 0;
	  static UINT8  S_SpeakState = 0;
	
	switch (S_SpeakState)
	{
		case 0:
		{
			S_DelayCnt++;
			//上电后30S才打开音推使能，避免音响开机爆音问题
			if (S_DelayCnt < 40)
			{
					;
			}
			else if(S_DelayCnt < 2*40)
			{
					MusicPwEnable();
			}
			else if(S_DelayCnt < 3*40)
			{
					MuteDisable();
			}
			else if(S_DelayCnt >= 30*40)
			{
					S_DelayCnt = 0;
					MusicPwEnable();
					MuteEnable();
				  sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower = ON;
				  S_SpeakState = 1;
			} 
		}
		break;
		
		case 1:
		{
		   	if(sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower != sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower)
				{					
					if(sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower)
					{
							 MusicPwEnable();
						   S_DelayCnt++;
						   if(S_DelayCnt >= 800)
							 {
							    MuteEnable(); 
							    MutePowerAnswer(0x01);
								  sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower;
							 }
					}
					else
					{
						   S_DelayCnt = 0;
							 MuteDisable();
							 MusicPwDisable();		
							 MutePowerAnswer(0x02);
               sPowerManager.sBoardPowerInfo.PowerOnState.PowerOnOffReg.bit.SpeakerPower = sPowerManager.sBoardPowerInfo.PowerOnConfig.PowerOnOffReg.bit.SpeakerPower;						
					}
				}
		}
		break;
	}
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
//    if (!ApplicationMode)
//    {
//        VbusEnable();
//        delay_ms(2);
//        DrvPwEnable();
//        SetVbusPower(1);
//
//        return;
//    }

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

        AdcSample0Start();
        AdcSample0ClearFlag();

        Vbus_tmp_last = Vbus_tmp;
        Vbus_tmp =   GetVbusVoltage();
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

