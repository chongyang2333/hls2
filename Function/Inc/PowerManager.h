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
 * History:
------------------------------------------------------------------------
16-11-2019 Version 2.00 : Created by DengZhuo
----------------------------------------------------------------------*/

#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include "UserDataTypes.h"

enum KeyEvent
{
    NULL_KEY_EVENT,
    LONG_PRESS,     //2s
    LONGLONG_PRESS, //10s
    SHORT_PRESS,
    RELEASED
};

enum KeyState
{
    KEY_UP,
    KEY_DOWN
};

enum HeartState
{
    OFFLINE,
    ONLINE
};

enum BatteryCoverState
{
    COVERED,
    NOT_COVERED
};

enum RstState
{
    NULL_RST,
    SOFT_RST,
    POWERON_RST
};

enum ChargerEvent
{
    NULL_CHARGER_EVENT=0,
    PULL_OUT = 1,
    INSERT_IN = 2
};

enum ConnectState
{
    DISCONNECT = 0,
    CONNECT = 1
};

enum CheckState
{
    CHECK_APPENDING = 0,
    CHECK_START = 1,
    CHECK_FAILED = 2,
    CHECK_APPROVED = 3
};

struct CheckStruct
{
    enum CheckState eCheckState;
    UINT32 CheckStartTime;
};


enum MosfetState
{
    MOS_OFF,
    MOS_ON
};

enum ChargeManageState
{
    NOT_CHARGED = 0,
    CHECK_BEFORE_CHARGING = 1,
    CHARGING = 2,
    POWER_ALARM =3
};

enum PowerOnOffManageState
{
    POWERON_INIT = 0,
    NORMAL_POWERON = 1,
    PAD_POWEROFF = 2,
    POWEROFF_UPLOAD = 3,
    POWEROFF_UPLOAD_ACKED = 4,
    POWEROFF_UPLOAD_ACK_TIMEOUT = 5,
    POWEROFF = 6
};


/*Power On or Off Config Data Type*/
struct PowerOnCfgBits   // bits   description
{
    UINT16 ChassisPower:1;
    UINT16 PadPower:1;
    UINT16 SpeakerPower:1;
    UINT16 LidarPower:1;
    UINT16 LedPower:1;
    UINT16 VbusPower:1;
    UINT16 DisinfectionModulePower:1;
    UINT16 Rsv:9;
};

union PowerOnCfgReg
{
    UINT16                    all;
    struct PowerOnCfgBits     bit;
};

struct PowerOnCfgStruct
{
    union PowerOnCfgReg       PowerOnOffReg;
    UINT8                     lidarSpeed;
    UINT8                     lidarPowerCmdSet;
};


/*Alarm Data Type Definite*/
#define ALARM_LOW_POWER_MASK        (~(1<<8))
#define ALARM_COM_DISCONNECT_MASK   (~(1<<0))

struct PowerAlarmRegBits   // bits   description
{
    UINT32 ComDisconnet:1;              //i2c disconnect
    UINT32 BatteryVoltageOver:1; 
    UINT32 ChargerVoltageOver:1;         
    UINT32 ChargeCurrentOVer:1;
    UINT32 ChargeTimeOver:1;            //Real charge time far away from Expected Charge time
    UINT32 ChargeTempOver:1;
    UINT32 VoltageElectricityMismatch:1;
    UINT32 ChargerVoltageUnder:1;       //充电器欠压
    UINT32 BatteryLowPower:1;           //电池电量过低
	  UINT32 ChargeTempUnder:1;           //电池充电低温
    UINT32 Rsv:22;
};

union PowerAlarmReg
{
    UINT32                      all;
    struct PowerAlarmRegBits    bit;
};

struct PowerAlarmStruct
{
    union PowerAlarmReg PowerAlarmReg;

    UINT16 ComDisconnectCnt;
    UINT16 ComDisconnectCntMax;

    UINT16  BatteryVoltageMax;
    UINT16  BatteryVolateOverCnt;
    UINT16  BatteryVolateOverCntMax;

    UINT16  ChargerVoltageMax;
    UINT16  ChargerVolateOverCnt;
    UINT16  ChargerVolateOverCntMax;

    INT16  ChargeCurrentMax;
    UINT16  ChargeCurrentOverCnt;
    UINT16  ChargeCurrentOverCntMax;

    INT16  ChargeTempMax;
    INT16  ChargeTempMin;
    UINT16  ChargeTempOverCnt;
    UINT16  ChargeTempOverCntMax;
		UINT16  ChargeTempUnderCnt;
		UINT16  ChargeTempUnderCntMax;

    UINT16  BatteryLowPowerLevel;
    UINT16  BatteryLowPowerCnt;
    UINT16  BatteryLowPowerCntMax;
}; 

/*Power manage State Machine*/
enum PowerManageState
{
    POWER_MANAGE_INIT = 0,
    POWER_MANAGE_CHARGE_KEY_EVENT_DETECT = 1,
    POWER_MANAGE_ALARM_DETECT = 2,
    POWER_MANAGE_POWER_ON_OFF_CHARGE_EXEC = 3,
    POWER_MANAGE_INFO_UPLOAD = 4
};

/*Charge Manage Data Type Definite*/
struct ChargeManageStruct
{
    INT16                           ChargeCurrent;
    INT16                           ChargeCurrentBias;
    UINT8                           ChargeCurrentBiasCorrectedFlag;
    UINT16                          ChargeVoltage;
    INT16                           ChargerVoltageDecDelta;

    UINT16                          ChargerConnectedVoltageThreshold;
    INT16                           ChargerConnectedCurrentThreshold;
    UINT8                           ChargerConnectJudgeTime;
    UINT8                           ChargerDisconnectJudgeTime;

    enum ChargerEvent               eChargerEvent;
    enum ConnectState               eChargerConnectState;
    struct CheckStruct              sChargerCheck;
    enum MosfetState                eChargeMosState;
    enum ChargeManageState          ChargeAppState;
	
		UINT8                           ChargeMode; // 0无  1：手动   2：充电桩
};

/*Battery manage System IC type*/
enum BatteryManageSystemType
{
    NONE_RECOGNIZED = 0,
    TI_BQ34Z100 = 1,
    GSA7S147 = 2,
    GSA7S119 = 3,
    GSA7S139 = 4,
    GSA7S140 = 5,
    GSA7S141 = 6,    
    GF_7S6P = 7,
    GF_7S8P = 8
};

struct BatteryLifeTimeDataBlock
{
    UINT16                          MaxCellVoltage1;
    UINT16                          MaxCellVoltage2;
    UINT16                          MaxCellVoltage3;
    UINT16                          MaxCellVoltage4;
    UINT16                          MaxCellVoltage5;
    UINT16                          MaxCellVoltage6;
    UINT16                          MaxCellVoltage7;
    UINT16                          MinCellVoltage1;
    UINT16                          MinCellVoltage2;
    UINT16                          MinCellVoltage3;
    UINT16                          MinCellVoltage4;
    UINT16                          MinCellVoltage5;
    UINT16                          MinCellVoltage6;
    UINT16                          MinCellVoltage7;
    UINT16                          MaxDeltaCellVoltage;
    UINT16                          MaxChargeCurrent;
    UINT16                          MaxDischargeCurrent;
    UINT16                          MaxAvgDsgCurrent;
    UINT16                          MaxAvgDsgPower;
    INT8                            MaxTempCell;
    INT8                            MinTempCell;
    INT8                            MaxDeltaCellTemperature;
    INT8                            MaxTempIntSensor;
    INT8                            MinTempIntSensor;
    INT8                            MaxTempFET;
    UINT8                           NoOfShutdowns;
    UINT8                           NoOfPartialResets;
    UINT8                           NoOfFullResets;
    UINT8                           NoOfWDTResets;
    UINT16                          TotalFWRuntime; /*Unit: Hours*/
    UINT16                          NoOfCOVEvents;  /*COV: Cell OverVoltage*/ /*all count*/
    UINT16                          LastCOVEvents;                            /*the cycle count on last events*/
    UINT16                          NoOfCUVEvents;  /*CUV: cell UnderVoltage*/
    UINT16                          LastCUVEvents;  
    UINT16                          NoOfOCD1Events; /*OCD1: overcurrent in discharge 1st tier*/ 
    UINT16                          LastOCD1Events;  
    UINT16                          NoOfOCD2Events; /*OCD2: overcurrent in discharge 2st tier*/
    UINT16                          LastOCD2Events;  
    UINT16                          NoOfOCC1Events; /*OCC1: overcurrent in charge 1st tier*/
    UINT16                          LastOCC1Events;  
    UINT16                          NoOfOCC2Events;  /*OCC2: Overcurrent in charge 2nd tier*/
    UINT16                          LastOCC2Events; 
    UINT16                          NoOfOAOLDEvents; /*AOLD: Overload in discharge latch*/
    UINT16                          LastAOLDEvents;  
    UINT16                          NoOfASCDEvents; /*ASCD: Short circuit in discharge latch*/
    UINT16                          LastASCDEvents;  
    UINT16                          NoOfASCCEvents; /*ASCC: short circuit in charge latch*/
    UINT16                          LastASCCEvents;  
    UINT16                          NoOfOTCEvents;  /*OTC: over temperature in charge*/
    UINT16                          LastOTCEvents;  
    UINT16                          NoOfOTDEvents;  /*OTD: over temperature in discharge*/
    UINT16                          LastOTDEvents;  
    UINT16                          NoOfOTFEvents;  /*OTF: over temperature in FET*/
    UINT16                          LastOTFEvents;
    UINT16                          LastValidChargeTerm;
    UINT16                          NoOfValidChargeTerm;
    UINT16                          NoOfQmaxUpdates;
    UINT16                          LastQmaxUpdates; 
    UINT16                          NoOfRaUpdates;
    UINT16                          LastRaUpdates;  
    UINT16                          NoOfRaDisable;
    UINT16                          LastRaDisable; 
};

/*Battery Info Data Type Definite*/
struct BatteryInfoStruct
{
    enum BatteryManageSystemType    BMS_icType;
    UINT16                          BatterySN;
    INT16                           BatteryTemp;
    UINT16                          BatteryCurrent;
    UINT16                          BatteryVoltage;
    UINT8                           BatteryLevelRaw;
    UINT8                           BatteryLevelOptimized;
    UINT8                           BatteryFloorLevelLimit;
    UINT8                           BatteryTopLevelLimit;
    UINT8                           BatteryFullChargeFloorLevel;
    UINT8                           BatteryFullChargeTopLevel;
    UINT8                           BatteryLowPowerLevelReadCnt;
    UINT16                          BatteryFullCapacity;
    UINT16                          BatteryRemaingCapacity;
    UINT16                          BatteryVoltageIIC;
    UINT16                          SOH;
    UINT16                          CycleCnt;
    UINT16                          CellVoltage[7];
    UINT32                          SafetyAlert;
    UINT32                          SafetyStatus;
    UINT32                          PFAlert;
    UINT32                          PFStatus;
    UINT32                          OperationStatus;
    UINT32                          ChargingStatus;
    struct BatteryLifeTimeDataBlock sLifetimeData;
};

/*Board Power On or Off Manage Data Type Definite*/
struct BoardPowerOnOffStruct
{
    UINT8                           LowPowerConsumeMode;
    UINT8                           PoweroffUploadAckFlag;
    UINT8                           VbusSoftStartFlag;  //0:Unknow 1:success 2:fail
    UINT8                           VbusSoftStartEn;    //0:Disable 1:enable    
    
    enum PowerOnOffManageState      PowerOnOnffAppState;
    struct PowerOnCfgStruct         PowerOnConfig;
    struct PowerOnCfgStruct         PowerOnState;
};

/*Power Manage Data Type Definite*/
struct PowerManagerStruct
{
    UINT32                          uTick;
    UINT32                          uPowerOffKeyPressTimestamp;
    UINT32                          uPoweroffCmdFeedbackTimestamp;

    enum PowerManageState           ePMState;
    enum RstState                   eSysRstState;
    enum KeyEvent                   ePowerKeyEvent;
    enum KeyState                   ePowerKeyState;
    enum KeyState                   ePowerKeyStateLast;
    enum HeartState                 eRk3399HeartState;
    enum BatteryCoverState          eBatteryCoverState;
    UINT8                           BatteryCoverLeadtoPoweroffSet;
    
    struct BatteryInfoStruct        sBatteryInfo;
    struct ChargeManageStruct       sChargeInfo;
    struct BoardPowerOnOffStruct    sBoardPowerInfo;
    struct PowerAlarmStruct         sAlarm;
};

PUBLIC void PowerManagerInit(UINT8 AppMode);
PUBLIC void PowerManagerExec(void);
PUBLIC void BatteryInfoReadLoop(void);
PUBLIC void LdsPowerCtrl(UINT8 En, UINT8 Speed);
PUBLIC UINT8 ReadChargeAppState(void);
PUBLIC void FrameHeader0xB0Parser(UINT8 data[8]);
PUBLIC void DisinfectionModulePowerCtrl(UINT8 En);
PUBLIC void SetVbusPower(UINT8 State);
PUBLIC void VbusSoftStartNoBlock(REAL32 VbusAdcValue);
PUBLIC void lidarPowerOnOffExec(void);
#endif // _POWER_MANAGER_H_
