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

#define BATTERY_NUMBERS   2

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
    POWEROFF = 6,
    POWEROFF_LOW = 7
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
    UINT8                     disinfectionModulePowerCmdSet;
};



/*Alarm Data Type Definite*/
#define ALARM_LOW_POWER_MASK        (~(1<<8))
#define ALARM_COM_DISCONNECT_MASK   (~(1<<0))

struct PowerAlarmRegBits   // bits   description
{
    UINT32 ComFail:1;              //i2c disconnect
    UINT32 BatteryVoltageOver:1; 
    UINT32 ChargerVoltageOver:1;         
    UINT32 ChargeCurrentOVer:1;
    UINT32 ChargeTimeOver:1;            //Real charge time far away from Expected Charge time
    UINT32 CellTempOver:1;
    UINT32 VoltageElectricityMismatch:1;
    UINT32 ChargerVoltageUnder:1;       //充电器欠压
    UINT32 BatteryLowPower:1;           //电池电量过低
    UINT32 Rsv:24;
};

union PowerAlarmReg
{
    UINT32                      all;
    struct PowerAlarmRegBits    bit;
};

struct PowerAlarmStruct
{
    union PowerAlarmReg PowerAlarmReg;

    UINT16 ComFailCombinedSet;
    UINT16 ComFailCntMax;

    UINT16  BatteryVoltageMax;
    UINT16  BoardVoltageOverCnt;
    UINT16  BatteryVolateOverCnt[BATTERY_NUMBERS];
    UINT16  BatteryVolateOverCntMax;

    UINT16  ChargerVoltageMax;
    UINT16  ChargerVolateOverCnt;
    UINT16  ChargerVolateOverCntMax;

    INT16   ChargeCurrentMax;
    UINT16  ChargeCurrentOverCnt;
    UINT16  ChargeCurrentOverCntMax;

    INT16   CellTempMax;
    UINT16  CellTempOverCnt[BATTERY_NUMBERS];
    UINT16  CellTempOverCntMax;

    UINT16  LowPowerSoc;
    UINT16  LowPowerCntMax;
    UINT8   LowPowerCombinedSet;
}; 

/*Power manage State Machine*/
enum PowerManageState
{
    POWER_MANAGE_INIT = 0,
    POWER_MANAGE_CHARGE_KEY_EVENT_DETECT = 1,
    POWER_MANAGE_ALARM_DETECT = 2,
    POWER_MANAGE_POWER_ON_OFF_CHARGE_EXEC = 3,
    POWER_MANAGE_STATE_CLEAR = 4
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
    UINT8 charge_over_cnt;
};

/*Battery manage System IC type*/
enum BatteryManageSystemType
{
    NONE_RECOGNIZED = 0, 
    GF_7S6P_0x18 = 7,
    GF_7S8P_0x18 = 8,
    
    GF_7S6P_0x20 = 39,
    GF_7S8P_0x20 = 40,    
};


/*Battery Info Data Type Definite*/
#pragma pack (1)
struct BatteryInfoStruct
{
    enum BatteryManageSystemType    BMS_icType;
    UINT16                          Voltage;
    INT16                           DischargeCurrent;
    INT16                           ChargeCurrent;
    UINT8                           SocRaw;
    INT16                           CellTemp;
    UINT16                          FullCapacity;
    UINT16                          SOH;
    UINT16                          CycleCnt;
    UINT16                          CellVoltage[7];
    UINT32                          Protect;
    UINT32                          Fault;
    UINT8                           ComFailSet;
    UINT8                           LowPowerSet;
    
    UINT8                           id;
    UINT8                           Addr;
    UINT16                          SN;

    INT16                           MosTemp;
    
    UINT8                           SocOptimized;
    UINT8                           FloorSocLimit;
    UINT8                           TopSocLimit;
    UINT8                           FullChargeFloorSoc;
    UINT8                           FullChargeTopSoc;
    UINT16                          RemaingCapacity;

    UINT8                           SocBuffer;
    UINT8                           SocDebounceCnt;
    
    UINT16                          LowPowerCnt;
    UINT16                          ComFailCnt;
};
#pragma pack ()

/*Board Power On or Off Manage Data Type Definite*/
struct BoardPowerOnOffStruct
{
    UINT8                           LowPowerConsumeMode;
    UINT8                           PoweroffUploadAckFlag;
    UINT8                           VbusSoftStartFlag;  //0:Unknow 1:success 2:fail
    UINT8                           VbusSoftStartEn;    //0:Disable 1:enable
    UINT16                          ParallelVoltage;

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
    UINT8                           DevSocOptimized;
    
    struct BatteryInfoStruct        sBatteryInfo[BATTERY_NUMBERS];
    struct ChargeManageStruct       sChargeInfo;
    struct BoardPowerOnOffStruct    sBoardPowerInfo;
    struct PowerAlarmStruct         sAlarm;
};

struct BatteryObjectEntryStruct
{
    UINT16                       Index;      /**< \brief Object index*/
    UINT16                       TimeGap;    /** \brief time(s) between two frames*/
    UINT16                       BitLength;  /**< \brief Entry bit size*/
    void                         *pVarPtr;   /**< \brief Pointer to object buffer*/
};

PUBLIC void PowerManagerInit(UINT8 AppMode);
PUBLIC void PowerManagerExec(void);
PUBLIC void BatteryParamLoop(void);
PUBLIC void LdsPowerCtrl(UINT8 En, UINT8 Speed);
PUBLIC UINT8 ReadChargeAppState(void);
PUBLIC void FrameHeader0xB0Parser(UINT8 data[8]);
PUBLIC void SetVbusPower(UINT8 State);
PUBLIC void VbusSoftStartNoBlock(REAL32 VbusVoltage);
PUBLIC void DisinfectionModulePowerCtrl(UINT8 En);
PUBLIC void lidarPowerOnOffExec(void);
#endif // _POWER_MANAGER_H_
