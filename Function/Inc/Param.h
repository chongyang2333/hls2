/*******************************************************************
 *
 * FILE NAME:  Param.h
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2018.11.12
 *
 * AUTHOR:      Denny
 *
 * History:
------------------------------------------------------------------------
12-11-2018 Version 1.00 : Created by Denny
----------------------------------------------------------------------*/

#ifndef _PARAM_H_
#define _PARAM_H_

#include "UserDataTypes.h"
#include "ControlRun.h"
#include "gd32f4xx.h"


/*---------------------------------------------
-    DataTypes
-----------------------------------------------*/
#define     DEFTYPE_NULL                0x0000 /**< \brief Null*/
#define     DEFTYPE_BOOLEAN             0x0001 /**< \brief BOOLEAN*/
#define     DEFTYPE_INTEGER8            0x0002 /**< \brief INTEGER8*/
#define     DEFTYPE_INTEGER16           0x0003 /**< \brief INTEGER16*/
#define     DEFTYPE_INTEGER32           0x0004 /**< \brief INTEGER32*/
#define     DEFTYPE_UNSIGNED8           0x0005 /**< \brief UNSIGNED8*/
#define     DEFTYPE_UNSIGNED16          0x0006 /**< \brief UNSIGNED16*/
#define     DEFTYPE_UNSIGNED32          0x0007 /**< \brief UNSIGNED32*/
#define     DEFTYPE_REAL32              0x0008 /**< \brief REAL32*/
#define     DEFTYPE_VISIBLESTRING       0x0009 /**< \brief VISIBLE_STRING*/
#define     DEFTYPE_OCTETSTRING         0x000A /**< \brief OCTET_STRING*/
#define     DEFTYPE_UNICODE_STRING      0x000B /**< \brief UNICODE_STRING*/
#define     DEFTYPE_TIME_OF_DAY         0x000C /**< \brief TIME_OF_DAY*/
#define     DEFTYPE_TIME_DIFFERENCE     0x000D /**< \brief TIME_DIFFERENCE*/
#define     DEFTYPE_INTEGER24           0x0010 /**< \brief INTEGER24*/
#define     DEFTYPE_REAL64              0x0011 /**< \brief REAL64*/
#define     DEFTYPE_INTEGER40           0x0012 /**< \brief INTEGER40*/
#define     DEFTYPE_INTEGER48           0x0013 /**< \brief INTEGER48*/
#define     DEFTYPE_INTEGER56           0x0014 /**< \brief INTEGER56*/
#define     DEFTYPE_INTEGER64           0x0015 /**< \brief INTEGER64*/
#define     DEFTYPE_UNSIGNED24          0x0016 /**< \brief UNSIGNED24*/
#define     DEFTYPE_UNSIGNED40          0x0018 /**< \brief UNSIGNED40*/
#define     DEFTYPE_UNSIGNED48          0x0019 /**< \brief UNSIGNED48*/
#define     DEFTYPE_UNSIGNED56          0x001A /**< \brief UNSIGNED56*/
#define     DEFTYPE_UNSIGNED64          0x001B /**< \brief UNSIGNED64*/
#define     DEFTYPE_GUID                0x001D /**< \brief DEFTYPE_GUID*/
#define     DEFTYPE_BYTE                0x001E /**< \brief DEFTYPE_BYTE*/
#define     DEFTYPE_WORD                0x001F /**< \brief DEFTYPE_WORD*/
#define     DEFTYPE_DWORD               0x0020 /**< \brief DEFTYPE_DWORD*/
#define     DEFTYPE_PDOMAPPING          0x0021 /**< \brief PDO_MAPPING*/
#define     DEFTYPE_IDENTITY            0x0023 /**< \brief IDENTITY*/
#define     DEFTYPE_COMMAND             0x0025 /**< \brief COMMAND_PAR*/
#define     DEFTYPE_PDOCOMPAR           0x0027 /**< \brief PDO_PARAMETER*/
#define     DEFTYPE_ENUM                0x0028 /**< \brief DEFTYPE_ENUM */
#define     DEFTYPE_SMPAR               0x0029 /**< \brief */

#define 		EN_RESET_TYPE_OTHER					0x00
#define 		EN_RESET_TYPE_SOFT					0x01
#define 		EN_RESET_TYPE_PORRSTF				0x02


#define 		EN_RESET_TYPE_BKP_ADDR			0U
#define         EN_BATTERY_PROTECT_BKP_ADDR     4U

/* Software Version */
struct SoftwareVersionStruct
{
		UINT8  majorVer;
		UINT8  minorVer;
		UINT8  patchVer;
};

/*SN Data Type Definite*/
struct SNDataStruct
{
    UINT8 CompareFlag;
	  UINT8 RWFlag;
    UINT8 SNDataRead[32];
	  UINT8 SNDataWrite[32];
};

/* MachineInfo */
#define MACHINE_INFO_NO_CMD   0
#define MACHINE_INFO_WRITING  1
#define MACHINE_INFO_SUCCESS  2
#define MACHINE_INFO_FAULT    3

typedef struct
{
    UINT32	year;
    UINT32	month;
    UINT32	day;
}PcbProductionDate;

/*
	Motor info In MachineInfo 
*/
struct MotorDataInMachineInfoStruct
{
    REAL32  machineWheelDiameter;  
    REAL32	machineWheelPerimeter;  
    REAL32  machineWheelbase;
    REAL32 	encoderPulsePerCircle;
    REAL32  encoderSampleTimesPerPulse;
    REAL32	reductionRatio;
    UINT32	isEncoderCountInv;
};

/*
	machine info struct parameters 
*/
struct MachineInfoStruct
{
    REAL32  machineWheelDiameter;  
    REAL32	machineWheelPerimeter;  
    REAL32  machineWheelbase;
    REAL32 	encoderPulsePerCircle;
    REAL32  encoderSampleTimesPerPulse;
    REAL32	reductionRatio;
    UINT32	isEncoderCountInv;
    UINT32 	uwbTagPcbMajorVersion;
    UINT32 	uwbTagPcbMinorVersion;
    UINT32 	chassisPcbMajorVersion;
    UINT32 	chassisPcbMinorVersion;
    UINT32 	infraredSensorVersion;
    UINT32	ldsSensorVersion;
    UINT32	motorVersion;
    UINT32	weighSensorVersion;
    UINT32	batteryVersion;
    PcbProductionDate   uwbTagPcbMPD;
    PcbProductionDate	chassisPcbMPD;
    /*INSERT->20190503*/
    UINT32  makerCameraVersion;
    /*INSERT->20190628*/
    UINT8   ESP32Version;
    /*INSERT->20190716*/
    UINT8   RGBDVersion;
    /*INSERT->2019*/
    UINT8   type; //machine type 6:plus,8:bella
    UINT8   AudioVersion;  // Index 23 第四个字节
    
    UINT8   LoRaVersion; // Index 24
    UINT8   BarCodeScanningGunVersion;
    UINT8   FaceDetectCameraVersion;
    UINT8   SlamwareVersion;
    
    UINT32  Odometer;//Index 25
    UINT32  CumulativeTime; //Index 26
    
    /*INSERT->2020.10.19 Jordan*/
    UINT16  MachineModel; // Index 27
    UINT8   MachineMainVersion;
    UINT8   MachineMinorVersion;
    
    /*INSERT->2021.08.31 */
    UINT8  NpuTpye;   // Index 28
    UINT8  Matrixmic; 
    UINT8  HostCoreBoardType;
    UINT8  HeadType;
    
    UINT8  LidarCommunicateType;  // Index 29
    UINT8  LidarType;
    UINT8  Version4G;
    UINT8  CarbinDoorMotorType;
    
    /*INSERT->2021.10.25 */
    UINT8  TrayBoardType;  // Index 30 
    UINT8  FunctionBoardType;
    UINT8  ChassisBoardType;
    UINT8  DoorBoardType;
    
    /*INSERT->2021.11.23 */
    UINT8 LaserProjection;
    UINT8 VedioOutput;
    UINT8 DistributionAreaLight;
    UINT8 res31_1;
    
    UINT8 PowerBoardType;
    UINT8 res32_1;
    UINT8 res32_2;
    UINT8 res32_3;
    
    UINT32  rsv[40];
    
    UINT16  EepromCRC;
    UINT16  CrcState;
    
    UINT16  RestoreDefault;
    UINT16  SaveMachineInfo;
    UINT16  MachineInfoSaveState;
    UINT32  MotorVersionLast;   
};

/*
	sensor data struct 
*/
struct SensorDataStruct
{ 
    INT16  GyroSpeedX0x4000;
    INT16  GyroSpeedY0x4001;
    INT16  GyroSpeedZ0x4002;
    INT16  GyroAccelX0x4003;
    INT16  GyroAccelY0x4004;
    INT16  GyroAccelZ0x4005;
    
    UINT8  InfraRed10x4006;
    UINT8  InfraRed20x4007;
    UINT8  InfraRed30x4008;
    UINT8  InfraRed40x4009;
    
    UINT16  BatteryVoltage0x400A;
    UINT16  BatteryCurrent0x400B;
    INT16  ChargeCurrent0x400C;
    UINT16  BatteryLevel0x400D;
    UINT16  ChargeState0x400E;
    UINT16  ChargeVoltage0x400F;
    INT16   BatteryTemp0x4010;
    INT16 AverageBatteryCurrent0x4011;
};


struct ParameterStruct 
{ 
    INT32   PositionLimitMin0x2000;
    INT32   PositionLimitMax0x2001;
    UINT16  SpeedLimit0x2002;
    UINT16  CurrentLimit0x2003;
    UINT16  VdcLimitMax0x2004;
    UINT16  VdcLimitMin0x2005;
    UINT16  TempLimitMax0x2006;
    UINT32  AlarmSwitch0x2007;
    UINT32  SpeedFollowErrWindow0x2008;
    UINT32  SpeedFollowErrTime0x2009;
    UINT16  CanLostErrTime0x200A;
    
    UINT16  PositionLoopKp0x2100;
    UINT16  VelocityLoopKp0x2101;
    UINT16  VelocityLoopKi0x2102;
    UINT16  CurrentLoopKp0x2103;
    UINT16  CurrentLoopKi0x2104;
    UINT16  FilterConfig0x2105;
    UINT16  PosRefFilterFc0x2106;
    UINT16  SpdRefFilterFc0x2107;
    UINT16  SpdFdbFilterFc0x2108;
    UINT16  CurRefFilterFc0x2109;
    UINT16  TorffFilterFc0x210A;
    UINT16  TorffGain0x210B;
    UINT16  DisturFilterFc0x210C;
    UINT16  DisturGainFc0x210D;
    
    UINT16  MotorMaxSpeed0x2200;
    UINT16  MotorPolePairs0x2201;
    UINT32  EncoderPPR0x2202;
    UINT16  HallEnable0x2203;
    UINT16  Commutation0x2204;
    UINT16  CommutationCurrent0x2205;
    UINT16  CommutationStableTime0x2206;
    UINT16  CommutationOffset0x2207;
    UINT16  MotorInverse0x2208;
    UINT16  MotorRatedCurrent0x2209;
    UINT16  MotorPeakCurrent0x220A;
  
    /* for eeprom save param */  
    INT16   OperationMode0x6060;
    UINT32  ProfileAcc0x6083;
    UINT32  ProfileDec0x6084;
    UINT32  QuickStopDec0x6085;
    UINT8   QuickStopEn0x6086;
    
    /* for eeprom save param */  
    UINT16  EepromCRC;

    INT16   IdRef0x2300;
    INT16   IqRef0x2301;
    INT16   IdFdb0x2302;
    INT16   IqFdb0x2303;
    INT16   VdRef0x2304;
    INT16   VqRef0x2305;
    INT16   Ia0x2306;
    INT16   Ib0x2307;
    INT16   Ic0x2308;
    UINT16  Vdc0x2309;
    UINT32  EncoderSingleTurn0x230A;
    INT16   MosTemp0x230B;
    INT16   MotorTemp0x230C;
    UINT32  ErrorRegister0x230D;
    
    UINT16  RestoreDefaults0x2400;
    UINT16  SaveParameter0x2401;
    UINT16  ClearErrorLog0x2402;
    UINT16  SystemReset0x2403;
    UINT16  ReadEepromParam0x2404;
    
    UINT32  TimeStamp0x2500;
    
    UINT16  CooperativeEnable0x3000;
    UINT16  CooperativeCW0x3001;
    INT32   CO_ProfileVelocity0x3002;
    UINT32  CO_ProfileTime0x3003;
    
    UINT16  InnerCtrlEnable0x3004;
    UINT16  InnerCtrlType0x3005;
    UINT16  InnerCtrlCycle0x3006;
    UINT32  InnerCtrlPeriod0x3007;
    UINT32  InnerCtrlAmp0x3008;
    UINT32  InnerCtrlOffset0x3009;
    
    UINT16  ErrorCode0x603F;
    UINT16  ControlWord0x6040;     
	  UINT16  StatusWord0x6041;        
    INT16   OperationModeDisplay0x6061; 
    INT32   ActualPosition0x6064;
    INT32   ActualVelocity0x606C;
    INT16   TargetCurrent0x6071;
    INT16   ActualCurrent0x6078; 
    INT32   TargetPosition0x607A;
    INT32   TargetVelocity0x60FF;
};


typedef struct ObjectEntryStruct
{
    struct ObjectEntryStruct     *pPrev; /**< \brief Previous entry(object) in the object dictionary list*/
    struct ObjectEntryStruct     *pNext; /**< \brief Next entry(object) in the object dictionary list*/

    UINT16                       Index;      /**< \brief Object index*/
    UINT16                       DataType;   /**< \brief Entry data type*/
    UINT16                       BitLength;  /**< \brief Entry bit size*/
    UINT16                       ObjAccess;  /**< \brief Entry access rights*/
    void                         *pVarPtr;   /**< \brief Pointer to object buffer*/
    
    #define ACCESS_READWRITE     0x003F /**< \brief Read/write in all states*/
    #define ACCESS_READ_ONLY     0x0007 /**< \brief Read only in all states*/
    #define ACCESS_WRITE_ONLY    0x0038 /**< \brief Write only in all states*/
}
OBJ_ENTRY;


extern struct ParameterStruct        gParam[2];
extern struct MachineInfoStruct      gMachineInfo;
extern struct SoftwareVersionStruct  gSoftVersion;
extern struct SensorDataStruct       gSensorData;
extern UINT8 	SysResetType;

//#define _OBJD_  // todo :should be delete
#ifdef _OBJD_
const OBJ_ENTRY ApplicationObjDic[] = {
{NULL, NULL, 0x2000, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[0].PositionLimitMin0x2000},  
{NULL, NULL, 0x2001, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[0].PositionLimitMax0x2001}, 
{NULL, NULL, 0x2002, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].SpeedLimit0x2002},
{NULL, NULL, 0x2003, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CurrentLimit0x2003},
{NULL, NULL, 0x2004, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].VdcLimitMax0x2004},
{NULL, NULL, 0x2005, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].VdcLimitMin0x2005},
{NULL, NULL, 0x2006, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].TempLimitMax0x2006},
{NULL, NULL, 0x2007, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].AlarmSwitch0x2007},
{NULL, NULL, 0x2008, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].SpeedFollowErrWindow0x2008},
{NULL, NULL, 0x2009, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].SpeedFollowErrTime0x2009},
{NULL, NULL, 0x200A, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CanLostErrTime0x200A},

{NULL, NULL, 0x2100, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].PositionLoopKp0x2100},
{NULL, NULL, 0x2101, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].VelocityLoopKp0x2101},
{NULL, NULL, 0x2102, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].VelocityLoopKi0x2102},
{NULL, NULL, 0x2103, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CurrentLoopKp0x2103},
{NULL, NULL, 0x2104, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CurrentLoopKi0x2104},
{NULL, NULL, 0x2105, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].FilterConfig0x2105},
{NULL, NULL, 0x2106, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].PosRefFilterFc0x2106},
{NULL, NULL, 0x2107, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].SpdRefFilterFc0x2107},
{NULL, NULL, 0x2108, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].SpdFdbFilterFc0x2108},
{NULL, NULL, 0x2109, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CurRefFilterFc0x2109},
{NULL, NULL, 0x210A, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].TorffFilterFc0x210A},
{NULL, NULL, 0x210B, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].TorffGain0x210B},
{NULL, NULL, 0x210C, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].DisturFilterFc0x210C},
{NULL, NULL, 0x210D, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].DisturGainFc0x210D},

{NULL, NULL, 0x2200, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].MotorMaxSpeed0x2200},
{NULL, NULL, 0x2201, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].MotorPolePairs0x2201},
{NULL, NULL, 0x2202, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].EncoderPPR0x2202},
{NULL, NULL, 0x2203, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].HallEnable0x2203},
{NULL, NULL, 0x2204, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].Commutation0x2204},
{NULL, NULL, 0x2205, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CommutationCurrent0x2205},
{NULL, NULL, 0x2206, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CommutationStableTime0x2206},
{NULL, NULL, 0x2207, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CommutationOffset0x2207},
{NULL, NULL, 0x2208, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].MotorInverse0x2208},
{NULL, NULL, 0x2209, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].MotorRatedCurrent0x2209},
{NULL, NULL, 0x220A, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].MotorPeakCurrent0x220A},

{NULL, NULL, 0x2300, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].IdRef0x2300},
{NULL, NULL, 0x2301, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].IqRef0x2301},
{NULL, NULL, 0x2302, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].IdFdb0x2302},
{NULL, NULL, 0x2303, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].IqFdb0x2303},
{NULL, NULL, 0x2304, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].VdRef0x2304},
{NULL, NULL, 0x2305, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].VqRef0x2305},
{NULL, NULL, 0x2306, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].Ia0x2306},
{NULL, NULL, 0x2307, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].Ib0x2307},
{NULL, NULL, 0x2308, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].Ic0x2308},
{NULL, NULL, 0x2309, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gParam[0].Vdc0x2309},
{NULL, NULL, 0x230A, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ_ONLY, &gParam[0].EncoderSingleTurn0x230A},
{NULL, NULL, 0x230B, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].MosTemp0x230B},
{NULL, NULL, 0x230C, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].MotorTemp0x230C},
{NULL, NULL, 0x230D, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ_ONLY, &gParam[0].ErrorRegister0x230D},

{NULL, NULL, 0x2400, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].RestoreDefaults0x2400},
{NULL, NULL, 0x2401, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].SaveParameter0x2401},
{NULL, NULL, 0x2402, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].ClearErrorLog0x2402},
{NULL, NULL, 0x2403, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].SystemReset0x2403},
{NULL, NULL, 0x2404, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].ReadEepromParam0x2404},
{NULL, NULL, 0x2405, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gMachineInfo.SaveMachineInfo},

{NULL, NULL, 0x2500, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].TimeStamp0x2500},

{NULL, NULL, 0x2800, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[1].PositionLimitMin0x2000},  
{NULL, NULL, 0x2801, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[1].PositionLimitMax0x2001}, 
{NULL, NULL, 0x2802, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].SpeedLimit0x2002},
{NULL, NULL, 0x2803, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].CurrentLimit0x2003},
{NULL, NULL, 0x2804, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].VdcLimitMax0x2004},
{NULL, NULL, 0x2805, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].VdcLimitMin0x2005},
{NULL, NULL, 0x2806, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].TempLimitMax0x2006},
{NULL, NULL, 0x2807, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[1].AlarmSwitch0x2007},
{NULL, NULL, 0x2808, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[1].SpeedFollowErrWindow0x2008},
{NULL, NULL, 0x2809, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[1].SpeedFollowErrTime0x2009},

{NULL, NULL, 0x2900, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].PositionLoopKp0x2100},
{NULL, NULL, 0x2901, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].VelocityLoopKp0x2101},
{NULL, NULL, 0x2902, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].VelocityLoopKi0x2102},
{NULL, NULL, 0x2903, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].CurrentLoopKp0x2103},
{NULL, NULL, 0x2904, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].CurrentLoopKi0x2104},
{NULL, NULL, 0x2905, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].FilterConfig0x2105},
{NULL, NULL, 0x2906, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].PosRefFilterFc0x2106},
{NULL, NULL, 0x2907, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].SpdRefFilterFc0x2107},
{NULL, NULL, 0x2908, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].SpdFdbFilterFc0x2108},
{NULL, NULL, 0x2909, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].CurRefFilterFc0x2109},
{NULL, NULL, 0x290A, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].TorffFilterFc0x210A},
{NULL, NULL, 0x290B, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].TorffGain0x210B},
{NULL, NULL, 0x290C, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].DisturFilterFc0x210C},
{NULL, NULL, 0x290D, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].DisturGainFc0x210D},

{NULL, NULL, 0x2A00, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].MotorMaxSpeed0x2200},
{NULL, NULL, 0x2A01, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].MotorPolePairs0x2201},
{NULL, NULL, 0x2A02, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[1].EncoderPPR0x2202},
{NULL, NULL, 0x2A03, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].HallEnable0x2203},
{NULL, NULL, 0x2A04, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].Commutation0x2204},
{NULL, NULL, 0x2A05, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].CommutationCurrent0x2205},
{NULL, NULL, 0x2A06, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].CommutationStableTime0x2206},
{NULL, NULL, 0x2A07, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].CommutationOffset0x2207},
{NULL, NULL, 0x2A08, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].MotorInverse0x2208},
{NULL, NULL, 0x2A09, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].MotorRatedCurrent0x2209},
{NULL, NULL, 0x2A0A, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].MotorPeakCurrent0x220A},

{NULL, NULL, 0x2B00, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].IdRef0x2300},
{NULL, NULL, 0x2B01, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].IqRef0x2301},
{NULL, NULL, 0x2B02, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].IdFdb0x2302},
{NULL, NULL, 0x2B03, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].IqFdb0x2303},
{NULL, NULL, 0x2B04, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].VdRef0x2304},
{NULL, NULL, 0x2B05, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].VqRef0x2305},
{NULL, NULL, 0x2B06, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].Ia0x2306},
{NULL, NULL, 0x2B07, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].Ib0x2307},
{NULL, NULL, 0x2B08, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].Ic0x2308},
{NULL, NULL, 0x2B09, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gParam[1].Vdc0x2309},
{NULL, NULL, 0x2B0A, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ_ONLY, &gParam[1].EncoderSingleTurn0x230A},
{NULL, NULL, 0x2B0B, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].MosTemp0x230B},
{NULL, NULL, 0x2B0C, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].MotorTemp0x230C},
{NULL, NULL, 0x2B0D, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ_ONLY, &gParam[1].ErrorRegister0x230D},

{NULL, NULL, 0x2C00, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].RestoreDefaults0x2400},
{NULL, NULL, 0x2C01, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].SaveParameter0x2401},

{NULL, NULL, 0x3000, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CooperativeEnable0x3000},
{NULL, NULL, 0x3001, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].CooperativeCW0x3001},
{NULL, NULL, 0x3002, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[0].CO_ProfileVelocity0x3002},
{NULL, NULL, 0x3003, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].CO_ProfileTime0x3003},
{NULL, NULL, 0x3004, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].InnerCtrlEnable0x3004},
{NULL, NULL, 0x3005, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].InnerCtrlType0x3005},
{NULL, NULL, 0x3006, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].InnerCtrlCycle0x3006},
{NULL, NULL, 0x3007, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].InnerCtrlPeriod0x3007},
{NULL, NULL, 0x3008, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].InnerCtrlAmp0x3008},
{NULL, NULL, 0x3009, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].InnerCtrlOffset0x3009},

{NULL, NULL, 0x4000, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gSensorData.GyroSpeedX0x4000},
{NULL, NULL, 0x4001, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gSensorData.GyroSpeedY0x4001},
{NULL, NULL, 0x4002, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gSensorData.GyroSpeedZ0x4002},
{NULL, NULL, 0x4003, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gSensorData.GyroAccelX0x4003},
{NULL, NULL, 0x4004, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gSensorData.GyroAccelY0x4004},
{NULL, NULL, 0x4005, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gSensorData.GyroAccelZ0x4005},
{NULL, NULL, 0x4006, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READ_ONLY, &gSensorData.InfraRed10x4006},
{NULL, NULL, 0x4007, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READ_ONLY, &gSensorData.InfraRed20x4007},
{NULL, NULL, 0x4008, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READ_ONLY, &gSensorData.InfraRed30x4008},
{NULL, NULL, 0x4009, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READ_ONLY, &gSensorData.InfraRed40x4009},
{NULL, NULL, 0x400A, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gSensorData.BatteryVoltage0x400A},
{NULL, NULL, 0x400B, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gSensorData.BatteryCurrent0x400B},
{NULL, NULL, 0x400C, DEFTYPE_INTEGER16, 0x10, ACCESS_READ_ONLY, &gSensorData.ChargeCurrent0x400C},
{NULL, NULL, 0x400D, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gSensorData.BatteryLevel0x400D},
{NULL, NULL, 0x400E, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gSensorData.ChargeState0x400E},
{NULL, NULL, 0x400F, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gSensorData.ChargeVoltage0x400F},
{NULL, NULL, 0x4010, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gSensorData.BatteryTemp0x4010},

{NULL, NULL, 0x4100, DEFTYPE_REAL32, 0x20, ACCESS_READWRITE, &gMachineInfo.machineWheelDiameter},
{NULL, NULL, 0x4101, DEFTYPE_REAL32, 0x20, ACCESS_READWRITE, &gMachineInfo.machineWheelPerimeter},
{NULL, NULL, 0x4102, DEFTYPE_REAL32, 0x20, ACCESS_READWRITE, &gMachineInfo.machineWheelbase},
{NULL, NULL, 0x4103, DEFTYPE_REAL32, 0x20, ACCESS_READ_ONLY, &gMachineInfo.encoderPulsePerCircle},
{NULL, NULL, 0x4104, DEFTYPE_REAL32, 0x20, ACCESS_READ_ONLY, &gMachineInfo.encoderSampleTimesPerPulse},
{NULL, NULL, 0x4105, DEFTYPE_REAL32, 0x20, ACCESS_READ_ONLY, &gMachineInfo.reductionRatio},
{NULL, NULL, 0x4106, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.isEncoderCountInv},
{NULL, NULL, 0x4107, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.uwbTagPcbMajorVersion},
{NULL, NULL, 0x4108, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.uwbTagPcbMinorVersion},
{NULL, NULL, 0x4109, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.chassisPcbMajorVersion},
{NULL, NULL, 0x410A, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.chassisPcbMinorVersion},
{NULL, NULL, 0x410B, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.infraredSensorVersion},
{NULL, NULL, 0x410C, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.ldsSensorVersion},
{NULL, NULL, 0x410D, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.motorVersion},
{NULL, NULL, 0x410E, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.weighSensorVersion},
{NULL, NULL, 0x410F, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.batteryVersion},
{NULL, NULL, 0x4110, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.uwbTagPcbMPD.year},
{NULL, NULL, 0x4111, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.uwbTagPcbMPD.month},
{NULL, NULL, 0x4112, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.uwbTagPcbMPD.day},
{NULL, NULL, 0x4113, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.chassisPcbMPD.year},
{NULL, NULL, 0x4114, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.chassisPcbMPD.month},
{NULL, NULL, 0x4115, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.chassisPcbMPD.day},
/*INSERT->20190503*/
{NULL, NULL, 0x4116, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.makerCameraVersion},
/*INSERT->20190628*/
{NULL, NULL, 0x4117, DEFTYPE_UNSIGNED8, 0x08, ACCESS_READWRITE, &gMachineInfo.ESP32Version},
/*INSERT->20190716*/
{NULL, NULL, 0x4118, DEFTYPE_UNSIGNED8, 0x08, ACCESS_READWRITE, &gMachineInfo.RGBDVersion},

{NULL, NULL, 0x4119, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.type},
{NULL, NULL, 0x411A, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.AudioVersion},
{NULL, NULL, 0x411B, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.LoRaVersion},
{NULL, NULL, 0x411C, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.Odometer},
{NULL, NULL, 0x411D, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gMachineInfo.CumulativeTime},

/*INSERT->20200725*/
{NULL, NULL, 0x411E, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.BarCodeScanningGunVersion},
{NULL, NULL, 0x411F, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.FaceDetectCameraVersion},
{NULL, NULL, 0x4120, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.SlamwareVersion},

/*INSERT->20201019*/
{NULL, NULL, 0x4121, DEFTYPE_UNSIGNED16,  0x10, ACCESS_READWRITE, &gMachineInfo.MachineModel},
{NULL, NULL, 0x4122, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.MachineMainVersion},
{NULL, NULL, 0x4123, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.MachineMinorVersion},

/*INSERT->20210831 Index 28*/ 
{NULL, NULL, 0x4124, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.NpuTpye},
{NULL, NULL, 0x4125, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.Matrixmic},
{NULL, NULL, 0x4126, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.HostCoreBoardType},
{NULL, NULL, 0x4127, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.HeadType},

/*INSERT->20210831 Index 29*/ 
{NULL, NULL, 0x4128, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.LidarCommunicateType},
{NULL, NULL, 0x4129, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.LidarType},
{NULL, NULL, 0x412A, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.Version4G},
{NULL, NULL, 0x412B, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.CarbinDoorMotorType},

/*INSERT->20211025 Index 30*/ 
{NULL, NULL, 0x412C, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.TrayBoardType},
{NULL, NULL, 0x412D, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.FunctionBoardType},
{NULL, NULL, 0x412E, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.ChassisBoardType},
{NULL, NULL, 0x412F, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.DoorBoardType},

/*INSERT->20211123 Index 31*/ 
{NULL, NULL, 0x4130, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.LaserProjection},
{NULL, NULL, 0x4131, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.VedioOutput},
{NULL, NULL, 0x4132, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.DistributionAreaLight},
{NULL, NULL, 0x4133, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.res31_1},

/*INSERT->20211123 Index 32*/ 
{NULL, NULL, 0x4134, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.PowerBoardType},
{NULL, NULL, 0x4135, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.res32_1},
{NULL, NULL, 0x4136, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.res32_2},
{NULL, NULL, 0x4137, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gMachineInfo.res32_3},

{NULL, NULL, 0x603F, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gParam[0].ErrorCode0x603F},
{NULL, NULL, 0x6040, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[0].ControlWord0x6040},
{NULL, NULL, 0x6041, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gParam[0].StatusWord0x6041},
{NULL, NULL, 0x6060, DEFTYPE_INTEGER16,  0x10, ACCESS_READWRITE, &gParam[0].OperationMode0x6060},
{NULL, NULL, 0x6061, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].OperationModeDisplay0x6061},
{NULL, NULL, 0x6064, DEFTYPE_INTEGER32,  0x20, ACCESS_READ_ONLY, &gParam[0].ActualPosition0x6064},
{NULL, NULL, 0x606C, DEFTYPE_INTEGER32,  0x20, ACCESS_READ_ONLY, &gParam[0].ActualVelocity0x606C},
{NULL, NULL, 0x6071, DEFTYPE_INTEGER16,  0x10, ACCESS_READWRITE, &gParam[0].TargetCurrent0x6071},
{NULL, NULL, 0x6078, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[0].ActualCurrent0x6078},
{NULL, NULL, 0x607A, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[0].TargetPosition0x607A},
{NULL, NULL, 0x6083, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].ProfileAcc0x6083},
{NULL, NULL, 0x6084, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].ProfileDec0x6084},
{NULL, NULL, 0x6085, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[0].QuickStopDec0x6085},
/*param:0x6086, Inserted by DengZhuo 20190509*/
{NULL, NULL, 0x6086, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gParam[0].QuickStopEn0x6086},
{NULL, NULL, 0x60FF, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[0].TargetVelocity0x60FF},

{NULL, NULL, 0x683F, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gParam[1].ErrorCode0x603F},
{NULL, NULL, 0x6840, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE, &gParam[1].ControlWord0x6040},
{NULL, NULL, 0x6841, DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ_ONLY, &gParam[1].StatusWord0x6041},
{NULL, NULL, 0x6860, DEFTYPE_INTEGER16,  0x10, ACCESS_READWRITE, &gParam[1].OperationMode0x6060},
{NULL, NULL, 0x6861, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].OperationModeDisplay0x6061},
{NULL, NULL, 0x6864, DEFTYPE_INTEGER32,  0x20, ACCESS_READ_ONLY, &gParam[1].ActualPosition0x6064},
{NULL, NULL, 0x686C, DEFTYPE_INTEGER32,  0x20, ACCESS_READ_ONLY, &gParam[1].ActualVelocity0x606C},
{NULL, NULL, 0x6871, DEFTYPE_INTEGER16,  0x10, ACCESS_READWRITE, &gParam[1].TargetCurrent0x6071},
{NULL, NULL, 0x6878, DEFTYPE_INTEGER16,  0x10, ACCESS_READ_ONLY, &gParam[1].ActualCurrent0x6078},
{NULL, NULL, 0x687A, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[1].TargetPosition0x607A},
{NULL, NULL, 0x6883, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[1].ProfileAcc0x6083},
{NULL, NULL, 0x6884, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[1].ProfileDec0x6084},
{NULL, NULL, 0x6885, DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE, &gParam[1].QuickStopDec0x6085},
/*param:0x6086, Inserted by DengZhuo 20190509*/
{NULL, NULL, 0x6886, DEFTYPE_UNSIGNED8,  0x08, ACCESS_READWRITE, &gParam[1].QuickStopEn0x6086},
{NULL, NULL, 0x68FF, DEFTYPE_INTEGER32,  0x20, ACCESS_READWRITE, &gParam[1].TargetVelocity0x60FF},


{NULL, NULL, 0xFFFF, 0x0, 0, NULL}};

#endif // _OBJD_
#ifdef _OBJD_
#warning "print not define OBJD"
const struct ParameterStruct gDefaultParam_Left[10] = {
//Motor Type:1->ZL 5.5" 1024PRD
[1] = {0x80000000, 0x7FFFFFFF, 300, 12000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 10, 4096, 1, 0, 1000, 500, 0, 1, 3000, 12000,   // motor param
3, 6516, 6516, 9308,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},

//Motor Type:2->ZL 5.5" 4096PRD
[2] = {0x80000000, 0x7FFFFFFF, 300, 12000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 10, 16384, 1, 0, 1000, 500, 0, 1, 3000, 12000,   // motor param
3, 26000, 26000, 40000,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},

//Motor Type:3->ZL 6.5" 1024PRD
[3] = {0x80000000, 0x7FFFFFFF, 300, 18000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 15, 4096, 1, 0, 1000, 500, 0, 0, 3000, 18000,   // motor param
3, 5366, 5366, 7665,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:4->Maxwell 5.5" 1024PRD
[4] = {0x80000000, 0x7FFFFFFF, 200, 10000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 380, 15, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
300, 19, 4096, 2, 0, 1000, 500, 0, 0, 3000, 10000,   // motor param
3, 6516, 6516, 9308,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:6->DXC 6.5" 1024PRD
[6] = {0x80000000, 0x7FFFFFFF, 300, 14000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 400, 7, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 15, 4096, 1, 0, 1000, 500, 0, 1, 3000, 14000,   // motor param
3, 5366, 5366, 7665,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},

//Motor Type:7->YaTeng 5.5" 1024PRD
[7] = {0x80000000, 0x7FFFFFFF, 300, 13500, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 3, 0x0F, 100, 20, 30, 50, 200, 10, 10, 10,       // motion config
400, 13, 16384, 1, 0, 1000, 500, 0, 1, 3000, 13500,   // motor param
3, 26060, 26060, 37228,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:8->YaTeng 6.5" 4096PRD
[8]= {0x80000000, 0x7FFFFFFF, 300, 18000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 40, 8, 150, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 10,       // motion config
400, 15, 4096, 1, 0, 1000, 500, 0, 0, 6000, 18000,   // motor param
3, 22000, 22000, 32000,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:9->Maxwell 5.5" 1024PRD 降本
[9] = {0x80000000, 0x7FFFFFFF, 200, 10000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 220, 12, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
300, 19, 4096, 2, 0, 1000, 500, 0, 0, 3000, 10000,   // motor param
3, 6516, 6516, 9308,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
}
};


const struct ParameterStruct gDefaultParam_Right[10] = {
//Motor Type:1->ZL 5.5" 1024PRD
[1] = {0x80000000, 0x7FFFFFFF, 300, 12000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 10, 4096, 1, 0, 1000, 500, 0, 0, 3000, 12000,   // motor param
3, 6516, 6516, 9308,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},

//Motor Type:2->ZL 5.5" 4096PRD
[2] = {0x80000000, 0x7FFFFFFF, 300, 12000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 10, 16384, 1, 0, 1000, 500, 0, 0, 3000, 12000,   // motor param
3, 26000, 26000, 60000,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},

//Motor Type:3->ZL 6.5" 1024PRD
[3] = {0x80000000, 0x7FFFFFFF, 300, 18000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 15, 4096, 1, 0, 1000, 500, 0, 1, 5000, 18000,   // motor param
3, 5366, 5366, 7665,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:4->Maxwell 5.5" 1024PRD
[4] = {0x80000000, 0x7FFFFFFF, 200, 10000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 380, 15, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
300, 19, 4096, 2, 0, 1000, 500, 0, 1, 3000, 10000,   // motor param
3, 6516, 6516, 9308,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:6->DXC 6.5" 1024PRD
[6] = {0x80000000, 0x7FFFFFFF, 300, 14000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 400, 7, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 15, 4096, 1, 0, 1000, 500, 0, 0, 3000, 14000,   // motor param
3, 5366, 5366, 7665,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},

//Motor Type:7->Yateng 5.5" 4096PRD
[7] = {0x80000000, 0x7FFFFFFF, 300, 13500, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 100, 3, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
400, 13, 16384, 1, 0, 1000, 500, 0, 0, 4000, 13500,   // motor param
3, 26060, 26060, 37228,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:8->YaTeng 6.5" 1024PRD
[8]= {0x80000000, 0x7FFFFFFF, 300, 18000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 50, 8, 150, 10, 0x0F, 100, 20, 30, 50, 200, 10, 10, 10,       // motion config
400, 15, 16384, 1, 0, 1000, 500, 0, 1, 5000, 18000,   // motor param
3, 22000, 22000, 32000,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
},
//Motor Type:9->Maxwell 5.5" 1024PRD 降本
[9] = {0x80000000, 0x7FFFFFFF, 200, 10000, 36000, 15000, 80, 0xFFFFFFFF, 60, 400, 300, // Limit param
100, 80, 10, 220, 12, 0x0F, 100, 20, 30, 50, 200, 10, 10, 50,       // motion config
300, 19, 4096, 2, 0, 1000, 500, 0, 1, 3000, 10000,   // motor param
3, 6516, 6516, 9308,           // motion mode
0,                                   // EepromCrc
0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0,                 // assist param
0,0,0,0,0,0,0,0,0,0   
}
};

//Motor Data Saved In MachineInfo
const struct MotorDataInMachineInfoStruct gMotor_basicData[10] = {
//Motor Type:1->ZL 5.5" 1024PRD
[1] = {0.14f, 0.439823f, 0.4405f,     // R, L, D
1024, 4, 1, 0       // Line, factor, ratio, inv
},

//Motor Type:2->ZL 5.5" 4096PRD
[2] = {0.14f, 0.439823f, 0.4405f,     // R, L, D
4096, 4, 1, 0       // Line, factor, ratio, inv
},

//Motor Type:3->ZL 6.5" 1024PRD
[3] = {0.173f, 0.534071f, 0.4003f,     // R, L, D
1024, 4, 1, 0       // Line, factor, ratio, inv
},
//Motor Type:4->Maxwell 5.5" 1024PRD
[4] = {0.14f, 0.439823f, 0.4425f,     // R, L, D
1024, 4, 1, 0       // Line, factor, ratio, inv
},
//Motor Type:6->DXC 6.5" 1024PRD
[6] = {0.17f, 0.534071f, 0.403f,     // R, L, D
1024, 4, 1, 0       // Line, factor, ratio, inv
},

//Motor Type:7->TaTeng 5.5" 4096PRD
[7] = {0.14f, 0.439823f, 0.3836f,     // R, L, D
4096, 4, 1, 0       // Line, factor, ratio, inv
},
//Motor Type:3->ZL 6.5" 1024PRD
[8] = {0.173f, 0.534071f, 0.4003f,     // R, L, D
4096, 4, 1, 0       // Line, factor, ratio, inv
},
//Motor Type:9->Maxwell 5.5" 1024PRD 降本
[9] = {0.14f, 0.439823f, 0.4425,     // R, L, D
1024, 4, 1, 0       // Line, factor, ratio, inv
},
};


const struct MachineInfoStruct gDefaultMachineInfo = {
0.14f, 0.439823f, 0.4425f,     // R, L, D
1024, 4, 1, 0,                 // Line, factor, ratio, inv
2, 3,   //Tag Pcb Version
2, 4,   //Chassis Pcb Version
2,
7,      //LDS version
1,
0, 
1,
{2019, 5, 27},
{2019, 5, 22},
1,    //markerCameraVersion
1,    //ESP32Version
0,    //RGBDVersion
6,    //machine type
};
#endif // _OBJD_

extern PUBLIC void ParamInit(void);
extern PUBLIC void ParamExec(void);
extern PUBLIC void ParamLoop(void);
extern PUBLIC UINT16 GetEepromErrorCode(void);
extern PUBLIC OBJ_ENTRY*  OBJ_GetObjectHandle(UINT16 index );
extern PUBLIC UINT8 GetResetType(void);
extern PUBLIC void BKP_Init(void);
extern PUBLIC void RTC_BKP_Write(UINT32 uiAddr0_19,UINT32 uiDataToWrite);
extern PUBLIC UINT32 RTC_BKP_Read(UINT32 uiAddr0_19);
extern PUBLIC void SNExecLoop(void);
#endif  // _PARAM_H_
