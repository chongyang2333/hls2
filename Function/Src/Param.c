/*******************************************************************
 *
 * FILE NAME:  Eeprom.c
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

/*------------------------- Include files ----------------------------*/

#define _OBJD_
#include "Param.h"
#undef _OBJD_
#include "UartApp.h"
#include "Eeprom.h"
#include "string.h"
#include "ControlRun.h"
#include "ErrorLog.h"
#include "CanApp.h"
#include "MachineAdditionalInfo.h"
#include "gd32f4xx.h"
#include "gd_hal.h"

#define OBJ_NUM (sizeof(ApplicationObjDic)/sizeof(OBJ_ENTRY))
    
#define AXIS_Left_PARAM_EEPROM_ADDR   0
#define AXIS_Right_PARAM_EEPROM_ADDR  256
#define MACHINE_INFO_EEPROM_ADDR      512
#define BATTERY_INFO_EEPROM_ARRD      416      //电池信息存储位置

struct ParameterStruct    gParam[2];
struct MachineInfoStruct  gMachineInfo;
struct SoftwareVersionStruct gSoftVersion = {21,0,2};
struct SensorDataStruct gSensorData = {0};

BatterySaveInfo_t g_tBatterySaveInfo;

//RTC_HandleTypeDef 				hrtc1;

UINT16 EepromErrorFlag=0;
UINT8 MotorVersionParam = 0;
UINT16 init_isr_flag = 0;

PRIVATE void LoadParamFromEeprom(UINT16 AxisID);
PRIVATE void SaveParamToEeprom(UINT16 AxisID);

PRIVATE void SaveMachineInfo(void);
PRIVATE UINT8 LoadMachineInfo(void);
PRIVATE UINT8 MachineInfoFactoryCheck(void);
PRIVATE UINT8 RestoreMachineInfo(void);
PRIVATE UINT8 EraseMachineInfo(void);
PRIVATE void SaveBatteryInfo(void);
PRIVATE UINT8 LoadBatteryInfo(void);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ParamInit(void)
{  	
    LoadMachineInfo();
    
    LoadParamFromEeprom(0);
    LoadParamFromEeprom(1);
	
    LoadMachineAddInfo();
    gMachineInfo.Odometer = MachineAddInfo.Odometer.Value;
    gMachineInfo.CumulativeTime = MachineAddInfo.CumulativeTime.Value;         
    gMachineInfo.MachineInfoSaveState = MACHINE_INFO_NO_CMD;
    
    LoadBatteryInfo();
    ErrorLogInit();
    
    gParam[0].ControlWord0x6040 = 6;
    gParam[1].ControlWord0x6040 = 6;
    
    gMachineInfo.encoderPulsePerCircle = (float)(gParam[0].EncoderPPR0x2202>>2);
    gMachineInfo.encoderSampleTimesPerPulse = 4;
    gMachineInfo.reductionRatio = 1;
    gMachineInfo.MotorVersionLast = gMachineInfo.motorVersion;
    
//    hrtc1.Instance = RTC;
    BKP_Init();
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ParamExec(void)
{

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
#define DEFAULT_MACHINE_MOTOR_TYPE 1
PUBLIC void ParamLoop(void)
{
    if(gParam[0].RestoreDefaults0x2400)
    {
        UINT32 len= (UINT32)&gParam[AXIS_LEFT].EepromCRC - (UINT32)&gParam[AXIS_LEFT].PositionLimitMin0x2000;
        //HLS Robot:Motor Type->1
        memcpy(&gParam[AXIS_LEFT], &gDefaultParam_Left[DEFAULT_MACHINE_MOTOR_TYPE], len);
        SaveParamToEeprom(0);
        LoadParamFromEeprom(0);
        memcpy(&gParam[AXIS_RIGHT], &gDefaultParam_Right[DEFAULT_MACHINE_MOTOR_TYPE], len);
        SaveParamToEeprom(1);
        LoadParamFromEeprom(1);
        gParam[0].RestoreDefaults0x2400 = 0;
        gMachineInfo.MotorVersionLast = 1;
        
        gMachineInfo.RestoreDefault = 1; //  todo
        gMachineInfo.MotorVersionLast = 1;
    }
    
    if(gParam[0].SaveParameter0x2401)
    {
        /*MotorVersion In MachineInfo Changed
        * SaveParameter0x2401 == 1 : save debuger param ;
        * SaveParameter0x2401 == 2 : download param & rst motor-config param;
        */
        if (gMachineInfo.motorVersion != gMachineInfo.MotorVersionLast || gParam[0].SaveParameter0x2401 == 2) 
        {
            UINT32 len= (UINT32)&gParam[AXIS_LEFT].EepromCRC - (UINT32)&gParam[AXIS_LEFT].PositionLimitMin0x2000;
            gMachineInfo.MotorVersionLast = gMachineInfo.motorVersion;
            memcpy(&gParam[AXIS_LEFT], &gDefaultParam_Left[gMachineInfo.motorVersion], len);
            memcpy(&gParam[AXIS_RIGHT], &gDefaultParam_Right[gMachineInfo.motorVersion], len);
            memcpy(&gMachineInfo ,&gMotor_basicData[gMachineInfo.motorVersion], sizeof(struct MotorDataInMachineInfoStruct));
        }
        
        //HLS Robot:Motor Type->1
        gParam[AXIS_LEFT].ProfileAcc0x6083 = gDefaultParam_Left[gMachineInfo.motorVersion].ProfileAcc0x6083;
        gParam[AXIS_LEFT].ProfileDec0x6084 = gDefaultParam_Left[gMachineInfo.motorVersion].ProfileDec0x6084;
        gParam[AXIS_LEFT].QuickStopDec0x6085 = gDefaultParam_Left[gMachineInfo.motorVersion].QuickStopDec0x6085;
        gParam[AXIS_RIGHT].ProfileAcc0x6083 = gDefaultParam_Right[gMachineInfo.motorVersion].ProfileAcc0x6083;
        gParam[AXIS_RIGHT].ProfileDec0x6084 = gDefaultParam_Right[gMachineInfo.motorVersion].ProfileDec0x6084;
        gParam[AXIS_RIGHT].QuickStopDec0x6085 = gDefaultParam_Right[gMachineInfo.motorVersion].QuickStopDec0x6085;        
        
        SaveParamToEeprom(0);
        LoadParamFromEeprom(0);
        SaveParamToEeprom(1);
        LoadParamFromEeprom(1);
        gParam[0].SaveParameter0x2401 = 0;
        
        gMachineInfo.SaveMachineInfo = 1;
        EnableMachineAddInfoSave();
        
        CanCarpetModeFdb(gParam[0].MotorRatedCurrent0x2209);
    }  
    
    if(gParam[0].ReadEepromParam0x2404)
    {
        LoadParamFromEeprom(0);
        LoadParamFromEeprom(1);
        gParam[0].ReadEepromParam0x2404 = 0;
    }
    
    if(gMachineInfo.RestoreDefault)
    {
        UINT32 len= (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
        UINT32 Tmp = gMachineInfo.batteryVersion;
        memcpy(&gMachineInfo, &gDefaultMachineInfo, len);
        //HLS Robot:Motor Type->1
        memcpy(&gMachineInfo, &gMotor_basicData[DEFAULT_MACHINE_MOTOR_TYPE], sizeof(struct MotorDataInMachineInfoStruct));
        gMachineInfo.batteryVersion = Tmp;
        SaveMachineInfo();
        LoadMachineInfo();
        gMachineInfo.RestoreDefault = 0;
    }
    
    if(gMachineInfo.SaveMachineInfo)
    {
        gMachineInfo.MachineInfoSaveState = MACHINE_INFO_WRITING;
        if(IsMachineAddInfoSaveOK())
        {
            SaveMachineInfo();
            
            if(LoadMachineInfo())
            {
                gMachineInfo.MachineInfoSaveState = MACHINE_INFO_SUCCESS;
            }
            else
            {
                gMachineInfo.MachineInfoSaveState = MACHINE_INFO_FAULT;
            }
            gMachineInfo.SaveMachineInfo = 0;
        }        
    }
    
    if(g_tBatterySaveInfo.hSaveInfo)
    {
        SaveBatteryInfo();
        LoadBatteryInfo();
        g_tBatterySaveInfo.hSaveInfo = 0;
    }

    if(gParam[0].ClearErrorLog0x2402)
    {
        ClearErrorLog();
        gParam[0].ClearErrorLog0x2402 = 0;
    }
    
    if(gParam[0].SystemReset0x2403)
    {
		RTC_BKP_Write(EN_RESET_TYPE_BKP_ADDR,EN_RESET_TYPE_SOFT);
        HAL_NVIC_SystemReset();
    }

    MachineAddInfoProcess();    
		if((gMachineInfo.motorVersion != 4)&&(init_isr_flag == 0))//非maxwell电机，取消外部中断
		{	
			init_isr_flag = 1;		
		    nvic_irq_disable(EXTI3_IRQn);
        	nvic_irq_disable(EXTI0_IRQn); 
		}
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT16 GetEepromErrorCode(void)
{
    return EepromErrorFlag;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SaveParamToEeprom(UINT16 AxisID)
{
    UINT32 len= (UINT32)&gParam[AXIS_LEFT].EepromCRC - (UINT32)&gParam[AXIS_LEFT].PositionLimitMin0x2000;
    
    if(AxisID == 0)
    {
        gParam[AXIS_LEFT].EepromCRC = GetCRC16((unsigned char*)&gParam[AXIS_LEFT], len);
        EEPROM_Serial_Write(AXIS_Left_PARAM_EEPROM_ADDR, (UINT8*)&gParam[AXIS_LEFT], len+2);
    }
    else if(AxisID == 1)
    {
        gParam[AXIS_RIGHT].EepromCRC = GetCRC16((unsigned char*)&gParam[AXIS_RIGHT], len);
        EEPROM_Serial_Write(AXIS_Right_PARAM_EEPROM_ADDR, (UINT8*)&gParam[AXIS_RIGHT], len+2);
    }
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void LoadParamFromEeprom(UINT16 AxisID)
{
    struct ParameterStruct  gParam_tmp;
    
    UINT32 len= (UINT32)&gParam[AXIS_LEFT].EepromCRC - (UINT32)&gParam[AXIS_LEFT].PositionLimitMin0x2000;
 
    if(AxisID == 0)
    {
        
        EEPROM_Serial_Read(AXIS_Left_PARAM_EEPROM_ADDR, (UINT8*)&gParam_tmp, len+2);
        if(GetCRC16((UINT8*)&gParam_tmp.PositionLimitMin0x2000, len) == gParam_tmp.EepromCRC )
        {
            memcpy(&gParam[AXIS_LEFT], &gParam_tmp, len+2);
        }
        else
        {
            EepromErrorFlag |= (0x1<<0);
        }
    }
    else if(AxisID == 1)
    {
        EEPROM_Serial_Read(AXIS_Right_PARAM_EEPROM_ADDR, (UINT8*)&gParam_tmp, len+2);
        if(GetCRC16((UINT8*)&gParam_tmp.PositionLimitMin0x2000, len) == gParam_tmp.EepromCRC )
        {
            memcpy(&gParam[AXIS_RIGHT], &gParam_tmp, len+2);
        }
        else
        {
            EepromErrorFlag |= (0x1<<1);
        }
    }
    
    gParam[AxisID].QuickStopDec0x6085 = gDefaultParam_Left[gMachineInfo.motorVersion].QuickStopDec0x6085;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SaveMachineInfo(void)
{
    UINT32 len= (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
    
    gMachineInfo.EepromCRC = GetCRC16((unsigned char*)&gMachineInfo, len);
    EEPROM_Serial_Write(MACHINE_INFO_EEPROM_ADDR, (UINT8*)&gMachineInfo, len+2);

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 LoadMachineInfo(void)
{
    struct MachineInfoStruct MachineInfo_tmp;
    
    UINT32 len= (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
    
    EEPROM_Serial_Read(MACHINE_INFO_EEPROM_ADDR, (UINT8*)&MachineInfo_tmp, len+2);
    
    if(GetCRC16((UINT8*)&MachineInfo_tmp, len) == MachineInfo_tmp.EepromCRC )
    {
        memcpy(&gMachineInfo, &MachineInfo_tmp, len+2);
        return TRUE;
    }
    else
    {
        gMachineInfo.CrcState = 2;
        EepromErrorFlag = 0x03;
        return FALSE;
    }
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SaveBatteryInfo(void)
{
    UINT32 len= (UINT32)&g_tBatterySaveInfo.hEepromCRC - (UINT32)&g_tBatterySaveInfo;
    
    g_tBatterySaveInfo.hEepromCRC = GetCRC16((unsigned char*)&g_tBatterySaveInfo, len);
    EEPROM_Serial_Write(BATTERY_INFO_EEPROM_ARRD, (UINT8*)&g_tBatterySaveInfo, len+2);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 LoadBatteryInfo(void)
{
    BatterySaveInfo_t tBatterySaveInfo;
    
    UINT32 len= (UINT32)&tBatterySaveInfo.hEepromCRC - (UINT32)&tBatterySaveInfo;
    
    EEPROM_Serial_Read(BATTERY_INFO_EEPROM_ARRD, (UINT8*)&tBatterySaveInfo, len+2);
    
    if(GetCRC16((UINT8*)&tBatterySaveInfo, len) == tBatterySaveInfo.hEepromCRC )
    {
        memcpy(&g_tBatterySaveInfo, &tBatterySaveInfo, len+2);
        return TRUE;
    }
    else
    {
        g_tBatterySaveInfo.CrcState = 2;
        EepromErrorFlag |= 0x04;
        return FALSE;
    }
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 MachineInfoFactoryCheck(void)
{
		struct MachineInfoStruct MachineInfo_tmp;
    
    UINT32 len= (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
    
    EEPROM_Serial_Read(MACHINE_INFO_EEPROM_ADDR, (UINT8*)&MachineInfo_tmp, len+2);
    
    if(GetCRC16((UINT8*)&MachineInfo_tmp, len) != MachineInfo_tmp.EepromCRC )
    {
        return FALSE;
    }
    return TRUE;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 RestoreMachineInfo(void)
{
		//struct MachineInfoStruct MachineInfo_tmp;
		UINT32 len = (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
		memcpy(&gMachineInfo,&gDefaultMachineInfo,len+2);
		SaveMachineInfo();
		return LoadMachineInfo();
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 EraseMachineInfo(void)
{
		struct MachineInfoStruct MachineInfo_tmp;
		UINT32 len = (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
		memset(&MachineInfo_tmp,0xFF,len+2);
    EEPROM_Serial_Write(MACHINE_INFO_EEPROM_ADDR, (UINT8*)&MachineInfo_tmp, len+2);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC OBJ_ENTRY*  OBJ_GetObjectHandle(UINT16 index )
{
    UINT16 TempMin = 0;
    UINT16 TempMax = OBJ_NUM-2;
    UINT16 MidVal = 0;
    OBJ_ENTRY *pObjEntry = NULL;

    while(TempMax >= TempMin)
    {     
        MidVal = (TempMax + TempMin) / 2;
        pObjEntry = (OBJ_ENTRY*)&ApplicationObjDic[MidVal]; 
        
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


PUBLIC UINT8 GetResetType(void)
{
		return RTC_BKP_Read(EN_RESET_TYPE_BKP_ADDR);
}


PUBLIC void BKP_Init_weak(void)
{
		// __HAL_RCC_PWR_CLK_ENABLE();
		// HAL_PWR_EnableBkUpAccess();
		// __HAL_RCC_BKPSRAM_CLK_ENABLE();
		// HAL_PWREx_EnableBkUpReg();
	
		// RTC->TAMPCR |= (0xFF << 17);
}

PUBLIC void BKP_Init(void)
{
 /* enable access to RTC registers in Backup domain */
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();	
    rcu_periph_clock_enable(RCU_BKPSRAM);
    rtc_tamper_disable(RTC_TAMPER0); //禁止Tamperx检测功能，防止清空BKP寄存器 
    rtc_tamper_disable(RTC_TAMPER1);
}


PUBLIC void RTC_BKP_Write(UINT32 uiAddr0_19,UINT32 uiDataToWrite)
{
    uint32_t * pAddr0_19 = (uint32_t *)(&RTC_BKP0 + uiAddr0_19);
  
    pmu_backup_write_enable();
    * pAddr0_19 = uiDataToWrite;
    pmu_backup_write_disable();
}

PUBLIC UINT32 RTC_BKP_Read(UINT32 uiAddr0_19)
{
    uint32_t *pAddr0_19 = (uint32_t *)(&RTC_BKP0 + uiAddr0_19);
    
    return (*pAddr0_19);
}

