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
#include "stm32f7xx_hal.h"
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

#define OBJ_NUM (sizeof(ApplicationObjDic)/sizeof(OBJ_ENTRY))
    
#define AXIS_Left_PARAM_EEPROM_ADDR   0
#define AXIS_Right_PARAM_EEPROM_ADDR  256
#define MACHINE_INFO_EEPROM_ADDR      512

#define AXIS_BLOCK_MAX_SIZE               ( 256 )
#define MACHINE_INFO_BLOCK_MAX_SIZE       ( 512 )
#define REDUNDANCY_BLOCK_OFFSET           ( 1024 )

struct ParameterStruct    gParam[2];
struct MachineInfoStruct  gMachineInfo;
struct SoftwareVersionStruct gSoftVersion = {21,0,2};
struct SensorDataStruct gSensorData = {0};

RTC_HandleTypeDef 				hrtc1;

UINT16 EepromErrorFlag=0;
UINT8 MotorVersionParam = 0;

PRIVATE void LoadParamFromEeprom(UINT16 AxisID);
PRIVATE void SaveParamToEeprom(UINT16 AxisID);

PRIVATE void SaveMachineInfo(void);
PRIVATE UINT8 LoadMachineInfo(void);
PRIVATE UINT8 RestoreMachineInfo(void);
PRIVATE UINT8 EraseMachineInfo(void);
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
    
    ErrorLogInit();
    
    gParam[0].ControlWord0x6040 = 6;
    gParam[1].ControlWord0x6040 = 6;
    
    gMachineInfo.encoderPulsePerCircle = (float)(gParam[0].EncoderPPR0x2202>>2);
    gMachineInfo.encoderSampleTimesPerPulse = 4;
    gMachineInfo.reductionRatio = 1;
    gMachineInfo.MotorVersionLast = gMachineInfo.motorVersion;
    
    hrtc1.Instance = RTC;
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
#define DEFAULT_MACHINE_MOTOR_TYPE 3
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
        /*MotorVersion In MachineInfo Changed*/
        if (gMachineInfo.motorVersion != gMachineInfo.MotorVersionLast)
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

    //电机左右两轴坏块检查，发现坏块时，拷贝数据CRC合法的数据至坏块区域
    for( int Index = 0; Index < PARAM_BLOCK_NUM; Index++ )
    {
        if( ( 0 != gParam[Index].EEPROM_GoodBlock ) && ( PARAM_BLOCK_BITS != gParam[Index].EEPROM_GoodBlock ) )
        {
            void SaveParamToEepromBlock( UINT16 AxisID, UINT8 Option );
            void LoadParamFromEepromBlock( UINT16 AxisID, UINT8 Option );

            if( gParam[Index].EEPROM_BlockRetryCnt < PARAM_RETRY_NUM )
            {
                LoadParamFromEepromBlock( Index, gParam[Index].EEPROM_GoodBlock ^ PARAM_BLOCK_BITS );
                if( PARAM_BLOCK_BITS != gParam[Index].EEPROM_GoodBlock ) 
                {
                    gParam[Index].EEPROM_BlockRetryCnt++;
                    SaveParamToEepromBlock( Index, gParam[Index].EEPROM_GoodBlock ^ PARAM_BLOCK_BITS );
                }
                else
                {
                    gParam[Index].EEPROM_BlockRetryCnt = 0;
                }
            }
        }
    }
    
    if(gMachineInfo.RestoreDefault)
    {
        UINT32 len= (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
        UINT32 Tmp = gMachineInfo.batteryVersion;
        memcpy(&gMachineInfo, &gDefaultMachineInfo, len);
        //HLS Robot:Motor Type->3
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
    //machineInfo eeprom 坏块管理,检查是否存在坏块，如果存在，需要将信息完好的数据拷贝到坏块
    else if( PARAM_BLOCK_BITS != gMachineInfo.EEPROM_GoodBlock )
    {
        extern UINT8 LoadMachineInfoBlock( unsigned char Option );
        extern void  SaveMachineInfoBlock( unsigned char Option );

        if( gMachineInfo.EEPROM_BlockRetryCnt < PARAM_RETRY_NUM )
        {
            gMachineInfo.EEPROM_BlockRetryCnt++;
            gMachineInfo.EEPROM_GoodBlock |= LoadMachineInfoBlock( gMachineInfo.EEPROM_GoodBlock ^ PARAM_BLOCK_BITS );
            if( 0 == gMachineInfo.EEPROM_GoodBlock ) 
            {
                UINT32 len = (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
                memcpy(&gMachineInfo,&gDefaultMachineInfo,len+2);
            }
            SaveMachineInfoBlock( gMachineInfo.EEPROM_GoodBlock ^ PARAM_BLOCK_BITS );
        }
    }
    else
    {
        gMachineInfo.EEPROM_BlockRetryCnt = 0;
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
PRIVATE void SaveParamToEepromBlock( UINT16 AxisID, UINT8 Option )
{
    if( AxisID < 2 )
    {
        int i;
        int Index;
        int Address;
        UINT32 len= (UINT32)&gParam[0].EepromCRC - (UINT32)&gParam[0].PositionLimitMin0x2000;
        
        if( 0 == AxisID )
        {
            Index = AXIS_LEFT;
            Address = AXIS_Left_PARAM_EEPROM_ADDR;
        }
        else
        {
            Index = AXIS_RIGHT;
            Address = AXIS_Right_PARAM_EEPROM_ADDR;
        }

        gParam[Index].EepromCRC = GetCRC16((unsigned char*)&gParam[Index], len );
        for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
        {
            if( Option & ( 1 << i ) )
            {
                EEPROM_Serial_Write( Address, (UINT8*)&gParam[Index], len+2 );
            }
        }
    }
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void LoadParamFromEepromBlock(UINT16 AxisID, UINT8 Option )
{
    if( AxisID < 2 )
    {
        int i;
        int n;
        int Address;
        int Index;
        struct ParameterStruct  gParam_tmp;
        UINT32 len = (UINT32)&gParam[AXIS_LEFT].EepromCRC - (UINT32)&gParam[AXIS_LEFT].PositionLimitMin0x2000;
        
        if( 0 == AxisID )
        {
            Index = AXIS_LEFT;
            Address = AXIS_Left_PARAM_EEPROM_ADDR;
        }
        else 
        {
            Index = AXIS_RIGHT;
            Address = AXIS_Right_PARAM_EEPROM_ADDR;
        }
        
        for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
        {
            if( Option & ( 1 << i ) )
            {
                for( n = 0; n < 3; n++ )
                {
                    EEPROM_Serial_Read( Address, (UINT8*)&gParam_tmp, len+2);
                    if( GetCRC16( ( UINT8 * )&gParam_tmp.PositionLimitMin0x2000, len ) == gParam_tmp.EepromCRC )
                    {
                        gParam[Index].EEPROM_GoodBlock |= ( 1 << i );
                        break;
                    }
                    else
                    {
                        gParam[Index].EEPROM_GoodBlock &= ~( 1 << i );
                    }
                }
            }
        }
    }
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SaveParamToEeprom(UINT16 AxisID)
{
    if( AxisID < 2 )
    {
        int i;
        int Index;
        int Address;
        UINT32 len= (UINT32)&gParam[AXIS_LEFT].EepromCRC - (UINT32)&gParam[AXIS_LEFT].PositionLimitMin0x2000;
        
        if( 0 == AxisID )
        {
            Index = AXIS_LEFT;
            Address = AXIS_Left_PARAM_EEPROM_ADDR;
        }
        else
        {
            Index = AXIS_RIGHT;
            Address = AXIS_Right_PARAM_EEPROM_ADDR;
        }

        gParam[Index].EepromCRC = GetCRC16((unsigned char*)&gParam[Index], len );
        for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
        {
            EEPROM_Serial_Write( Address, (UINT8*)&gParam[Index], len+2 );
        }
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
    if( AxisID < 2 )
    {
        int i;
        int n;
        int Index;
        int Address;
        struct ParameterStruct  gParam_tmp;
        UINT32 len = (UINT32)&gParam[AXIS_LEFT].EepromCRC - (UINT32)&gParam[AXIS_LEFT].PositionLimitMin0x2000;
        
        if( 0 == AxisID )
        {
            Index = AXIS_LEFT;
            Address = AXIS_Left_PARAM_EEPROM_ADDR;
        }
        else 
        {
            Index = AXIS_RIGHT;
            Address = AXIS_Right_PARAM_EEPROM_ADDR;
        }

        gParam[Index].EEPROM_GoodBlock = 0;
        gParam[Index].EEPROM_BlockRetryCnt = 0;
        for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
        {
            for( n = 0; n < 3; n++ )
            {
                EEPROM_Serial_Read( Address, (UINT8*)&gParam_tmp, len+2);
                if( GetCRC16( ( UINT8 * )&gParam_tmp.PositionLimitMin0x2000, len ) == gParam_tmp.EepromCRC )
                {
                    memcpy(&gParam[Index], &gParam_tmp, len+2);
                    gParam[Index].EEPROM_GoodBlock |= 1 << i;
                    break;
                }
            }

            if( 0 != gParam[Index].EEPROM_GoodBlock )
            {
                break;
            }
        }

        if( 0 == gParam[Index].EEPROM_GoodBlock )
        {
            EepromErrorFlag |= ( 0x1 << AxisID );
        }
        
        gParam[Index].QuickStopDec0x6085 = gDefaultParam_Left[3].QuickStopDec0x6085;
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
    int i;
    int Address;
    UINT32 len= (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
    
    gMachineInfo.EepromCRC = GetCRC16((unsigned char*)&gMachineInfo, len);
    Address = MACHINE_INFO_EEPROM_ADDR;
    for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
    {
        EEPROM_Serial_Write( Address, (UINT8*)&gMachineInfo, len+2);
    }
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SaveMachineInfoBlock( unsigned char Option )
{
    int i;
    int Address;
    UINT32 len= (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
    
    gMachineInfo.EepromCRC = GetCRC16((unsigned char*)&gMachineInfo, len);
    Address = MACHINE_INFO_EEPROM_ADDR;
    for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
    {
        if( Option & ( 0x01 << i ) )
        {
            EEPROM_Serial_Write( Address, (UINT8*)&gMachineInfo, len+2);
        }
    }
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 LoadMachineInfoBlock( unsigned char Option )
{
    int i;
    int Flag;
    int Address;
    struct MachineInfoStruct MachineInfo_tmp;
    
    UINT32 len = (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;

    Flag = 0;
    Address = MACHINE_INFO_EEPROM_ADDR;
    for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
    {
        if( Option & ( 1 << i ) )
        {
            for( int n = 0; n < 3; n++ )
            {
                EEPROM_Serial_Read( Address, (UINT8*)&MachineInfo_tmp, len+2);
                if( GetCRC16((UINT8*)&MachineInfo_tmp, len) == MachineInfo_tmp.EepromCRC )
                {
                    Flag |= 1 << i;
                    break;
                }
            }
        }
    }

    return( Flag );
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE UINT8 LoadMachineInfo(void)
{
    int i;
    unsigned int Address = 0;
    struct MachineInfoStruct MachineInfo_tmp;
    
    UINT32 len = (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;

    gMachineInfo.EEPROM_GoodBlock = 0;
    gMachineInfo.EEPROM_BlockRetryCnt = 0;
    Address = MACHINE_INFO_EEPROM_ADDR;
    for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += REDUNDANCY_BLOCK_OFFSET )
    {
        int n;
        
        for( n = 0; n < 3; n++ )
        {
            EEPROM_Serial_Read( Address, (UINT8*)&MachineInfo_tmp, len + 2 );
            if( GetCRC16((UINT8*)&MachineInfo_tmp, len) == MachineInfo_tmp.EepromCRC )
            {
                memcpy( &gMachineInfo, &MachineInfo_tmp, len + 2 );
                gMachineInfo.EEPROM_GoodBlock |= 1 << i;
                break;
            }
        }

        if( 0 != gMachineInfo.EEPROM_GoodBlock )
        {
            break;
        }
    }

    if( 0 == gMachineInfo.EEPROM_GoodBlock )
    {
        gMachineInfo.CrcState = 2;
        EepromErrorFlag |= 1<<2;
        return( FALSE );
    }
    else
    {
        return( TRUE );
    }
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
UINT8 EraseMachineInfo(void)
{
		struct MachineInfoStruct MachineInfo_tmp;
		UINT32 len = (UINT32)&gMachineInfo.EepromCRC - (UINT32)&gMachineInfo;
		memset(&MachineInfo_tmp,0xFF,len+2);
    EEPROM_Serial_Write(MACHINE_INFO_EEPROM_ADDR, (UINT8*)&MachineInfo_tmp, len+2);
	return( TRUE );
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


PUBLIC void BKP_Init(void)
{
		__HAL_RCC_PWR_CLK_ENABLE();
		HAL_PWR_EnableBkUpAccess(); // YCHP ADD
		__HAL_RCC_BKPSRAM_CLK_ENABLE();
		HAL_PWREx_EnableBkUpReg();
	
		RTC->TAMPCR |= (0xFF << 17);
 		
}


PUBLIC void RTC_BKP_Write(UINT32 uiAddr0_19,UINT32 uiDataToWrite)
{
		HAL_PWR_EnableBkUpAccess();
		HAL_RTCEx_BKUPWrite(&hrtc1,uiAddr0_19,uiDataToWrite);
		HAL_PWR_DisableBkUpAccess();
}

PUBLIC UINT32 RTC_BKP_Read(UINT32 uiAddr0_19)
{
		return HAL_RTCEx_BKUPRead(&hrtc1,uiAddr0_19);
}

