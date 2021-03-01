/*******************************************************************
 *
 * FILE NAME:  ErrorLog.c
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2019.01.15
 *
 * AUTHOR:      Denny
 *
 * History:
------------------------------------------------------------------------
15-01-2019 Version 1.00 : Created by Denny
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/

#include "ErrorLog.h"
#include "Eeprom.h"
#include "Param.h"

#define ERR_LOG_POINTER_EE_ADDR   1024    // TODO
#define ERR_LOG_INFO_EE_ADDR      (ERR_LOG_POINTER_EE_ADDR+4)    

#define ERR_LOG_MAX_NUM           80

struct ErrorLogStruct sErrLog={0};

PRIVATE void ReadErrorLogPointer(void);
PRIVATE void SaveErrorLog(void);
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ErrorLogInit(void)
{
    ReadErrorLogPointer();
}


/***********************************************************************
 * DESCRIPTION: 40HZ
 *
 * RETURNS:
 *

***********************************************************************/
PUBLIC void ErrorLogExec(UINT32 LeftErr, UINT32 RightErr)
{
    if(sErrLog.SaveErrLogEn)
    {
        sErrLog.TimeStamp = gParam[0].TimeStamp0x2500;  
        sErrLog.LeftErrCode = LeftErr;
        sErrLog.RightErrCode = RightErr;
        SaveErrorLog();
        sErrLog.SaveErrLogEn = 0;
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT16 ReadErrorLog(UINT8 *pData)
{
    UINT16 TmpAddr = 0;
    
    EEPROM_Serial_Read(ERR_LOG_POINTER_EE_ADDR, (UINT8*)&sErrLog.WritePointer, 4);
    
    UINT16 ReadPtr = (sErrLog.WritePointer + ERR_LOG_MAX_NUM - sErrLog.ErrLogNum)%ERR_LOG_MAX_NUM;
    
    for(int i=0;i<sErrLog.ErrLogNum;i++)
    {
        TmpAddr = ERR_LOG_INFO_EE_ADDR + ReadPtr*4*3;
        EEPROM_Serial_Read(TmpAddr, (UINT8*)(pData+i*12), 12);
        ReadPtr = (ReadPtr +1)%ERR_LOG_MAX_NUM;
    }

    if(sErrLog.ErrLogNum>80) 
        sErrLog.ErrLogNum = 80;
    
    return sErrLog.ErrLogNum;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void ReadErrorLogPointer(void)
{
    EEPROM_Serial_Read(ERR_LOG_POINTER_EE_ADDR, (UINT8*)&sErrLog.WritePointer, 4);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void SaveErrorLog(void)
{
    UINT16 TmpAddr = ERR_LOG_INFO_EE_ADDR + sErrLog.WritePointer*4*3;
    EEPROM_Serial_Write(TmpAddr, (UINT8*)&sErrLog.TimeStamp, 12);
    
    sErrLog.WritePointer = (sErrLog.WritePointer+1)%ERR_LOG_MAX_NUM;
    
    sErrLog.ErrLogNum++;
    if(sErrLog.ErrLogNum > ERR_LOG_MAX_NUM)
    {
        sErrLog.ErrLogNum = ERR_LOG_MAX_NUM;
    }
      
    EEPROM_Serial_Write(ERR_LOG_POINTER_EE_ADDR, (UINT8*)&sErrLog.WritePointer, 4);   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ClearErrorLog(void)
{
    UINT8 Data[4]={0};
    
    EEPROM_Serial_Write(ERR_LOG_POINTER_EE_ADDR, (UINT8*)Data, 4);  
    sErrLog.ErrLogNum = 0;
    sErrLog.WritePointer = 0;

}
