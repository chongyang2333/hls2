/*******************************************************************
 *
 * FILE NAME:  ErrorLog.h
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

#ifndef _ERROR_LOG_H_
#define _ERROR_LOG_H_

#include "UserDataTypes.h"

struct ErrorLogStruct
{
    UINT16  WritePointer;
    UINT16  ErrLogNum;
    
    UINT32  TimeStamp;
    UINT32  LeftErrCode;
    UINT32  RightErrCode;

    UINT8   SaveErrLogEn;
};

extern struct ErrorLogStruct sErrLog;

extern PUBLIC void ErrorLogInit(void);
extern PUBLIC void ErrorLogExec(UINT32 LeftErr, UINT32 RightErr);
extern PUBLIC void ClearErrorLog(void);
extern PUBLIC UINT16 ReadErrorLog(UINT8 *pData);

#endif
