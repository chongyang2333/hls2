/*******************************************************************
 *
 * FILE NAME:  UartApp.c
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
#include "string.h"
#include "ControlRun.h"
#include "UartApp.h"
#include "Param.h"
#include "HardApi.h"
#include "usart.h"
#include "ErrorLog.h"

#define MAX_COLLECT_NUM_PER_FLAME  180

struct UartAppCommStruct   sUartApp={0};
struct  DataCollectStruct  sDataCollect;

PRIVATE void PcWriteSingleParam(UINT8 *buf, UINT16 Len);
PRIVATE void PcWriteParam(UINT8 *buf, UINT16 Len);
PRIVATE void PcReadParam(UINT8 *buf, UINT16 Len);
PRIVATE void PcDataCollect(UINT8 *buf, UINT16 Len);
PRIVATE void PcStopDataCollect(UINT8 *buf, UINT16 Len);
PRIVATE void PcReadErrorLog(UINT8 *buf, UINT16 Len);
PRIVATE void FdbUartError(UINT16 ErrorCode);


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void UartAppInit(void)
{
    ;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void UartRecvDispatch(UINT8 *pData)
{
    HAL_StatusTypeDef Status = HAL_OK;
    UINT16 RecvLen = 0;
    UINT16 CrcData = 0;
    
 
    // return if sending data_collect data
    if(sDataCollect.SampleState == SAMPLE_STATE_DONE)
    {
        return;
    }

    RecvLen = pData[1] | (pData[2]<<8);
    
    if(RecvLen < 8)
    {
        FdbUartError(ERROR_LEN);
        return;
    }
    
    CrcData = pData[RecvLen-1] | (pData[RecvLen]<<8);
    
    UINT16 CrcTmp = GetCRC16(&pData[1], RecvLen-2);
    if(CrcTmp != CrcData || HAL_OK != Status)
    {
        FdbUartError(ERROR_CRC);
        return;
    }
    
    UINT8 Cmd = pData[5];
    switch(Cmd)
    {
        case 0x81 :
            PcWriteSingleParam(&pData[8], RecvLen-9);
            break;
        
        case 0x82 :
            
            break;
        
        case 0x83 :
            
            break;
        
        case 0x84 :
            PcWriteParam(&pData[8], RecvLen-9);
            break;
        
        case 0x85 :
            PcReadParam(&pData[8], RecvLen-9);
            break;
        
        case 0x86 :
            PcDataCollect(&pData[8], RecvLen-9);
            break;
        
        case 0x87 :
            PcStopDataCollect(&pData[8], RecvLen-9);
            break;
        
        case 0x8D :
            PcReadErrorLog(&pData[8], RecvLen-9);
            break;
        
        default :
            break;
    }
    
} 

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void DataCollectingLoop(UINT32 SchTick)
{
    static UINT16 LoopCnt = 0;
    UINT32 u32Tmp = 0;
    INT32  NewValue = 0;
    UINT16 CurTrigLogic = 0;
    static UINT16 OldTrigLogic = 0;

    // return if no sample task
    if(!sDataCollect.SampleEn || sDataCollect.SampleState != SAMPLE_STATE_RUN)
    {
        return;
    }
    
    LoopCnt++;
    if(LoopCnt >= sDataCollect.SamplePRD)
    {
        LoopCnt = 0;
        
        u32Tmp = sDataCollect.WritePtr%MAX_COLLECT_DATA_NUM;
        sDataCollect.DataBuffer[u32Tmp] = SchTick;
        sDataCollect.WritePtr++;
        
        UINT16 i = 0;
        
        for(i = 0;i < sDataCollect.ChannelNum; i++ )
        {    
            u32Tmp = sDataCollect.WritePtr%MAX_COLLECT_DATA_NUM;
            sDataCollect.DataBuffer[u32Tmp] = DataCollectGetValue(sDataCollect.Channel[i]);
            sDataCollect.WritePtr++;
            
            if(i == (sDataCollect.TrigChannel-1))
            {
                NewValue = sDataCollect.DataBuffer[u32Tmp];
            }
        }
        
        sDataCollect.SaveNum += (sDataCollect.ChannelNum+1);
        
        if(NewValue > sDataCollect.TrigValue) 
        {
            CurTrigLogic = 1;
        }
        else
        {
            CurTrigLogic = 0;
        }
    
        // pass the first value
        if(sDataCollect.WritePtr < 5)
        {
            OldTrigLogic = CurTrigLogic;
        }    
        else if(sDataCollect.TrigMode)
        {
            u32Tmp = sDataCollect.WritePtr - sDataCollect.ReadPtr;
            if(u32Tmp > sDataCollect.PreTrigNum)
            {
                sDataCollect.ReadPtr = sDataCollect.ReadPtr + sDataCollect.ChannelNum + 1;
                sDataCollect.SaveNum -= (sDataCollect.ChannelNum+1);
            }                
            
            if(CurTrigLogic == 1 && OldTrigLogic == 0)
            {
                if( (sDataCollect.TrigMode == RISING_EDGE_TRIGGER) 
                    || (sDataCollect.TrigMode == DOUBLE_EDGE_TRIGGER))
                {
                    sDataCollect.TrigMode = 0;
                    sDataCollect.bSendEn = 1;
                }
            }
            else if(CurTrigLogic == 0 && OldTrigLogic == 1)
            {
                if( (sDataCollect.TrigMode == FALLING_EDGE_TRIGGER) 
                    || (sDataCollect.TrigMode == DOUBLE_EDGE_TRIGGER))
                {
                    sDataCollect.TrigMode = 0;
                    sDataCollect.bSendEn = 1;
                }
            }
            
            OldTrigLogic = CurTrigLogic;
            
        }
        else
        {
            sDataCollect.bSendEn = 1;
        }
        
        if(sDataCollect.SaveNum == sDataCollect.SampleTotalNum)
        {
            sDataCollect.SampleState = SAMPLE_STATE_DONE;
            sDataCollect.SampleEn = 0;
        }
        else if((sDataCollect.SaveNum > sDataCollect.SampleTotalNum) || (sDataCollect.SaveNum >  MAX_COLLECT_DATA_NUM) )         
        {
            sDataCollect.SampleState = SAMPLE_STATE_ERROR;
            sDataCollect.SampleEn = 0;
        }
        
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void DataCollectSendLoop(void)
{
    UINT16 length = 0;
    UINT16 DataEntry = 0;
    UINT16 CrcData = 0;
    UINT32 u32Tmp = 0;
    UINT8 *pSendBuf = sUartApp.SendBuf;
    if(huart3.Instance->ISR & 0xF)
    {    
        huart3.Instance->ICR = 0xF;
    }
   
    if(!sDataCollect.bSendEn) return;

    while(1)
    {
        UINT32 TmpNum = sDataCollect.WritePtr - sDataCollect.ReadPtr;
        
        if(TmpNum > MAX_COLLECT_NUM_PER_FLAME)
        {
            DataEntry = 9;
            // feedback
            pSendBuf[0] = 0xAA;     // start flag
            pSendBuf[3]= 0x01;      // protocol version
            pSendBuf[4]= 0x00;      // device address
            pSendBuf[5]= 0x06;      // function code
            pSendBuf[8]= 0x01; 
            
            u32Tmp = sDataCollect.ReadPtr%MAX_COLLECT_DATA_NUM;
            
            UINT32 remain_num = MAX_COLLECT_DATA_NUM - u32Tmp;
            
            if(remain_num >= MAX_COLLECT_NUM_PER_FLAME)
            {
                memcpy(&pSendBuf[DataEntry], 
                       &sDataCollect.DataBuffer[u32Tmp], 
                       MAX_COLLECT_NUM_PER_FLAME*4); 
            }
            else
            {
                memcpy(&pSendBuf[DataEntry], 
                       &sDataCollect.DataBuffer[u32Tmp], 
                       remain_num*4);
                
                memcpy(&pSendBuf[DataEntry+remain_num*4], 
                       &sDataCollect.DataBuffer[0], 
                       (MAX_COLLECT_NUM_PER_FLAME - remain_num)*4);
                
            }
                 
            DataEntry = DataEntry + MAX_COLLECT_NUM_PER_FLAME*4;
            sDataCollect.ReadPtr = sDataCollect.ReadPtr + MAX_COLLECT_NUM_PER_FLAME;  

            length = DataEntry+1;
            pSendBuf[1] = length;
            pSendBuf[2] = length>>8;  

            CrcData = GetCRC16(&pSendBuf[1],length-2);
            pSendBuf[length-1] = CrcData;
            pSendBuf[length] = CrcData>>8;
            
            UartSendData(pSendBuf, length+1); 
        
        }
        else if(TmpNum > 0 && sDataCollect.SampleState == SAMPLE_STATE_DONE)
        {
            DataEntry = 9;
            // feedback
            pSendBuf[0] = 0xAA;     // start flag
            pSendBuf[3]= 0x01;      // protocol version
            pSendBuf[4]= 0x00;      // device address
            pSendBuf[5]= 0x06;      // function code
            pSendBuf[8]= 0x02; 
            
            u32Tmp = sDataCollect.ReadPtr%MAX_COLLECT_DATA_NUM;
            
            UINT32 remain_num = MAX_COLLECT_DATA_NUM - u32Tmp;
            
            if(remain_num >= TmpNum)
            {
                memcpy(&pSendBuf[DataEntry], 
                       &sDataCollect.DataBuffer[u32Tmp], 
                       TmpNum*4); 
            }
            else 
            {
                memcpy(&pSendBuf[DataEntry], 
                       &sDataCollect.DataBuffer[u32Tmp], 
                       remain_num*4);
                
                memcpy(&pSendBuf[DataEntry+remain_num*4], 
                       &sDataCollect.DataBuffer[0], 
                       (TmpNum - remain_num)*4);
            }
            
            sDataCollect.ReadPtr = sDataCollect.ReadPtr + TmpNum;
            DataEntry = DataEntry + TmpNum*4;   
            
            sDataCollect.SampleState = SAMPLE_STATE_WAIT;
            sDataCollect.ReadPtr = 0;
            sDataCollect.WritePtr = 0;
            
            
            length = DataEntry+1;
            pSendBuf[1] = length;
            pSendBuf[2] = length>>8;  

            CrcData = GetCRC16(&pSendBuf[1],length-2);
            pSendBuf[length-1] = CrcData;
            pSendBuf[length] = CrcData>>8;
            
            UartSendData(pSendBuf, length+1); 
            
            sDataCollect.bSendEn = 0;
            sDataCollect.SaveNum = 0;
            
            return;
        }
        else
        {
            return;
        }
        

        
    }

}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PcWriteSingleParam(UINT8 *buf, UINT16 Len)
{
    UINT16 result = NO_ERROR;
	UINT16 CrcData, ParamLen;
	UINT16 Index, SubIndex;
    UINT8 *pSendBuf = sUartApp.SendBuf;

    OBJ_ENTRY * pObject = NULL;
	UINT32 *pVarPtr = 0;

    Index = buf[4] | (buf[5]<<8);
    SubIndex = buf[2] | (buf[3]<<8);

	pObject = OBJ_GetObjectHandle(Index);
        
    if(pObject != NULL)
    {
        ParamLen = pObject->BitLength/8;

        if(ParamLen == (buf[0]-6))
        {
            pVarPtr = (UINT32 *) ((UINT8 *)pObject->pVarPtr);
            memcpy(pVarPtr ,&buf[6], ParamLen);
        }
        else
        {
            result =ERROR_LEN;  // error param len
            
        }

    }
    else
    {
        result= ERROR_INDEX; // error param index
    
    }

	// feedback
	UINT16 length = 16;
    pSendBuf[0] = 0xAA;
 
	pSendBuf[3]= 0x01;   // protocol version
	pSendBuf[4]= 0x00;   // device address
	pSendBuf[5]= 0x01;   // function code
	pSendBuf[8]= 0x07;   // length.
    pSendBuf[9]= SubIndex;
    pSendBuf[10]= SubIndex>>8;
    pSendBuf[11]= Index;
    pSendBuf[12]= Index>>8;
    pSendBuf[13]= result;
    pSendBuf[14]= result>>8;

	CrcData = GetCRC16(&pSendBuf[1],length-2);
    pSendBuf[length-1] = CrcData;
    pSendBuf[length] = CrcData>>8;
    
    UartSendData(pSendBuf, length+1);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PcWriteParam(UINT8 *buf, UINT16 Len)
{
    UINT16 result = NO_ERROR;
	UINT16 CrcData, ParamLen;
	UINT16 ParamEntry;
	UINT8  ParamNum;
	UINT16 Index, SubIndex;
    UINT8 *pSendBuf = sUartApp.SendBuf;

    OBJ_ENTRY * pObject = NULL;
	UINT32 *pVarPtr = 0;

	ParamEntry = 2;   // singal param len buf[]
	ParamNum = buf[1]; // param number

	while(ParamNum)
	{
        Index = buf[ParamEntry+4] | (buf[ParamEntry+5]<<8);
        SubIndex = buf[ParamEntry+2] | (buf[ParamEntry+3]<<8);

		pObject = OBJ_GetObjectHandle(Index);
        
		if(pObject != NULL)
		{
			ParamLen = pObject->BitLength/8;

            if(pObject->ObjAccess != ACCESS_READ_ONLY)
            {
                if(ParamLen == (buf[ParamEntry]-6))
                {
                    pVarPtr = (UINT32 *) ((UINT8 *)pObject->pVarPtr);
                    memcpy(pVarPtr ,&buf[ParamEntry+6], ParamLen);
                }
                else
                {
                    result =ERROR_LEN;  // error param len
                    break;
                }
            }
		}
		else
		{
			result= ERROR_INDEX; // error param index
			break;
		}

		ParamEntry += buf[ParamEntry];
		ParamNum--;
	}

	// feedback
	UINT16 length = 16;
    pSendBuf[0] = 0xAA;
    pSendBuf[1] = length;
    pSendBuf[2] = length>>8;  
	pSendBuf[3]= 0x01;   // protocol version
	pSendBuf[4]= 0x00;   // device address
	pSendBuf[5]= 0x04;   // function code
	pSendBuf[8]= 0x07;   // length
    pSendBuf[9]= result;
    pSendBuf[10]= result>>8;
    pSendBuf[11]= SubIndex;
    pSendBuf[12]= SubIndex>>8;
    pSendBuf[13]= Index;
    pSendBuf[14]= Index>>8;

	CrcData = GetCRC16(&pSendBuf[1],length-2);
    pSendBuf[length-1] = CrcData;
    pSendBuf[length] = CrcData>>8;
    
    UartSendData(pSendBuf, length+1); 
   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PcReadParam(UINT8 *buf, UINT16 Len)
{
	UINT16 CrcData, ParamLen;
	UINT16 ParamEntry;
	UINT8  ParamNum;
	UINT16 Index, SubIndex;
    UINT8 *pSendBuf = sUartApp.SendBuf;
    OBJ_ENTRY * pObject = NULL;
	UINT32 *pVarPtr = 0;
  
    // Can not read more than 20 parameters per frame    
    if(Len > 81) // 81=20*4+1
    {
        FdbUartError(ERROR_LEN);
        return;
    }
    
    ParamNum = buf[0];
	pSendBuf[8]= 2;          // feedback start/end flag
	pSendBuf[9]= ParamNum;   // feedback ParamNum
	ParamEntry = 10;      

    UINT8 tmp = 0;
    while(ParamNum)
	{
		tmp = (buf[0]-ParamNum)*4;
        Index = buf[tmp+3] | buf[tmp+4]<<8;
        SubIndex = buf[tmp+1] | buf[tmp+2]<<8;

		pObject = OBJ_GetObjectHandle(Index);
		if(pObject != NULL)
		{
			ParamLen = pObject->BitLength/8;

            pVarPtr = (UINT32 *) ((UINT8 *)pObject->pVarPtr);

			pSendBuf[ParamEntry] = ParamLen+6;
			pSendBuf[ParamEntry+1] = (UINT8)pObject->DataType;
            pSendBuf[ParamEntry+2] = SubIndex;
            pSendBuf[ParamEntry+3] = SubIndex>>8;
            pSendBuf[ParamEntry+4] = Index;
            pSendBuf[ParamEntry+5] = Index>>8;
			memcpy(&pSendBuf[ParamEntry+6], pVarPtr, ParamLen);
			ParamEntry = ParamEntry+ParamLen+6;
		}
		else
		{
			FdbUartError(ERROR_INDEX);   // error param index
			return ;
		}
		ParamNum--;
	}
  	// feedback
    pSendBuf[0]=0xAA;
	pSendBuf[3]= 0x01;   // protocol version
	pSendBuf[4]= 0x00;   // device address
	pSendBuf[5]= 0x05;   // function code

	UINT16 length = ParamEntry+1;
    pSendBuf[1] = length;
    pSendBuf[2] = length>>8;

	CrcData = GetCRC16(&pSendBuf[1],length-2);
    pSendBuf[length-1] = CrcData;
    pSendBuf[length] = CrcData>>8;
    
    UartSendData(pSendBuf, length+1); 
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PcDataCollect(UINT8 *buf, UINT16 Len)
{
    UINT32 u32Tmp;
    UINT16 u16Tmp;

	//init paramters
    sDataCollect.TrigMode = buf[0];
    sDataCollect.TrigChannel = buf[1]; 
    
    memcpy(&sDataCollect.TrigValue, &buf[4], 4);
    
    memcpy(&u32Tmp, &buf[8], 4);
    sDataCollect.SamplePRD = u32Tmp/100;
        
    if(sDataCollect.SamplePRD ==0) 
        sDataCollect.SamplePRD =1;
    
    sDataCollect.ChannelNum = buf[16];
    
    memcpy(&u32Tmp, &buf[12], 4);
    u32Tmp = u32Tmp*10/sDataCollect.SamplePRD;  // = (ms*1000/100)/PRD
    sDataCollect.SampleTotalNum = u32Tmp*(sDataCollect.ChannelNum+1);

    u16Tmp = buf[2] | (buf[3]<<8);
    sDataCollect.PreTrigNum = sDataCollect.SampleTotalNum*u16Tmp/100;
    
    
    
    UINT16 i=0;
    for(i=0; i<sDataCollect.ChannelNum; i++)
    {   
        memcpy(&u32Tmp, &buf[17+4*i], 4);
        sDataCollect.Channel[i] = u32Tmp>>16; // index
    }
    
    sDataCollect.ReadPtr = 0;
    sDataCollect.WritePtr = 0;
    sDataCollect.SampleState = SAMPLE_STATE_RUN;
    sDataCollect.bSendEn = 0;
    sDataCollect.SaveNum = 0;
    sDataCollect.SampleEn = 1;  
    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PcStopDataCollect(UINT8 *buf, UINT16 Len)
{

    UINT16 result = NO_ERROR;
 	UINT16 CrcData;
    UINT8 *pSendBuf = sUartApp.SendBuf;
    
    sDataCollect.SampleEn = 0;
    sDataCollect.SampleState = SAMPLE_STATE_WAIT;
    sDataCollect.ReadPtr = 0;
    sDataCollect.WritePtr = 0;

	UINT16 length = 11;
    pSendBuf[0] = 0xAA;
    pSendBuf[1] = length;
    pSendBuf[2] = length>>8;  
	pSendBuf[3]= 0x01;   // protocol version
	pSendBuf[4]= 0x00;   // device address
	pSendBuf[5]= 0x07;   // function code
    pSendBuf[8] = result;
    pSendBuf[9] = result>>8;
	CrcData = GetCRC16(&pSendBuf[1],length-2);
    pSendBuf[length-1] = CrcData;
    pSendBuf[length] = CrcData>>8;
    UartSendData(pSendBuf, length+1);    
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void PcReadErrorLog(UINT8 *buf, UINT16 Len)
{
 	UINT16 CrcData;
    UINT8 *pSendBuf = sUartApp.SendBuf;

    UINT16 ErrLogNum = ReadErrorLog(&pSendBuf[8]);
    
	UINT16 length = ErrLogNum*12+9;  // 80*12 + 9
    pSendBuf[0] = 0xAA;
    pSendBuf[1] = length;
    pSendBuf[2] = length>>8;  
	pSendBuf[3]= 0x01;   // protocol version
	pSendBuf[4]= 0x00;   // device address
	pSendBuf[5]= 0x0D;   // function code
    
	CrcData = GetCRC16(&pSendBuf[1],length-2);
    pSendBuf[length-1] = CrcData;
    pSendBuf[length] = CrcData>>8;
    UartSendData(pSendBuf, length+1);   
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PRIVATE void FdbUartError(UINT16 ErrorCode)
{
	UINT16 CrcData = 0;
    UINT8 *pSendBuf = sUartApp.SendBuf;

	UINT16 length = 11;
    pSendBuf[0] = 0xAA;
    pSendBuf[1] = length;
    pSendBuf[2] = length>>8;  
	pSendBuf[3]= 0x01;   // protocol version
	pSendBuf[4]= 0x00;   // device address
	pSendBuf[5]= 0x7E;   // function code
    pSendBuf[8] = ErrorCode;
    pSendBuf[9] = ErrorCode>>8;

	CrcData = GetCRC16(&pSendBuf[1],length-2);
    pSendBuf[length-1] = CrcData;
    pSendBuf[length] = CrcData>>8;
    
    UartSendData(pSendBuf, length+1); 
}

/*********************************************************************************************************
 CRC16
*********************************************************************************************************/
#define CRC_SEED 0xFFFF
#define POLY16 0x1021

PRIVATE UINT8 auchCRCHi[] =
{
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40
};

PRIVATE UINT8 auchCRCLo[] =
{
	0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,
	0x07,0xC7,0x05,0xC5,0xC4,0x04,0xCC,0x0C,0x0D,0xCD,
	0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
	0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,
	0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,0x14,0xD4,
	0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
	0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,
	0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
	0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
	0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,
	0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,
	0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
	0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,
	0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,
	0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
	0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
	0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,
	0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
	0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,
	0x70,0xB0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
	0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
	0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,
	0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4B,0x8B,
	0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
	0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,
	0x43,0x83,0x41,0x81,0x80,0x40
};

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT16 GetCRC16(UINT8 *buf, UINT16 length)
{
	UINT8  uchCRCHi = 0xFF ;
	UINT8  uchCRCLo = 0xFF ;
	UINT16 uIndex ;
	while (length--)
	{
		uIndex = uchCRCHi ^ *buf++ ;
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex] ;
	}
	return ((uchCRCHi << 8) | uchCRCLo);
}
