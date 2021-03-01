/*******************************************************************
 *
 * FILE NAME:  UartApp.h
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

#ifndef _UART_APP_H_
#define _UART_APP_H_

#include "UserDataTypes.h"

#define NO_ERROR      0x4B4F  // OK
#define ERROR_LEN     0x4C45  // EL
#define ERROR_INDEX   0x4945  // EI
#define ERROR_CRC     0x4345  // EC

#define MAX_CHANNEL_NUM  4
#define MAX_COLLECT_DATA_NUM   32760  // 128KByte 

#define NO_TRIGGER             0
#define RISING_EDGE_TRIGGER    1
#define FALLING_EDGE_TRIGGER   2
#define DOUBLE_EDGE_TRIGGER    3

#define SAMPLE_STATE_WAIT      0
#define SAMPLE_STATE_RUN       1
#define SAMPLE_STATE_DONE      2
#define SAMPLE_STATE_ERROR     3

#define UART_RECV_MAX_NUM  1024
#define UART_SEND_MAX_NUM  1024
struct UartAppCommStruct
{
    UINT8 RecvBuf[UART_RECV_MAX_NUM];
    UINT16 RecvBufNum;
    UINT8 SendBuf[UART_SEND_MAX_NUM];
};
struct DataCollectStruct
{
    UINT16  SampleEn;         // enable sample
    UINT16  SampleState;      // sample status
    UINT16  SamplePRD;        // sample priod. unit: pwm period
    UINT16  SampleTotalNum;   // sample total Number.  
    UINT16  ChannelNum;       // total channel number
    UINT32  ReadPtr;          // Data buffer read pointer
    UINT32  WritePtr;         // Data buffer write pointer  
    UINT16  TrigChannel;      // Indicate which channel is trigger channel
    UINT16  TrigMode;         // 0:no trigger, 1:rising edge trigger, 2:falling edge trigger, 3:double edge trigger
    UINT16  PreTrigNum;      // 
    INT32   TrigValue;        // trigger position
    UINT16  SaveNum;
    UINT16  bSendEn;
        
    UINT16  Channel[MAX_CHANNEL_NUM];
    INT32  DataBuffer[MAX_COLLECT_DATA_NUM];    // 

};

extern struct UartAppCommStruct   sUartApp;
extern PUBLIC void UartAppTest(void);
    
extern PUBLIC void UartAppInit(void);  
extern PUBLIC void UartRecvDispatch(UINT8 *pData);

extern PUBLIC void DataCollectingLoop(UINT32 SchTick);
extern PUBLIC void DataCollectSendLoop(void);

extern PUBLIC UINT16 GetCRC16(UINT8 *buf, UINT16 length);

#endif
