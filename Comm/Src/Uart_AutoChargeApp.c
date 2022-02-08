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
#include "gd_hal.h"

#include "USBApp.h"
#include "CanApp.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>


#include "Uart_AutoChargeApp.h"


UINT8 GetSum(UINT8 *buf, UINT16 length)
{
    UINT8 Result =0x00;
    for(UINT8 i =0; i<length; i++ )
    {
        Result += buf[i];
    }
    return Result;
}


/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void Uart_AutoChargeApp_RecvDispatch(UINT8 *data, UINT16 datalen)
{
    /*
       帧头（2bytes） id（2bytes） 数据（8bytes） 数据长度（1byte） 帧尾（2bytes）
        0x57 0x58        id         data                cnt           0xA8 0xA7
    */
    /*
       帧头（2bytes） id（2bytes） 数据（8bytes） 数据长度（1byte） 帧尾（2bytes）
        0x57 0x58        id         data                cnt           0xA8 0xA7
    */

    
    #if 0
    usb2can_frame frame;
    UINT8 CmdType;
    CAN_RX_Message CanRxMessage;
    UINT16 DataSize = datalen;

    frame.header[0] = data[0]; //帧头
    frame.header[1] = data[1];

    frame.tail[0] = data[DataSize-2];//帧尾
    frame.tail[1] = data[DataSize-1];

    frame.can_dlc = data[DataSize-3];//数据长度


    if(frame.header[0] == 0x57 && frame.header[1] == 0x58 \
            && frame.tail[0] == 0xA8 && frame.tail[1] == 0xA7)
    {

        frame.can_id = (data[2]<<8) | data[3];
        memcpy(CanRxMessage.RxData,&data[4],frame.can_dlc);

        CmdType = CanRxMessage.RxData[0];
        switch(CmdType)
        {

        case EN_IAP_CMD:
        {
            USB_send(frame.can_id,CanRxMessage.RxData,8);
        }
        break;
        default:
            break;
        }
    }
    
    #else
    
    USB_send_dirt(data,datalen);
    
    #endif

}