/***************************************************************************************************
file:
funciton:
auther:
***************************************************************************************************/
/***************************************************************************************************

***************************************************************************************************/
#define  __MACHINE_ADDITIONAL_INFO_C__
#include <stdlib.h>
#include "gd32f4xx.h"
#include "UserDataTypes.h"
#include "Param.h"
#include "UartApp.h"
#include "Eeprom.h"
#include "CanApp.h"
#include "PowerManager.h"
#include "MachineAdditionalInfo.h"
#undef   __MACHINE_ADDITIONAL_INFO_C__
/**************************************************************************************************/
/***************************************************************************************************

***************************************************************************************************/
unsigned long OdometerMark;
unsigned long OdometerCounter;
unsigned long CumulativeTimeBase;

STRUCT_WHEEL_MILEAGE   WheelsMileage[2];
STRUCT_ADDITIONAL_INFO MachineAddInfo;
STRUCT_ADDITIONAL_INFO_HEADER MachineAddHeader;
STRUCT_ADDITIONAL_INFO_ACCESS MachineAddAccess;

extern struct PowerManagerStruct sPowerManager;
extern uint8_t ReadChargeInPinState(void);

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
const STRUCT_ADDITIONAL_INFO_HEADER MachineAddHeaderDefault =
{
    .InfoSize = sizeof( STRUCT_ADDITIONAL_INFO ),
    .InfoAddress = MACHINE_ADD_INFO_ADDR + sizeof( STRUCT_ADDITIONAL_INFO_HEADER ),
};

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
const STRUCT_ADDITIONAL_INFO MachineAddInfoDefault =
{
    .Odometer = { .ItemSize = sizeof( STRUCT_ADDITIONAL_INFO_ITEM ), .Keyword = "Odometer", },
    .CumulativeTime = { .ItemSize = sizeof( STRUCT_ADDITIONAL_INFO_ITEM ), .Keyword = "Cumu. Time", },
};

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void CalculateWheelMileage( STRUCT_WHEEL_MILEAGE *pWheel, double WheelPerimeter, unsigned int PlusCntPerCircle )
{
    double Mileage;
    unsigned long Value;

    if( !pWheel || ( 0 == PlusCntPerCircle ) || ( ( WheelPerimeter > -1e-6 ) && ( WheelPerimeter < 1e-6 ) ) )
    {
        return;
    }

    PlusCntPerCircle /= 4;
    if( pWheel->EnconderPlusCnt >= PlusCntPerCircle )
    {
        Value = pWheel->EnconderPlusCnt / PlusCntPerCircle;
        pWheel->EnconderPlusCnt %= PlusCntPerCircle;
        Mileage = Value * WheelPerimeter / 4;
        pWheel->Mileage += Mileage;
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void UpdateWheelEncoderPlusCount( STRUCT_WHEEL_MILEAGE *pWheel, signed int DeltaPlusCnt )
{
    if( !pWheel )
    {
        return;
    }

    if( DeltaPlusCnt < 0 )
    {
        DeltaPlusCnt = -DeltaPlusCnt;
    }

    /*1rpm = 4096Inc/60*40tick = 1.7Inc/tick;  1tick = 25ms*/
    /*remove encoder jetter err*/
    if (DeltaPlusCnt < 3)
    {
        DeltaPlusCnt = 0;
    }

    pWheel->EnconderPlusCnt += DeltaPlusCnt;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void CumulativeTimeCount( void )
{
    static unsigned char ChargeState = 0;
    static unsigned char LowLevelFilterCnt = 0;
    static unsigned char HighLevelFilterCnt = 0;

    if( 2 != ChargeState )
    {
        if( ReadChargeInPinState() )
        {
            LowLevelFilterCnt = 0;
            HighLevelFilterCnt++;
            if( HighLevelFilterCnt >= 5 )
            {
                ChargeState = 2;
                HighLevelFilterCnt = 0;
            }
        }
        else
        {
            HighLevelFilterCnt = 0;
            if( 0 == ChargeState )
            {
                LowLevelFilterCnt++;
                if( LowLevelFilterCnt >= 5 )
                {
                    ChargeState = 1;
                    LowLevelFilterCnt = 0;
                }
            }
        }
    }
    else
    {
        if( ReadChargeInPinState() )
        {
            LowLevelFilterCnt = 0;
        }
        else
        {
            HighLevelFilterCnt = 0;
            LowLevelFilterCnt++;
            if( LowLevelFilterCnt >= 5 )
            {
                ChargeState = 1;
                LowLevelFilterCnt = 0;
            }
        }
    }

    if( ( 1 == ChargeState ) && ( DISCONNECT == sPowerManager.sChargeInfo.eChargerConnectState ) )
    {
        CumulativeTimeBase++;
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
unsigned char SaveMachineAddInfoHeader( void )
{
    int i;
    unsigned int Address;

    i = MachineAddInfo.RecordCounter / MACHINE_ADD_INFO_MAX_RECORD_NUM;
    if( i >= MACHINE_ADD_INFO_AREA_NUM )
    {
        i = MACHINE_ADD_INFO_AREA_NUM - 1;
    }
    Address = MachineAddHeaderDefault.InfoAddress + i * sizeof( MachineAddInfo );
    if( MachineAddAccess.IsInfoInvalid || ( Address != MachineAddHeader.InfoAddress ) || ( sizeof( MachineAddInfo ) != MachineAddHeader.InfoSize ) )
    {
        MachineAddAccess.GoodHeaderBlock = 0;
        MachineAddHeader.InfoAddress = Address;
        MachineAddHeader.InfoSize = sizeof( MachineAddInfo );
        MachineAddHeader.HeaderCRC = GetCRC16( ( unsigned char * )&MachineAddHeader,  sizeof( MachineAddHeader ) - sizeof( MachineAddHeader.HeaderCRC ) );
        Address = MACHINE_ADD_INFO_ADDR;
        for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += MACHINE_ADD_INFO_REDUNDANCY_OFFSET )
        {
            EEPROM_Serial_Write( Address, ( unsigned char *)&MachineAddHeader, sizeof( MachineAddHeader ) );
        }

        return( 1 );
    }
    else
    {
        return( 0 );
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
unsigned char CheckMachineAddInfoHeader( void )
{
    int i;
    unsigned int  Address;
    unsigned char Flag;

    Flag = 0;
    Address = MACHINE_ADD_INFO_ADDR;
    for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += MACHINE_ADD_INFO_REDUNDANCY_OFFSET )
    {
        int n;

        /*read add. info header*/
        for( n = 0; n < 3; n++ )
        {
            STRUCT_ADDITIONAL_INFO_HEADER Header;

            EEPROM_Serial_Read( Address, (  unsigned char * )&Header, sizeof( Header ) );
            if( GetCRC16( ( unsigned char * )&Header, sizeof( Header ) - sizeof( Header.HeaderCRC ) ) == Header.HeaderCRC )
            {
                Flag |= 1 << i;
                break;
            }
        }
    }

    if( PARAM_BLOCK_BITS == Flag )
    {
        return( TRUE );
    }
    else
    {
        return( FALSE );
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
unsigned char SaveMachineAddInfo( void )
{
    int i;
    unsigned int Address;
    STRUCT_ADDITIONAL_INFO Info;

    DI();
    Info = MachineAddInfo;
    EI();

    MachineAddAccess.GoodInfoBlock = 0;
    Address = MachineAddHeader.InfoAddress;
    Info.BlockCRC = GetCRC16( ( unsigned char * )&Info, sizeof( Info ) - sizeof( Info.BlockCRC ) );
    for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += MACHINE_ADD_INFO_REDUNDANCY_OFFSET )
    {
        EEPROM_Serial_Write( Address, ( unsigned char *)&Info, sizeof( Info ) );
    }

    return( TRUE );
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
unsigned char LoadMachineAddInfo( void )
{
    int i;
    unsigned int   Address = 0;
    unsigned char  *pBuff = 0;

    MachineAddAccess.IsInfoInvalid = 0;
    MachineAddAccess.BlockRetryCnt = 0;
    MachineAddAccess.GoodInfoBlock = 0;
    MachineAddAccess.GoodHeaderBlock = 0;
    MachineAddInfo = MachineAddInfoDefault;
    MachineAddHeader = MachineAddHeaderDefault;
    Address = MACHINE_ADD_INFO_ADDR;
    for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += MACHINE_ADD_INFO_REDUNDANCY_OFFSET )
    {
        for( int n = 0; n < 3; n++ )
        {
            STRUCT_ADDITIONAL_INFO_HEADER Header;

            EEPROM_Serial_Read( Address, (  unsigned char * )&Header, sizeof( Header ) );
            if( GetCRC16( ( unsigned char * )&Header, sizeof( Header ) - sizeof( Header.HeaderCRC ) ) == Header.HeaderCRC )
            {
                MachineAddAccess.GoodHeaderBlock |= 1 << i;
                MachineAddHeader = Header;
                break;
            }
        }

        if( 0 != MachineAddAccess.GoodHeaderBlock )
        {
            break;
        }
    }

    if( 0 == MachineAddAccess.GoodHeaderBlock )
    {
        MachineAddAccess.IsInfoInvalid = TRUE;
        return( FALSE );
    }

    Address = MachineAddHeader.InfoAddress;
    pBuff = ( unsigned char * )malloc( MachineAddHeader.InfoSize );
    if( pBuff )
    {
        for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += MACHINE_ADD_INFO_REDUNDANCY_OFFSET )
        {
            for( int n = 0; n < 3; n++ )
            {
                unsigned short BlockCRC;

                EEPROM_Serial_Read( Address, pBuff, MachineAddHeader.InfoSize );
                BlockCRC = *( ( unsigned short * )&pBuff[MachineAddHeader.InfoSize - sizeof( MachineAddInfo.BlockCRC )] );
                if( GetCRC16( pBuff, MachineAddHeader.InfoSize - sizeof( MachineAddInfo.BlockCRC ) ) == BlockCRC )
                {
                    MachineAddInfo = *( ( STRUCT_ADDITIONAL_INFO * )pBuff );
                    MachineAddAccess.GoodInfoBlock |= 1 << i;
                    break;
                }
            }

            if( 0 != MachineAddAccess.GoodInfoBlock )
            {
                break;
            }
        }
    }

    free( pBuff );

    if( 0 == MachineAddAccess.GoodInfoBlock )
    {
        MachineAddAccess.IsInfoInvalid = TRUE;
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
unsigned char CheckMachineAddInfo( void )
{
    int i;
    int Address;
    int Flag;
    STRUCT_ADDITIONAL_INFO *pInfo;

    Flag = 0;
    Address = MachineAddHeader.InfoAddress;
    pInfo = ( STRUCT_ADDITIONAL_INFO * )malloc( MachineAddHeader.InfoSize );
    if( pInfo )
    {
        for( i = 0; i < PARAM_BLOCK_NUM; i++, Address += MACHINE_ADD_INFO_REDUNDANCY_OFFSET )
        {
            int n;

            for( n = 0; n < 3; n++ )
            {
                EEPROM_Serial_Read( Address, ( unsigned char * )pInfo, MachineAddHeader.InfoSize );
                if( GetCRC16( ( unsigned char * )pInfo, sizeof( *pInfo ) - sizeof( pInfo->BlockCRC ) ) == pInfo->BlockCRC )
                {
                    Flag |= 1 << i;
                    break;
                }
            }
        }
    }

    free( pInfo );

    if( 0 == Flag )
    {
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
unsigned char IsMachineAddInfoSaveOK( void )
{
    if( INFO_ACCESS_NONE == MachineAddAccess.Status )
    {
        return( TRUE );
    }
    else
    {
        return( FALSE );
    }
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void ResetOdometer( void )
{
    MachineAddAccess.Status = INFO_ACCESS_RESET_ODOMETER;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void ResetCumulativeTime( void )
{
    MachineAddAccess.Status = INFO_ACCESS_RESET_TIME;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void EnableMachineAddInfoSave( void )
{
    MachineAddAccess.Status = INFO_ACCESS_SAVE;
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void MachineAddInfoProcess( void )
{
    int i;

    if( INFO_ACCESS_WRITE_HEADER == MachineAddAccess.Status )
    {
        i = SaveMachineAddInfoHeader();
        if( 1 == i )
        {
            MachineAddAccess.Status = INFO_ACCESS_CHECK_HEADER;
        }
        else if( 0 == i )
        {
            MachineAddAccess.Status = INFO_ACCESS_WRITE_INFO;
        }
    }
    else if( INFO_ACCESS_CHECK_HEADER == MachineAddAccess.Status )
    {
        if( CheckMachineAddInfoHeader() )
        {
            MachineAddAccess.IsInfoInvalid = 0;
            MachineAddAccess.HeaderRetryCnt = 0;
            MachineAddAccess.Status = INFO_ACCESS_WRITE_INFO;
        }
        else
        {
            MachineAddAccess.HeaderRetryCnt++;
            if( MachineAddAccess.HeaderRetryCnt >= PARAM_RETRY_NUM )
            {
                MachineAddAccess.IsInfoInvalid = 0;
                MachineAddAccess.HeaderRetryCnt = 0;
                MachineAddAccess.Status = INFO_ACCESS_NONE;
            }
            else
            {
                MachineAddAccess.Status = INFO_ACCESS_WRITE_HEADER;
            }
        }
    }
    else if( INFO_ACCESS_WRITE_INFO == MachineAddAccess.Status )
    {
        if( MachineAddAccess.BlockRetryCnt < PARAM_RETRY_NUM )
        {
            if( SaveMachineAddInfo() )
            {
                MachineAddAccess.Status = INFO_ACCESS_CHECK_INFO;
            }
            else
            {
                MachineAddInfo.RecordCounter++;
                MachineAddAccess.BlockRetryCnt++;
                MachineAddAccess.Status = INFO_ACCESS_WRITE_HEADER;
            }
        }
        else
        {
            MachineAddAccess.Status = INFO_ACCESS_NONE;
        }
    }
    else if( INFO_ACCESS_CHECK_INFO == MachineAddAccess.Status )
    {
        if( CheckMachineAddInfo() )
        {
            MachineAddAccess.Status = INFO_ACCESS_NONE;
        }
        else
        {
            MachineAddInfo.RecordCounter++;
            MachineAddAccess.BlockRetryCnt++;
            MachineAddAccess.Status = INFO_ACCESS_WRITE_HEADER;
        }
    }
    else
    {
        if( ( MachineAddInfo.Odometer.Value != gMachineInfo.Odometer ) || ( MachineAddInfo.CumulativeTime.Value != gMachineInfo.CumulativeTime ) )
        {
            /*if PC software modife the parameters, save the parameters to eeprom.*/
            MachineAddInfo.Odometer.Value = gMachineInfo.Odometer;
            MachineAddInfo.CumulativeTime.Value = gMachineInfo.CumulativeTime ;
            MachineAddAccess.Status = INFO_ACCESS_SAVE;
        }

        if( MachineAddAccess.IsInfoInvalid || ( CumulativeTimeBase >= MACHINE_CUMULATIVE_TIME_PERIOD ) ||
                ( OdometerCounter >= MACHINE_ODOMETER_RECORD_UNIT ) || ( INFO_ACCESS_SAVE == MachineAddAccess.Status ) ||
                ( INFO_ACCESS_RESET_TIME == MachineAddAccess.Status ) || ( INFO_ACCESS_RESET_ODOMETER == MachineAddAccess.Status ) ||
                ( sizeof( MachineAddInfo ) != MachineAddHeader.InfoSize ) )
        {
            DI();
            if( INFO_ACCESS_RESET_ODOMETER != MachineAddAccess.Status )
            {
                MachineAddInfo.Odometer.Value += OdometerCounter;
                OdometerCounter = 0;
            }
            else
            {
                OdometerCounter = 0;
                MachineAddInfo.Odometer.Value = 0;
            }
            gMachineInfo.Odometer = MachineAddInfo.Odometer.Value;

            if( INFO_ACCESS_RESET_TIME != MachineAddAccess.Status )
            {
                MachineAddInfo.CumulativeTime.Value += CumulativeTimeBase / MACHINE_CUMULATIVE_TIME_MINUTE;
                CumulativeTimeBase = 0;
            }
            else
            {
                MachineAddInfo.CumulativeTime.Value = 0;
                CumulativeTimeBase = 0;
            }
            gMachineInfo.CumulativeTime = MachineAddInfo.CumulativeTime.Value;
            EI();
            MachineAddAccess.BlockRetryCnt = 0;
            MachineAddAccess.HeaderRetryCnt = 0;
            MachineAddAccess.Status = INFO_ACCESS_WRITE_HEADER;
            MachineAddInfo.RecordCounter++;
        }
    }
}
/***************************************************************************************************
                                           END OF FILE
***************************************************************************************************/



