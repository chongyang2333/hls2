/***************************************************************************************************
file:
funciton:
auther:
***************************************************************************************************/
/***************************************************************************************************
                             
***************************************************************************************************/ 
#define  __SYSTEM_COMMON_C__
#include "stm32f7xx_hal.h"
#include "signature_code.h"
#include "system_common.h"
#undef   __SYSTEM_COMMON_C__
/**************************************************************************************************/
/***************************************************************************************************
                               此段定义的变量及分配的地址禁止修改     
***************************************************************************************************/
volatile unsigned long ResetSignature[2]     __attribute__( ( at( RESET_SIGNATURE_ADDRESS + 0x00 ) ) );
volatile unsigned long HotBootSignature[2]   __attribute__( ( at( RESET_SIGNATURE_ADDRESS + 0x08 ) ) );
volatile unsigned long ResetMode             __attribute__( ( at( RESET_SIGNATURE_ADDRESS + 0x10 ) ) );
/***************************************************************************************************
                                    Bootloader
***************************************************************************************************/
#if ( defined( BOOTLOADER ) && ( BOOTLOADER != 0 ) )
/*==================================================================================================
Name:
Funciton:
==================================================================================================*/
const STRUCT_BOOTLOADER_INFO BootloaderInfo  __attribute__( ( at( BOOTLOADER_INFO_ADDRESS ) ) ) = 
{
    .InfoSize = sizeof( STRUCT_BOOTLOADER_INFO ),
    .ValidSignature = { BOOTLOADER_SIGNATURE0_VALID, BOOTLOADER_SIGNATURE1_VALID },
	.Company = { .ItemSize = sizeof( STRUCT_INFO_ITEM ), .Keyword = "PuDu", .Value = 0 },
    .Product = { .ItemSize = sizeof( STRUCT_INFO_ITEM ), .Keyword = "HuiGou", .Value = 0 },
    .HardwareVersion = { .ItemSize = sizeof( STRUCT_INFO_ITEM ), .Keyword = "Hardware", .Value = 0x00010001 },
    .BootloaderVersion = { .ItemSize = sizeof( STRUCT_INFO_ITEM ), .Keyword = "Bootloader", .Value = 0x00010001 },
};

unsigned long FirmwareSignature[2]     __attribute__( ( at( FIRMWARE_BASE_ADDRESS + FIRMWARE_SIGNATURE_OFFSET + 0x00 ) ) );
unsigned long FirmwareSize             __attribute__( ( at( FIRMWARE_BASE_ADDRESS + FIRMWARE_SIGNATURE_OFFSET + 0x08 ) ) );
STRUCT_FIRMWARE_INFO FirmwareInfo      __attribute__( ( at( FIRMWARE_BASE_ADDRESS + FIRMWARE_SIGNATURE_OFFSET + 0x0C ) ) );
#else
/***************************************************************************************************
                                     Firmware 
***************************************************************************************************/
STRUCT_BOOTLOADER_INFO BootloaderInfo  __attribute__( ( at( BOOTLOADER_INFO_ADDRESS ) ) );
/*==================================================================================================
Name:
Funciton:
==================================================================================================*/
#if defined( DEBUG ) && ( DEBUG != 0 )

const unsigned long FirmwareSignature[2]     __attribute__( ( at( FIRMWARE_BASE_ADDRESS + FIRMWARE_SIGNATURE_OFFSET + 0x00 ) ) ) =
{ 
    FIRMWARE_SIGNATURE0_DEBUG, 
    FIRMWARE_SIGNATURE1_DEBUG,
};

#else

const unsigned long FirmwareSignature[2]     __attribute__( ( at( FIRMWARE_BASE_ADDRESS + FIRMWARE_SIGNATURE_OFFSET + 0x00 ) ) ) = 
{ 
    FIRMWARE_SIGNATURE0_RELEASE, 
    FIRMWARE_SIGNATURE1_RELEASE,
};

#endif
/*==================================================================================================
Name:
Funciton:
==================================================================================================*/
const unsigned long FirmwareSize             __attribute__( ( at( FIRMWARE_BASE_ADDRESS + FIRMWARE_SIGNATURE_OFFSET + 0x08 ) ) ) = FIRMWARE_SIZE_SIGNATURE;
/*==================================================================================================
Name:
Funciton:
==================================================================================================*/
const STRUCT_FIRMWARE_INFO FirmwareInfo      __attribute__( ( at( FIRMWARE_BASE_ADDRESS + FIRMWARE_SIGNATURE_OFFSET + 0x0C ) ) ) = 
{
    .InfoSize = sizeof( STRUCT_FIRMWARE_INFO ),
    .Company = { .ItemSize = sizeof( STRUCT_INFO_ITEM ), .Keyword = "PuDu", .Value = 0 },
    .Product = { .ItemSize = sizeof( STRUCT_INFO_ITEM ), .Keyword = "Robotic Dog", .Value = 0 },
    .FirmwareVersion = { .ItemSize = sizeof( STRUCT_INFO_ITEM ), .Keyword = "Firmware", .Value = 0x001E0063 },
};
/*==================================================================================================
Name:
Work:
Input:
Output:
==================================================================================================*/
void EnableFirmwareUpdate( void )
{
    SYS_DisableInterrupt();
    ResetSignature[0] = RESET_SIGNATURE0_UPDATE_FIRMWARE;
    ResetSignature[1] = RESET_SIGNATURE1_UPDATE_FIRMWARE;
    NVIC_SystemReset();
}

#endif
/*==================================================================================================
Name:
Work:
Input:
Output:
==================================================================================================*/
void SYS_EnableInterrupt( void )
{
    register unsigned int reg __asm( "primask" );
  
    reg = 0;
}
/*==================================================================================================
Name:
Work:
Input:
Output:
==================================================================================================*/
void SYS_DisableInterrupt( void )
{
    register unsigned int reg __asm( "primask" );
  
    reg = 1;
}
/*==================================================================================================
Name:
Funciton:
Input:
Output:
==================================================================================================*/
void ClearResetSignature( void )
{
    ResetSignature[0] = 0;
    ResetSignature[1] = 0;
}
/***************************************************************************************************
                                           END OF FILE
***************************************************************************************************/



