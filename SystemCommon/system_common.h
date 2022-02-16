/***************************************************************************************************
file:
funciton:
auther:
***************************************************************************************************/
/***************************************************************************************************
                             
***************************************************************************************************/ 
#ifndef      __SYSTEM_COMMON_H__
    #define  __SYSTEM_COMMON_H__
/***************************************************************************************************
                                   本模块EXTERN宏定义区
***************************************************************************************************/ 
    #ifdef      EXTERN
        #undef  EXTERN
    #endif
    
    #ifdef  __SYSTEM_COMMON_C__
        #define EXTERN
    #else
        #define EXTERN  extern
    #endif
/***************************************************************************************************
                                        
***************************************************************************************************/
#define DEBUG                                     ( 1 ) /* 0: Release Version, !0: Debug Version*/
#define BOOTLOADER                                ( 0 ) /* 0: Firmware, !0: Bootloader*/
/***************************************************************************************************
                                        
***************************************************************************************************/
#define MCU_FLASH_SIZE                            ( 512 * 1024UL )
#define BOOTLOADER_SIZE                           ( 128 * 1024UL )
#define FIRMWARE_MAX_SIZE                         ( MCU_FLASH_SIZE - BOOTLOADER_SIZE - 4 )
#define FIRMWARE_BASE_ADDRESS                     ( 0x08020000 )
/***************************************************************************************************
                                        
***************************************************************************************************/
typedef struct 
{
    unsigned long ItemSize;
    char Keyword[20];
    unsigned long Value;
}STRUCT_INFO_ITEM;

typedef struct 
{
    unsigned long InfoSize;
    unsigned long ValidSignature[2];
    STRUCT_INFO_ITEM Company;
    STRUCT_INFO_ITEM Product;
    STRUCT_INFO_ITEM HardwareVersion;
    STRUCT_INFO_ITEM BootloaderVersion;
}STRUCT_BOOTLOADER_INFO;

typedef struct 
{
    unsigned long InfoSize;
    STRUCT_INFO_ITEM Company;
    STRUCT_INFO_ITEM Product;
    STRUCT_INFO_ITEM FirmwareVersion;
}STRUCT_FIRMWARE_INFO;
/***************************************************************************************************
                                 
***************************************************************************************************/
extern volatile unsigned long ResetSignature[2];
extern volatile unsigned long HotBootSignature[2];
extern volatile unsigned long ResetMode;
/***************************************************************************************************
                                        
***************************************************************************************************/
#if defined( BOOTLOADER ) && ( BOOTLOADER != 0 )
extern const STRUCT_BOOTLOADER_INFO BootloaderInfo;
extern unsigned long FirmwareSignature[2];
extern unsigned long FirmwareSize;               
extern STRUCT_FIRMWARE_INFO   FirmwareInfo;
#else
extern STRUCT_BOOTLOADER_INFO BootloaderInfo;
extern const unsigned long FirmwareSignature[2];       
extern const unsigned long FirmwareSize;               
extern const STRUCT_FIRMWARE_INFO   FirmwareInfo;
#endif
/***************************************************************************************************
                                        
***************************************************************************************************/
extern void ClearResetSignature( void );
extern void SYS_DisableInterrupt( void );
/**************************************************************************************************/
#endif /*__SYSTEM_COMMON_H__*/
/***************************************************************************************************
                                       END OF FILE
***************************************************************************************************/

