#ifndef _BOOTLOADER_INFO_H
#define _BOOTLOADER_INFO_H





#include "UserDataTypes.h"
//#include "flash.h"

/* Exported macro ------------------------------------------------------------*/
/*Base address of the Function Block*/
#define BLOCK_SIZE_BOOTLOADER_INFO					(4)//32
#define BASE_ADDR_BOOTLOADER_INFO       		(4096-BLOCK_SIZE_BOOTLOADER_INFO)

#define LOADER_APP_CODE  0
#define LOADER_IAP_CODE  1

#define APP_BKP_SUCCESS 0
#define APP_BKP_FAILED 1

#define APP_RECOVERY_SUCCESS 0
#define APP_RECOVERY_FAILED 1

#define IAP_SUCCESS 0
#define IAP_FAILED 1

#define EN_SOFTWAREVERSION_OFFSET 3
#define EN_APP_READY 1
#define EN_APP_NOT_READY 2

typedef struct ST_VersionStruct
{
	UINT8 majorVer; 
	UINT8 minorVer;
	UINT8 patchVer;
}ST_VersionStruct;

typedef struct BootLoaderInfo
{
	ST_VersionStruct SoftwareVersion;
	UINT32 FileSize;
	UINT16 CurrentBlockSize;
	UINT16  CurrentBlockNum;
	UINT8  CurrentAddr;
	UINT8  BlockDownloadBytesCnt;
	UINT8  CANMessage[8];
	UINT8  F_Reset;
}BootLoaderInfo;





#pragma pack ()


PUBLIC void BootloaderInfoOperateInit(void);
PUBLIC UINT8 WriteBootloaderInfo(void);
PUBLIC void ReadBootloaderInfo(void);


PUBLIC void CreateForceEnterAppTask(void);
PUBLIC void DeleteForceEnterAppTask(void);
PUBLIC UINT8 GetForceEnterAppTaskStatus(void);
PUBLIC void GetLastSoftwareVersion(BootLoaderInfo* pst_BootLoaderInfo);
void WriteSoftWareVersion(BootLoaderInfo* pst_bootloaderInfo,ST_VersionStruct* pst_NowSoftWareVersion);
void WriteAppNotReady(void);
#endif

