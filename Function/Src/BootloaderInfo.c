#include "BootloaderInfo.h"
#include "Eeprom.h"
//#include "Log.h"
#include "string.h"

extern struct BootLoaderInfo bootloaderInfo;
extern ST_VersionStruct SoftwareVersion;


PUBLIC void CreateForceEnterAppTask(void)
{
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void BootloaderInfoOperateInit(void)
{
		EepromInit();
}



/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC UINT8 WriteBootloaderInfo(void)
{
		return 0;
}



/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ReadBootloaderInfo(void)
{
		EEPROM_Serial_Read(BASE_ADDR_BOOTLOADER_INFO, (UINT8 *)&bootloaderInfo, BLOCK_SIZE_BOOTLOADER_INFO);
}



void GetLastSoftwareVersion(BootLoaderInfo* pst_BootLoaderInfo)
{
	EEPROM_Serial_Read(BASE_ADDR_BOOTLOADER_INFO, (UINT8 *)&pst_BootLoaderInfo->SoftwareVersion.majorVer, BLOCK_SIZE_BOOTLOADER_INFO);
}

void WriteSoftWareVersion(BootLoaderInfo* pst_bootloaderInfo,ST_VersionStruct* pst_NowSoftWareVersion)
{
		UINT32 LastSoftWareVesion =  (pst_bootloaderInfo->SoftwareVersion.majorVer<<16)
															+ (pst_bootloaderInfo->SoftwareVersion.minorVer<<8)
															+ (pst_bootloaderInfo->SoftwareVersion.patchVer);
	  UINT32 NewVersion = (pst_NowSoftWareVersion->majorVer<<16)
											+ (pst_NowSoftWareVersion->minorVer<<8)
											+ (pst_NowSoftWareVersion->patchVer);
		if(NewVersion != LastSoftWareVesion)
		{
			EEPROM_Serial_Write(BASE_ADDR_BOOTLOADER_INFO,&pst_NowSoftWareVersion->majorVer,3);
		}
		memcpy(&pst_bootloaderInfo->SoftwareVersion.majorVer, &pst_NowSoftWareVersion->majorVer, 3);
}

void WriteAppNotReady()
{
	unsigned char ucWrite = EN_APP_NOT_READY;
	EEPROM_Serial_Write(BASE_ADDR_BOOTLOADER_INFO + EN_SOFTWAREVERSION_OFFSET,&ucWrite,1);
}
