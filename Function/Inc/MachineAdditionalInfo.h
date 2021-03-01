/***************************************************************************************************
file:
funciton:
auther:
***************************************************************************************************/
/***************************************************************************************************
                             
***************************************************************************************************/ 
#ifndef      __MACHINE_ADDITIONAL_INFO_H__
    #define  __MACHINE_ADDITIONAL_INFO_H__
/***************************************************************************************************
                                   本模块EXTERN宏定义区
***************************************************************************************************/ 
    #ifdef      EXTERN
        #undef  EXTERN
    #endif
    
    #ifdef  __MACHINE_ADDITIONAL_INFO_C__
        #define EXTERN
    #else
        #define EXTERN  extern
    #endif
/***************************************************************************************************
                                        
***************************************************************************************************/
#define AXIS_BLOCK_MAX_SIZE                   ( 256 )
#define MACHINE_INFO_BLOCK_MAX_SIZE           ( 512 )
#define REDUNDANCY_BLOCK_OFFSET               ( 1024 )
#define MACHINE_ADD_INFO_ADDR                 ( 2048 )
#define MACHINE_ADD_INFO_REDUNDANCY_OFFSET    ( 256 )
#define MACHINE_ADD_INFO_AREA_NUM             ( 4 )
#define MACHINE_ADD_INFO_MAX_RECORD_NUM       ( 80000UL )
#define MACHINE_ODOMETER_RECORD_UNIT          ( 500 )  // in m
/*in 25ms tick，30 minute */
#define MACHINE_CUMULATIVE_TIME_PERIOD        ( 1800000UL / 25 )
#define MACHINE_CUMULATIVE_TIME_MINUTE        ( 60000UL / 25 )

#define PARAM_BLOCK1                          ( 0x01 )
#define PARAM_BLOCK2                          ( 0x02 )
#define PARAM_BLOCK_BITS                      ( PARAM_BLOCK1|PARAM_BLOCK2 )
#define PARAM_BLOCK_NUM                       ( 2 )
#define PARAM_RETRY_NUM                       ( 10 )
/***************************************************************************************************
                                        
***************************************************************************************************/
typedef __packed struct 
{
    char ItemSize;
    char Keyword[11];
    unsigned long Value;
}STRUCT_ADDITIONAL_INFO_ITEM;


typedef __packed struct
{
    unsigned short InfoSize;
    unsigned long  InfoAddress;
    unsigned short HeaderCRC;
}STRUCT_ADDITIONAL_INFO_HEADER;

typedef __packed struct 
{
     unsigned long RecordCounter;
     STRUCT_ADDITIONAL_INFO_ITEM   Odometer;
     STRUCT_ADDITIONAL_INFO_ITEM   CumulativeTime;
     unsigned short BlockCRC;
}STRUCT_ADDITIONAL_INFO;

typedef struct 
{
    float Mileage;
    unsigned long  EnconderPlusCnt;
}STRUCT_WHEEL_MILEAGE;


typedef enum 
{
    INFO_ACCESS_NONE,
    INFO_ACCESS_WRITE_HEADER,
    INFO_ACCESS_CHECK_HEADER,
    INFO_ACCESS_WRITE_INFO,
    INFO_ACCESS_CHECK_INFO,
    INFO_ACCESS_RESET_TIME,
    INFO_ACCESS_RESET_ODOMETER,
    INFO_ACCESS_SAVE,
}ENUM_INFO_ACCESS_STATUS;

typedef struct 
{
    ENUM_INFO_ACCESS_STATUS Status;
    unsigned char IsInfoInvalid;
    unsigned char GoodInfoBlock;
    unsigned char GoodHeaderBlock;
    unsigned char BlockRetryCnt;
    unsigned char HeaderRetryCnt;
}STRUCT_ADDITIONAL_INFO_ACCESS;
/***************************************************************************************************
                                        
***************************************************************************************************/
#define DI()                       __set_FAULTMASK(1);
#define EI()                       __set_FAULTMASK(0);
/***************************************************************************************************
                                        
***************************************************************************************************/
extern unsigned long OdometerCounter;
extern unsigned long CumulativeTimeBase;
extern STRUCT_WHEEL_MILEAGE   WheelsMileage[2];
extern STRUCT_ADDITIONAL_INFO MachineAddInfo;
/***************************************************************************************************
                                        
***************************************************************************************************/
extern void CumulativeTimeCount( void );
extern void MachineAddInfoProcess( void );
extern void EnableMachineAddInfoSave( void );
extern void UpdateWheelEncoderPlusCount( STRUCT_WHEEL_MILEAGE *pWheel, signed int DeltaPlusCnt );
extern void CalculateWheelMileage( STRUCT_WHEEL_MILEAGE *pWheel, double WheelPerimeter, unsigned int PlusCntPerCircle );
extern unsigned char IsMachineAddInfoSaveOK( void );
extern unsigned char LoadMachineAddInfo( void );
/**************************************************************************************************/
#endif /*__MACHINE_ADDITIONAL_INFO_H__*/
/***************************************************************************************************
                                       END OF FILE
***************************************************************************************************/

