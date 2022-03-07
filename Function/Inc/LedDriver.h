/*******************************************************************
 *
 * FILE NAME:  LedDriver.h
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2019.3.23
 *
 * AUTHOR:      Liang
 *
 * History:
------------------------------------------------------------------------
23-3-2018 Version 1.00 : Created by Liang
----------------------------------------------------------------------*/

#ifndef _LedDriver_APP_H_
#define _LedDriver_APP_H_

#include "UserDataTypes.h"

/***********define TLC59108 command****************/
#define I2C2_LedDriverWrite					0x98
#define I2C2_LedDriverRead					0x99

/***********define TLC59108 internal register address****************/
#define	MODE1						0x00	//Mode register 1
#define	MODE2				    0x01	//Mode register 2
#define	PWM0				    0x02    //Brightness control LED0
#define	PWM1    				0x03    //Brightness control LED1
#define	PWM2				    0x04    //Brightness control LED2
#define	PWM3    				0x05    //Brightness control LED3
#define	PWM4				    0x06    //Brightness control LED4
#define	PWM5    				0x07    //Brightness control LED5
#define	PWM6				    0x08    //Brightness control LED6
#define	PWM7    				0x09    //Brightness control LED7
#define	GRPPWM					0x0A	//Group duty cycle control
#define	GRPFREQ				  0x0B	//Group frequency
#define	LEDOUT0					0x0C	//LED output state 0
#define	LEDOUT1					0x0D	//LED output state 1
#define	SUBADR1					0x0E	//I2C bus sub-address 1
#define	SUBADR2					0x0F	//I2C bus sub-address 2
#define	SUBADR3					0x10	//I2C bus sub-address 3
#define	ALLCALLADR			0x11	//LED all call I2C bus address

struct MODE1BITS
{
   BOOL ALLCALL:1; 
   BOOL SUB3:1;
   BOOL SUB2:1;
   BOOL SUB1:1;
   BOOL OSC:1;
   BOOL AI0:1;
   BOOL AI1:1;
   BOOL AI2:1;   
};

struct MODE2BITS
{
   UINT8 Reserved:3; 
   BOOL OCH:1;
   BOOL Reserved1:1;
   BOOL DMBLNK:1;
   UINT8 Reserved2:2;
 
};

struct RegLEDOUT0BITS
{
   UINT8 LDR0:2; 
   UINT8 LDR1:2; 
   UINT8 LDR2:2; 
   UINT8 LDR3:2;  
};

struct RegLEDOUT1BITS
{
   UINT8 LDR4:2; 
   UINT8 LDR5:2; 
   UINT8 LDR6:2; 
   UINT8 LDR7:2;  
};

struct CtrWordBITS
{
   UINT8 LedMode:3;
   BOOL  BLINK:1;
   BOOL  Remote:1;
   BOOL  Left:1; 
   BOOL  Right:1; 
   UINT8 Reserved:1;
};

typedef struct Tlc59108fReg
{
    union RegMODE1
    {
        UINT8 ALL;
        struct MODE1BITS bit;
    }RegMODE1;
    union RegMODE2
    {
        UINT8 ALL;
        struct MODE2BITS bit;
    }RegMODE2;    
    UINT8 RegPWM0;
    UINT8 RegPWM1;
    UINT8 RegPWM2;
    UINT8 RegPWM3;
    UINT8 RegPWM4;
    UINT8 RegPWM5;
    UINT8 RegPWM6;
    UINT8 RegPWM7;
    UINT8 RegGRPPWM;
    UINT8 RegGRPFREQ;
    union RegLEDOUT0
    {
        UINT8 ALL;
        struct RegLEDOUT0BITS bit;
    }RegLEDOUT0;
    union RegLEDOUT1
    {
        UINT8 ALL;
        struct RegLEDOUT1BITS bit;
    }RegLEDOUT1;    
    UINT8 RegSUBADR1;
    UINT8 RegSUBADR2;
    UINT8 RegSUBADR3;
    UINT8 RegALLCALLADR;

    union CtrWord
    {
        UINT8 ALL;
        struct CtrWordBITS bit;
    }CtrWord;

}t_Tlc59108fReg;

typedef enum
{
	LED_STATE_CLOSE		      = 0x00,
	LED_STATE_START         = 0X01,
	LED_STATE_RUN_STRAIGHT  = 0x02,
	LED_STATE_RUN_TRUNL     = 0x03,
	LED_STATE_RUN_TRUNR     = 0x04,
	LED_STATE_DISPATCH	    = 0x05,
	LED_STATE_ARRIVE        = 0x06,
	LED_STATE_CHARGE_ING    = 0x07,
	LED_STATE_CHARGE_FINISH = 0x08,
	LED_STATE_ERROR         = 0x09,
	LED_STATE_AWAIT         = 0X0A,
	LED_STATE_CHARGE_0_19   = 0X0B,
	LED_STATE_CHARGE_20_39  = 0X0C,
	LED_STATE_CHARGE_40_59  = 0X0D,
	LED_STATE_CHARGE_60_79  = 0X0E,
	LED_STATE_CHARGE_80_99  = 0X0F,
	 LED_STATE_POWEROFF      = 0X10,
    LED_STATE_POWERON       = 0X11,
	LED_STATE_MAX
}LedStateEnum;

typedef enum
{
	LED_EVENT_START_FINISH  	= 0x00,
	LED_EVENT_REMOTE_CONTROL  = 0x01,
	LED_EVENT_MCU_POWEROFF    = 0x02,
	LED_EVENT_CHARGE_ING      = 0x03,
	LED_EVENT_CHARGE_OUT      = 0x04,
	LED_EVENT_MAX 
}LedEventEnum;

/***********define Led state mask****************/
#define EVENT_START_FINISH_CUR_STATE_MASK      0x0000002 //state: 0x01
#define EVENT_START_FINISH_NEXT_STATE_MASK     0x0000400 //state: 0x0A

#define EVENT_REMOTE_CONTROL_CUR_STATE_MASK		 0x000067D //state: 0x00 0x02 0x03 0x04 0x05 0x06 0x09 0x0A
#define EVENT_REMOTE_CONTROL_NEXT_STATE_MASK   0x000067D //state: 0x00 0x02 0x03 0x04 0x05 0x06 0x09 0x0A

#define EVENT_MCU_POWEROFF_CUR_STATE_MASK      0x000067D //state: 0x00 0x02 0x03 0x04 0x05 0x06 0x09 0x0A
#define EVENT_MCU_POWEROFF_NEXT_STATE_MASK		 0x0000001 //state: 0x00

#define EVENT_CHARGE_ING_CUR_STATE_MASK        0x000FFFD //state: 0x00 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09 0x0A 0x0B 0x0C 0x0D 0x0E 0x0F 
#define EVENT_CHARGE_ING_NEXT_STATE_MASK       0x000F980 //state: 0x07 0x08 0x0B 0x0C 0x0D 0x0E 0x0F

#define EVENT_CHARGE_OUT_CUR_STATE_MASK        0x000FB80 //state: 0x07 0x08 0x09 0x0B 0x0C 0x0D 0x0E 0x0F
#define EVENT_CHARGE_OUT_NEXT_STATE_MASK			 0x0000400 //state: 0x0A

typedef struct
{
	LedEventEnum event;                		/* trigger event */
	UINT32 curStateMask;             	   	/* current status mask */
	UINT32 nextStateMask;            	    /* next status mask */
	void (*eventActFun)(void *);  			  /* act fuction */
}LedFsmTableStruct;

typedef struct
{
	LedFsmTableStruct *sLedFsmTable;       /* status transfer table */
	LedStateEnum curState;             	   /* status machine current status */
	UINT8 stuMaxNum;            					 /* status transfer number */
	UINT8 stateTransferFlag;               /* status transfer flag */
}LedFsmStruct;

extern LedFsmStruct sLedFsm;

extern PUBLIC void led_bar_driver( void );
extern PUBLIC void LedDriverInit(void);
extern PUBLIC void LedDriverExec(void);
extern PUBLIC void LedFsmEventHandle(LedFsmStruct *pLedFsm, UINT8 event, LedStateEnum aimState, void *parm);
extern PUBLIC void LedPowerOn(void);
extern PUBLIC void LedPowerOff(void);
extern PUBLIC void GetChargeState(UINT8 *recbuff);

PUBLIC UINT8 GetBatteryLevelForLed(UINT8 BatteryLevel);

void YDA138_MuteOff(void);
void YDA138_MuteOn(void);
PUBLIC void A2M7_CtrlOn(UINT8 Spd);
PUBLIC void A2M7_CtrlOff(void);
#endif
