/*******************************************************************
 *
 * FILE NAME:  ISTMagic.c
 *
 * DESCRIPTION:
 *
 * CREATED ON:  2021.9.1
 *
 * AUTHOR:      Hzy
 *
 * History:
------------------------------------------------------------------------
----------------------------------------------------------------------*/

/*------------------------- Include files ----------------------------*/

#include "delay.h"
#include "CanApp.h"
#include "IST8310I2C.h"
#include "stdlib.h"
#include "Param.h"
#include "ISTMagic.h"
#include "gpio.h"

UINT8 alarm_level=0;
UINT8 alarm_levelBak=0;
INT16 MagXYZ_L[3]= {0};
INT16 MagXYZ_R[3]= {0};
extern struct CanAppStruct    sMyCan;
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void ISTMagic_init(void)
{
    IST8310I2C_Init();
    IST8310I2C_WADDR = 0x18;
    IST8310_Byte_Write(IST8310_REG_CNTRL2, 0x01); //soft-reset
    delay_ms(20);
    IST8310_Byte_Write(IST8310_REG_AVGCNTL, 0x24); //average 16 times
    IST8310_Byte_Write(IST8310_REG_PULSE_DURATION, 0xC0); //Pulse Duration Control
    IST8310_Byte_Write(IST8310_REG_CNTRL1,0x1);// single - mode
    delay_ms(6);

    IST8310I2C_WADDR = 0x1C;
    IST8310_Byte_Write(IST8310_REG_CNTRL2, 0x01); //soft-reset
    delay_ms(20);
    IST8310_Byte_Write(IST8310_REG_AVGCNTL, 0x24); //average 16 times
    IST8310_Byte_Write(IST8310_REG_PULSE_DURATION, 0xC0); //Pulse Duration Control
    IST8310_Byte_Write(IST8310_REG_CNTRL1,0x1);// single - mode
    delay_ms(6);
}

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
PUBLIC void MagXYZ_Exec(void)
{
    UINT8 DataBuffer[14] = {0};
    UINT8 i;
    UINT16 tempL;
    UINT16 tempR;
    UINT16 temp_level = 0;

    if(sMyCan.Magic_Enable == 0)
    {
        alarm_level = 0;
        ExtVDisable();//关闭3.3V
        return;
    }
    else if(sMyCan.Magic_Enable == 1)
    {
        ExtVEnable();//打开3.3V
    }

    IST8310I2C_WADDR = 0x18;
    IST8310I2C_Serial_Read(IST8310_REG_DATAX,DataBuffer,6);

    IST8310I2C_WADDR = 0x1C;
    IST8310I2C_Serial_Read(IST8310_REG_DATAX,&DataBuffer[6],6);

    MagXYZ_L[0] = (short)((DataBuffer[1]<<8)+DataBuffer[0]);

    MagXYZ_L[1] = (short)((DataBuffer[3]<<8)+DataBuffer[2]);

    MagXYZ_L[2] = (short)((DataBuffer[5]<<8)+DataBuffer[4]);

    MagXYZ_R[0] = (short)((DataBuffer[7]<<8)+DataBuffer[6]);

    MagXYZ_R[1] = (short)((DataBuffer[9]<<8)+DataBuffer[8]);

    MagXYZ_R[2] = (short)((DataBuffer[11]<<8)+DataBuffer[10]);

    for(i=0; i<3; i++)
    {
        tempL = abs(MagXYZ_L[i]);
        tempR = abs(MagXYZ_R[i]);
        if((tempL>sMyCan.MagicThreshold_left)||(tempR>sMyCan.MagicThreshold_Right))
        {
            temp_level = 1;
            break;
        }
    }

    alarm_level = temp_level;
#if 0
    if(sMyCan.Magic_Enable == 1)
    {
        alarm_level = temp_level;
    }
    else
    {
        alarm_level = 0;
    }
#endif
    CanSendMagXYZ(MagXYZ_L,0xAA);
    CanSendMagXYZ(MagXYZ_R,0xAB);

    IST8310I2C_WADDR = 0x18;
    IST8310_Byte_Write(IST8310_REG_CNTRL1,0x1);// single - mode
    IST8310I2C_WADDR = 0x1C;
    IST8310_Byte_Write(IST8310_REG_CNTRL1,0x1);// single - mode

}

