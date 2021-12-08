
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __magfieldsensor_iic_H
#define __magfieldsensor_iic_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f4xx.h"

#define MAG_SCL_PORT  GPIOE
#define MAG_SCL_GPIO  GPIO_PIN_6
#define MAG_SDA_PORT  GPIOE
#define MAG_SDA_GPIO  GPIO_PIN_2



typedef struct{
		void        (*SDA_WritePin)(uint8_t PinState);
        void        (*SCL_WritePin)(uint8_t PinState);
        uint8_t     (*SDA_ReadPinState)(void);
        uint8_t     (*SCL_ReadPinState)(void);
}MagfieldsensorIICStruct;


void Mag_I2C_GPIO_Init(void);
uint8_t is_MagFieldsensor_free(MagfieldsensorIICStruct* p);
void magfieldsensor_iic_start(MagfieldsensorIICStruct* p);
uint8_t magfieldsensor_iic_stop(MagfieldsensorIICStruct* p);
void magfieldsensor_iic_ack(MagfieldsensorIICStruct* p);
void magfieldsensor_iic_noack(MagfieldsensorIICStruct* p);
_Bool magfieldsensor_iic_waitack(MagfieldsensorIICStruct* p);
void magfieldsensor_iic_WriteByte(MagfieldsensorIICStruct* p, uint8_t txd);
uint8_t magfieldsensor_iic_ReadByte(MagfieldsensorIICStruct* p, uint8_t *pRdata);

#ifdef __cplusplus
}
#endif
#endif /*__ __magfieldsensor_iic_H */

