
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpioiic_H
#define __gpioiic_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32f7xx_hal.h"
#include "UserDataTypes.h"

typedef struct{

        void        (*SDA_SetPinDir )( UINT8 Dir );
        void        (*SCL_SetPinDir )( UINT8 Dir );
				void        (*SDA_WritePin)(UINT8 PinState);
        void        (*SCL_WritePin)(UINT8 PinState);
        UINT8       (*SDA_ReadPinState)(void);
}GpioIICStruct;


void gpioiic_start(GpioIICStruct* GpioIIC);
void gpioiic_stop(GpioIICStruct* GpioIIC);
void gpioiic_ack(GpioIICStruct* GpioIIC);
void gpioiic_noack(GpioIICStruct* GpioIIC);
_Bool gpioiic_waitack(GpioIICStruct* GpioIIC);
void gpioiic_WriteByte(GpioIICStruct* GpioIIC, UINT8 txd);
UINT8 gpioiic_ReadByte(GpioIICStruct* GpioIIC);

#ifdef __cplusplus
}
#endif
#endif /*__ __gpioiic_H */

