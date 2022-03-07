
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpioiic_H
#define __gpioiic_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f4xx.h"

typedef struct{

        void        (*SDA_SetPinDir )( uint8_t Dir );
        void        (*SCL_SetPinDir )( uint8_t Dir );
				void        (*SDA_WritePin)(uint8_t PinState);
        void        (*SCL_WritePin)(uint8_t PinState);
        uint8_t     (*SDA_ReadPinState)(void);
}GpioIICStruct;


void gpioiic_start(GpioIICStruct* GpioIIC);
void gpioiic_stop(GpioIICStruct* GpioIIC);
void gpioiic_ack(GpioIICStruct* GpioIIC);
void gpioiic_noack(GpioIICStruct* GpioIIC);
_Bool gpioiic_waitack(GpioIICStruct* GpioIIC);
void gpioiic_WriteByte(GpioIICStruct* GpioIIC, uint8_t txd);
uint8_t gpioiic_ReadByte(GpioIICStruct* GpioIIC);

#ifdef __cplusplus
}
#endif
#endif /*__ __gpioiic_H */

