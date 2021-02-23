
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __myiic_H
#define __myiic_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

typedef struct{

        void        (*SDA_SetPinDir )( uint8_t Dir );
        void        (*SCL_SetPinDir )( uint8_t Dir );
		void        (*SDA_WritePin)(uint8_t PinState);
        void        (*SCL_WritePin)(uint8_t PinState);
        uint8_t     (*SDA_ReadPinState)(void);
}MyIICStruct;


void myiic_start(MyIICStruct* MyIIC);
void myiic_stop(MyIICStruct* MyIIC);
void myiic_ack(MyIICStruct* MyIIC);
void myiic_noack(MyIICStruct* MyIIC);
_Bool myiic_waitack(MyIICStruct* MyIIC);
void myiic_WriteByte(MyIICStruct* MyIIC, uint8_t txd);
uint8_t myiic_ReadByte(MyIICStruct* MyIIC);

#ifdef __cplusplus
}
#endif
#endif /*__ __myiic_H */

