
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __battery_iic_H
#define __battery_iic_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f4xx.h"

typedef struct{
		void        (*SDA_WritePin)(uint8_t PinState);
        void        (*SCL_WritePin)(uint8_t PinState);
        uint8_t     (*SDA_ReadPinState)(void);
        uint8_t     (*SCL_ReadPinState)(void);
}BatteryIICStruct;

uint8_t is_battery_iic_free(BatteryIICStruct* p);
void battery_iic_start(BatteryIICStruct* p);
uint8_t battery_iic_stop(BatteryIICStruct* p);
void battery_iic_ack(BatteryIICStruct* p);
void battery_iic_noack(BatteryIICStruct* p);
_Bool battery_iic_waitack(BatteryIICStruct* p);
void battery_iic_WriteByte(BatteryIICStruct* p, uint8_t txd);
uint8_t battery_iic_ReadByte(BatteryIICStruct* p, uint8_t *pRdata);

#ifdef __cplusplus
}
#endif
#endif /*__ __battery_iic_H */

