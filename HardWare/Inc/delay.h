
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DELAY_H
#define _DELAY_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"


void delay_us(uint32_t nus);
void delay_ms(uint32_t nms);

#ifdef __cplusplus
}
#endif
#endif /* _DELAY_H */

