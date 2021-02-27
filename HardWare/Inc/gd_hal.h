#ifndef GD_HAL_H
#define GD_HAL_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "gd32f4xx.h"

typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


void HAL_GPIO_WritePin(uint32_t gpio_periph,uint32_t pin,GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(uint32_t gpio_periph,uint32_t pin);

void HAL_NVIC_SystemReset(void);

#ifdef __cplusplus
}
#endif


#endif
