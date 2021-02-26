#include "gd_hal.h"
#include "gd32f4xx.h"

/*!
    \brief    HAL_GPIO_WritePin
    \param[in]  gpio_periph: GPIO port 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,E,F,G,H,I)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[in]  PinState:
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_RESET
                  GPIO_PIN_SET
    \param[out] none
    \retval     none
*/
void HAL_GPIO_WritePin(uint32_t gpio_periph,uint32_t pin,GPIO_PinState PinState)
{
    if(PinState == GPIO_PIN_RESET)
    {
        GPIO_BC(gpio_periph) = (uint32_t)pin;
    }
    else if(PinState == GPIO_PIN_SET)
    {
        GPIO_BOP(gpio_periph) = (uint32_t)pin;
    }
    
}


/*!
    \brief    get GPIO pin input status
    \param[in]  gpio_periph: GPIO port 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,E,F,G,H,I)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     input status of GPIO pin: GPIO_PIN_SET or GPIO_PIN_RESET
*/
GPIO_PinState HAL_GPIO_ReadPin(uint32_t gpio_periph,uint32_t pin)
{
  GPIO_PinState bitstatus;

  if((GPIO_ISTAT(gpio_periph)&(pin)) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
}

















