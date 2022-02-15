/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"
#include "UartApp.h"
#include "HardApi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* usart initiliaze parameters struct */
typedef struct
{
    uint32_t baudval;                    /*!< USART baud rate value */
    uint32_t paritycfg;                  /*!< USART parity function */
    uint32_t wlen;                       /*!< USART word length */
    uint32_t stblen;                     /*!< USART stop bit length */
    uint32_t msbf;                       /*!< USART LSB/MSB first */
    usart_invert_enum invertpara;        /*!< USART inverted */
    uint32_t oversamp;                   /*!< USART oversample mode */
    uint32_t obsm;                       /*!< USART sample bit method */
    uint32_t rtimeout_enable;            /*!< USART receiver timeout enable or disable */
    uint32_t rtimeout;                   /*!< USART receiver timeout threshold */
    uint32_t tx_enable;                  /*!< USART transmitter enable or disable */
    uint32_t rx_enable;                  /*!< USART receiver enable or disable */
} usart_parameter_struct;

// UART_HandleTypeDef huart1;
// UART_HandleTypeDef huart3;
// UART_HandleTypeDef huart6;
// DMA_HandleTypeDef hdma_usart3_rx;
// DMA_HandleTypeDef hdma_usart3_tx;


/**
 * \brief      initialize USART parameter struct with a default value
 * \prarm[in]  p_struct: the pointer of the specific struct 
 * \param[out] none
 * \retval     none
*/
static void usart_struct_para_init(void *p_struct)
{
    ((usart_parameter_struct *)p_struct)->baudval         = 115200;
    ((usart_parameter_struct *)p_struct)->paritycfg       = USART_PM_NONE;
    ((usart_parameter_struct *)p_struct)->wlen            = USART_WL_8BIT;
    ((usart_parameter_struct *)p_struct)->stblen          = USART_STB_1BIT;
    ((usart_parameter_struct *)p_struct)->msbf            = USART_MSBF_LSB;
    ((usart_parameter_struct *)p_struct)->invertpara      = USART_DINV_DISABLE;
    ((usart_parameter_struct *)p_struct)->oversamp        = USART_OVSMOD_16;
    ((usart_parameter_struct *)p_struct)->obsm            = USART_OSB_3bit;
    ((usart_parameter_struct *)p_struct)->rtimeout_enable = DISABLE;
    ((usart_parameter_struct *)p_struct)->rtimeout        = ((usart_parameter_struct *)p_struct)->baudval * 3;
    ((usart_parameter_struct *)p_struct)->tx_enable       = USART_TRANSMIT_ENABLE;
    ((usart_parameter_struct *)p_struct)->rx_enable       = USART_RECEIVE_ENABLE;
}


/**
 * \brief      initialize USART gpio pin
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
static void usart_gpio_init(uint32_t usart_periph)
{
    if (usart_periph == USART2)
    {
        #define __USART2_Tx_PORT  GPIOE
        #define __USART2_Tx_GPIO  GPIO_PIN_8
        #define __USART2_Tx_AF    GPIO_AF_7
        #define __USART2_Rx_PORT  GPIOE
        #define __USART2_Rx_GPIO  GPIO_PIN_7
        #define __USART2_Rx_AF    GPIO_AF_7

        /* enable can clock */
        rcu_periph_clock_enable(RCU_USART2);
        rcu_periph_clock_enable(RCU_GPIOD);

        /* configure I2C GPIO */
        gpio_output_options_set(__USART2_Tx_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, __USART2_Tx_GPIO);
        gpio_mode_set(__USART2_Tx_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, __USART2_Tx_GPIO);
        gpio_af_set(__USART2_Tx_PORT, __USART2_Tx_AF, __USART2_Tx_GPIO);
        
        gpio_output_options_set(__USART2_Rx_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, __USART2_Rx_GPIO);
        gpio_mode_set(__USART2_Rx_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, __USART2_Rx_GPIO);
        gpio_af_set(__USART2_Rx_PORT, __USART2_Rx_AF, __USART2_Rx_GPIO);
    }
    else if (usart_periph == USART1)
    {
        #define __USART1_Tx_PORT  GPIOD
        #define __USART1_Tx_GPIO  GPIO_PIN_5
        #define __USART1_Tx_AF    GPIO_AF_7
        #define __USART1_Rx_PORT  GPIOD
        #define __USART1_Rx_GPIO  GPIO_PIN_6
        #define __USART1_Rx_AF    GPIO_AF_7

        /* enable can clock */
        rcu_periph_clock_enable(RCU_USART1);
        rcu_periph_clock_enable(RCU_GPIOD);

        /* configure I2C GPIO */
        gpio_output_options_set(__USART1_Tx_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, __USART1_Tx_GPIO);
        gpio_mode_set(__USART1_Tx_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __USART1_Tx_GPIO);
        gpio_af_set(__USART1_Tx_PORT, __USART1_Tx_AF, __USART1_Tx_GPIO);
        
        gpio_output_options_set(__USART1_Rx_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, __USART1_Rx_GPIO);
        gpio_mode_set(__USART1_Rx_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __USART1_Rx_GPIO);
        gpio_af_set(__USART1_Rx_PORT, __USART1_Rx_AF, __USART1_Rx_GPIO);
    }
}

/**
 * \brief      initialize USART interface
 * \prarm[in]  usart_periph
 * \param[in]  usart_parameter_init
 * \param[out] none
 * \retval     none
*/
static void usart_interface_init(uint32_t usart_periph, usart_parameter_struct *usart_parameter_init)
{
    /* reset usart */
    usart_deinit(usart_periph);

    /* configure usart baud rate */
    usart_baudrate_set(usart_periph, usart_parameter_init->baudval);
    
    /* configure usart parity */
    usart_parity_config(usart_periph, usart_parameter_init->paritycfg);

    /* configure usart word length */
    usart_word_length_set(usart_periph, usart_parameter_init->wlen);

    /* configure usart stop bit length */
    usart_stop_bit_set(usart_periph, usart_parameter_init->stblen);

    /* configure LSB/MSB first */
    usart_data_first_config(usart_periph, usart_parameter_init->msbf);

    /* configure usart inverted */
    usart_invert_config(usart_periph, usart_parameter_init->invertpara);
    
    /* configure usart oversample mode */
    usart_oversample_config(usart_periph, usart_parameter_init->oversamp);

    /* configure usart sample bit method */
    usart_sample_bit_config(usart_periph, usart_parameter_init->obsm);

    /* configure usart receiver timeout threshold */
    usart_receiver_timeout_threshold_config(usart_periph, usart_parameter_init->rtimeout);

    /* configure usart receiver timeout enable or disable */
    if (ENABLE == usart_parameter_init->rtimeout_enable)
    {
        usart_receiver_timeout_enable(usart_periph);
    }
    else {
        usart_receiver_timeout_disable(usart_periph);
    }

    /* configure usart transmitter and receiver enable or disable */
    usart_transmit_config(usart_periph, usart_parameter_init->tx_enable);
    usart_receive_config(usart_periph, usart_parameter_init->rx_enable);

    /* enable usart */
    usart_enable(usart_periph);
}

/**
 * \brief      initialize USART dma xfer
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
static void usart_dma_init(uint32_t usart_periph)
{
    dma_single_data_parameter_struct dma_parameter;

    if (usart_periph == USART2)
    {
        /* Look up <<GD32F4xx_User_Manual>> 'Table 10-5. Peripheral requests to DMA0'
        and 'Table 10-6. Peripheral requests to DMA1'.
          It is forbidden to simultaneously enable these two DMA channels with 
        selecting the same peripheral request.
        */
        #define __USART2_TxDMA_PERIPH  DMA0
        #define __USART2_TxDMA_CHANNEL DMA_CH3
        #define __USART2_TxDMA_SUBPERI DMA_SUBPERI4
        #define __USART2_TxDMA_IRQn    DMA0_Channel3_IRQn

        
        #define __USART2_RxDMA_PERIPH  DMA0
        #define __USART2_RxDMA_CHANNEL DMA_CH1
        #define __USART2_RxDMA_SUBPERI DMA_SUBPERI4
        #define __USART2_RxDMA_IRQn    DMA0_Channel1_IRQn

        /* enable DMA clock */
        rcu_periph_clock_enable(RCU_DMA0);
        
        /*configure DMA interrupt*/
//        nvic_irq_enable(__USART2_TxDMA_IRQn, 0, 0);
//        nvic_irq_enable(__USART2_RxDMA_IRQn, 0, 1);
        
        /* configure DMA channel(for USART tx) */
        dma_single_data_para_struct_init(&dma_parameter);
        dma_deinit(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);
        dma_parameter.direction           = DMA_MEMORY_TO_PERIPH;
        dma_parameter.memory0_addr        = 0U;
        dma_parameter.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
        dma_parameter.periph_addr         = ((uint32_t)&USART_DATA(USART2));
        dma_parameter.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
        dma_parameter.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
        dma_parameter.number              = 0U;
        dma_parameter.priority            = DMA_PRIORITY_HIGH;
        dma_single_data_mode_init(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL, &dma_parameter);
        
        /* configure DMA mode */
        dma_circulation_disable(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);

        /* configure DMA periph request */
        dma_channel_subperipheral_select(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL, __USART2_TxDMA_SUBPERI);

        /* enable DMA channel transfer complete interrupt */
//        dma_interrupt_enable(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL, DMA_CHXCTL_FTFIE);

        /* enable DMA channel */
//        dma_channel_enable(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);

        /* configure DMA channel(for USART rx) */
        dma_single_data_para_struct_init(&dma_parameter);
        dma_deinit(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
        dma_parameter.direction           = DMA_PERIPH_TO_MEMORY;
        dma_parameter.memory0_addr        = 0U;
        dma_parameter.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
        dma_parameter.periph_addr         = (uint32_t)&USART_DATA(USART2);
        dma_parameter.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
        dma_parameter.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
        dma_parameter.number              = 0U;
        dma_parameter.priority            = DMA_PRIORITY_ULTRA_HIGH;
        dma_single_data_mode_init(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL, &dma_parameter);

        /* configure DMA mode */
        dma_circulation_disable(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);

        /* configure DMA periph request */
        dma_channel_subperipheral_select(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL, __USART2_RxDMA_SUBPERI);

        /* enable DMA channel transfer complete interrupt */
//        dma_interrupt_enable(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL, DMA_CHXCTL_FTFIE);

        /* enable DMA1 channel2 */
//        dma_channel_enable(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        /* Look up <<GD32F4xx_User_Manual>> 'Table 10-5. Peripheral requests to DMA0'
        and 'Table 10-6. Peripheral requests to DMA1'.
          It is forbidden to simultaneously enable these two DMA channels with 
        selecting the same peripheral request.
        */
        #define __USART1_TxDMA_PERIPH  DMA0
        #define __USART1_TxDMA_CHANNEL DMA_CH6
        #define __USART1_TxDMA_SUBPERI DMA_SUBPERI4
        #define __USART1_TxDMA_IRQn    DMA0_Channel6_IRQn

        
        #define __USART1_RxDMA_PERIPH  DMA0
        #define __USART1_RxDMA_CHANNEL DMA_CH5
        #define __USART1_RxDMA_SUBPERI DMA_SUBPERI4
        #define __USART1_RxDMA_IRQn    DMA0_Channel5_IRQn

        /* enable DMA clock */
        rcu_periph_clock_enable(RCU_DMA0);
        
        /*configure DMA interrupt*/
//        nvic_irq_enable(__USART2_TxDMA_IRQn, 0, 0);
//        nvic_irq_enable(__USART2_RxDMA_IRQn, 0, 1);
        
        /* configure DMA channel(for USART tx) */
        dma_single_data_para_struct_init(&dma_parameter);
        dma_deinit(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);
        dma_parameter.direction           = DMA_MEMORY_TO_PERIPH;
        dma_parameter.memory0_addr        = 0U;
        dma_parameter.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
        dma_parameter.periph_addr         = ((uint32_t)&USART_DATA(USART1));
        dma_parameter.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
        dma_parameter.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
        dma_parameter.number              = 0U;
        dma_parameter.priority            = DMA_PRIORITY_HIGH;
        dma_single_data_mode_init(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL, &dma_parameter);
        
        /* configure DMA mode */
        dma_circulation_disable(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);

        /* configure DMA periph request */
        dma_channel_subperipheral_select(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL, __USART1_TxDMA_SUBPERI);

        /* enable DMA channel transfer complete interrupt */
//        dma_interrupt_enable(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL, DMA_CHXCTL_FTFIE);

        /* enable DMA channel */
//        dma_channel_enable(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);

        /* configure DMA channel(for USART rx) */
        dma_single_data_para_struct_init(&dma_parameter);
        dma_deinit(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);
        dma_parameter.direction           = DMA_PERIPH_TO_MEMORY;
        dma_parameter.memory0_addr        = 0U;
        dma_parameter.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
        dma_parameter.periph_addr         = (uint32_t)&USART_DATA(USART1);
        dma_parameter.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
        dma_parameter.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
        dma_parameter.number              = 0U;
        dma_parameter.priority            = DMA_PRIORITY_ULTRA_HIGH;
        dma_single_data_mode_init(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL, &dma_parameter);

        /* configure DMA mode */
        dma_circulation_disable(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);

        /* configure DMA periph request */
        dma_channel_subperipheral_select(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL, __USART1_RxDMA_SUBPERI);

        /* enable DMA channel transfer complete interrupt */
//        dma_interrupt_enable(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL, DMA_CHXCTL_FTFIE);

        /* enable DMA1 channel2 */
//        dma_channel_enable(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
    }
}

/**
 * \brief      get USART dma transfer is done or not
 * \prarm[in]  dma_periph
 * \prarm[in]  channelx
 * \param[out] none
 * \retval     none
*/
__STATIC_INLINE FlagStatus usart_dma_done_get(uint32_t dma_periph, dma_channel_enum channelx)
{
    if (!(DMA_CHCTL(dma_periph, channelx) & DMA_CHXCTL_CHEN))
    {
        return SET;
    }
//    else {
//        return dma_flag_get(dma_periph, channelx, DMA_FLAG_FTF);
//    }
    
    return RESET;
}

/**
 * \brief      abort USART dma transfer
 * \prarm[in]  dma_periph
 * \prarm[in]  channelx
 * \param[out] none
 * \retval     none
*/
__STATIC_INLINE void usart_dma_abort(uint32_t dma_periph, dma_channel_enum channelx)
{
    if (DMA_CHCTL(dma_periph, channelx) & DMA_CHXCTL_CHEN)
    {
        dma_channel_disable(dma_periph, channelx);
    }
}

/**
 * \brief      clear USART dma flag
 * \prarm[in]  dma_periph
 * \prarm[in]  channelx
 * \param[out] none
 * \retval     none
*/
__STATIC_INLINE void usart_dma_flag_clear(uint32_t dma_periph, dma_channel_enum channelx)
{
    if (dma_flag_get(dma_periph, channelx, DMA_FLAG_FEE)) {
        dma_flag_clear(dma_periph, channelx, DMA_FLAG_FEE);
    }
    if (dma_flag_get(dma_periph, channelx, DMA_FLAG_SDE)) {
        dma_flag_clear(dma_periph, channelx, DMA_FLAG_SDE);
    }
    if (dma_flag_get(dma_periph, channelx, DMA_FLAG_TAE)) {
        dma_flag_clear(dma_periph, channelx, DMA_FLAG_TAE);
    }
    if (dma_flag_get(dma_periph, channelx, DMA_FLAG_HTF)) {
        dma_flag_clear(dma_periph, channelx, DMA_FLAG_HTF);
    }
    if (dma_flag_get(dma_periph, channelx, DMA_FLAG_FTF)) {
        dma_flag_clear(dma_periph, channelx, DMA_FLAG_FTF);
    }
}

/**
 * \brief      get USART dma transmit is done or not
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
FlagStatus usart_transmit_dma_done_get(uint32_t usart_periph)
{
    if (usart_periph == USART2)
    {
        return usart_dma_done_get(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        return usart_dma_done_get(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);
    }
}

/**
 * \brief      get USART dma receive is done or not
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
FlagStatus usart_receive_dma_done_get(uint32_t usart_periph)
{
    if (usart_periph == USART2)
    {
        return usart_dma_done_get(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        return usart_dma_done_get(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);
    }
}

/**
 * \brief      abort USART dma transmit transfer
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
void usart_transmit_dma_abort(uint32_t usart_periph)
{
    if (usart_periph == USART2)
    {
        usart_dma_abort(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        usart_dma_abort(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);
    }
}

/**
 * \brief      abort USART dma receive transfer
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
void usart_receive_dma_abort(uint32_t usart_periph)
{
    if (usart_periph == USART2)
    {
        usart_dma_abort(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        usart_dma_abort(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);
    }
}

/**
 * \brief      USART dma transmit
 * \prarm[in]  usart_periph
 * \prarm[in]  pdata
 * \prarm[in]  size
 * \param[out] none
 * \retval     none
*/
void usart_transmit_dma(uint32_t usart_periph, uint8_t *pdata, uint16_t size)
{
    if (usart_periph == USART2)
    {
        usart_dma_flag_clear(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);
        dma_channel_disable(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);
        dma_memory_address_config(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL, DMA_MEMORY_0, (uint32_t)pdata);
        dma_transfer_number_config(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL, size);
        dma_channel_enable(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        usart_dma_flag_clear(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);
        dma_channel_disable(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);
        dma_memory_address_config(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL, DMA_MEMORY_0, (uint32_t)pdata);
        dma_transfer_number_config(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL, size);
        dma_channel_enable(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);
    }
}

/**
 * \brief      USART dma receive
 * \prarm[in]  usart_periph
 * \prarm[in]  pdata
 * \prarm[in]  size
 * \param[out] none
 * \retval     none
*/
void usart_receive_dma(uint32_t usart_periph, uint8_t *pdata, uint16_t size)
{
    if (usart_periph == USART2)
    {
        usart_dma_flag_clear(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
        dma_channel_disable(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
        dma_memory_address_config(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL, DMA_MEMORY_0, (uint32_t)pdata);
        dma_transfer_number_config(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL, size);
        dma_channel_enable(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
    }
    else  if (usart_periph == USART1)
    {
        usart_dma_flag_clear(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);
        dma_channel_disable(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);
        dma_memory_address_config(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL, DMA_MEMORY_0, (uint32_t)pdata);
        dma_transfer_number_config(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL, size);
        dma_channel_enable(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);
    }
}

/**
 * \brief      get USART dma transmit remain number
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
__INLINE uint32_t usart_transmit_dma_number_get(uint32_t usart_periph)
{
    if (usart_periph == USART2)
    {
        return dma_transfer_number_get(__USART2_TxDMA_PERIPH, __USART2_TxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        return dma_transfer_number_get(__USART1_TxDMA_PERIPH, __USART1_TxDMA_CHANNEL);
    }

    return 0;
}

/**
 * \brief      get USART dma receive remain number
 * \prarm[in]  usart_periph
 * \param[out] none
 * \retval     none
*/
__INLINE uint32_t usart_receive_dma_number_get(uint32_t usart_periph)
{
    if (usart_periph == USART2)
    {
        return dma_transfer_number_get(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
    }
    else if (usart_periph == USART1)
    {
        return dma_transfer_number_get(__USART1_RxDMA_PERIPH, __USART1_RxDMA_CHANNEL);
    }

    return 0;
}

/**
 * \brief      this function handles DMA channel interrupt
 * \prarm[in]  none
 * \param[out] none
 * \retval     none
*/
//void DMA0_Channel3_IRQHandler(void)
//{
//    if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF))
//    {
//        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);
//    }
//}

//void DMA0_Channel1_IRQHandler(void)
//{
//    if (dma_interrupt_flag_get(DMA0, DMA_CH1, DMA_INT_FLAG_FTF))
//    {
//        dma_interrupt_flag_clear(DMA0, DMA_CH1, DMA_INT_FLAG_FTF);
//    }
//}

/* USART3 init function */
void MX_USART2_UART_Init(void)
{
    usart_parameter_struct usart1_parameter;
    /* stm32 i2c3 <==> gd32 i2c2 */

    /* configure USART gpio */
    usart_gpio_init(USART1);
    
    /* configure USART interface */
    usart_struct_para_init(&usart1_parameter);
    usart1_parameter.baudval         = 115200;
    usart1_parameter.paritycfg       = USART_PM_NONE;
    usart1_parameter.wlen            = USART_WL_8BIT;
    usart1_parameter.stblen          = USART_STB_1BIT;
    usart1_parameter.msbf            = USART_MSBF_LSB;
    usart1_parameter.invertpara      = USART_DINV_DISABLE;
    usart1_parameter.oversamp        = USART_OVSMOD_16;
    usart1_parameter.obsm            = USART_OSB_3bit;
    usart1_parameter.rtimeout_enable = DISABLE;
    usart1_parameter.tx_enable       = USART_TRANSMIT_ENABLE;
    usart1_parameter.rx_enable       = USART_RECEIVE_ENABLE;
    usart_interface_init(USART1, &usart1_parameter);

    /* USART DMA enable*/
    usart_dma_receive_config(USART1, USART_DENR_ENABLE);
    usart_dma_transmit_config(USART1, USART_DENT_ENABLE);

    /* configure DMA */
    usart_dma_init(USART1);

    /* configure USART interrupt */
    usart_interrupt_enable(USART1, USART_INT_IDLE);
    nvic_irq_enable(USART1_IRQn, 4, 0);
    
    /* start receive with dma methods */
    usart_receive_dma(USART1, sUartApp.RecvBuf, UART_RECV_MAX_NUM);
}

void MX_USART1_UART_Init(void)
{
    // huart1.Instance = USART1;
    // huart1.Init.BaudRate = 115200;
    // huart1.Init.WordLength = UART_WORDLENGTH_8B;
    // huart1.Init.StopBits = UART_STOPBITS_1;
    // huart1.Init.Parity = UART_PARITY_NONE;
    // huart1.Init.Mode = UART_MODE_TX_RX;
    // huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    // huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    // huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    // huart1.AdvancedInit.OverrunDisable = (1<<12);
    
    // if (HAL_UART_Init(&huart1) != HAL_OK)
    // {
    //     _Error_Handler(__FILE__, __LINE__);
    // }
  
    // /* USART1 interrupt Init */
    // __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);  
    // HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    // HAL_NVIC_EnableIRQ(USART1_IRQn);

    // __HAL_UART_DISABLE_IT(&huart1,UART_IT_ERR | UART_IT_ORE); 

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{
    usart_parameter_struct usart2_parameter;
    /* stm32 i2c3 <==> gd32 i2c2 */

    /* configure USART gpio */
    usart_gpio_init(USART2);
    
    /* configure USART interface */
    usart_struct_para_init(&usart2_parameter);
    usart2_parameter.baudval         = 115200;
    usart2_parameter.paritycfg       = USART_PM_NONE;
    usart2_parameter.wlen            = USART_WL_8BIT;
    usart2_parameter.stblen          = USART_STB_1BIT;
    usart2_parameter.msbf            = USART_MSBF_LSB;
    usart2_parameter.invertpara      = USART_DINV_DISABLE;
    usart2_parameter.oversamp        = USART_OVSMOD_16;
    usart2_parameter.obsm            = USART_OSB_3bit;
    usart2_parameter.rtimeout_enable = DISABLE;
    usart2_parameter.tx_enable       = USART_TRANSMIT_ENABLE;
    usart2_parameter.rx_enable       = USART_RECEIVE_ENABLE;
    usart_interface_init(USART2, &usart2_parameter);

    /* USART DMA enable*/
    usart_dma_receive_config(USART2, USART_DENR_ENABLE);
    usart_dma_transmit_config(USART2, USART_DENT_ENABLE);

    /* configure DMA */
    usart_dma_init(USART2);

    /* configure USART interrupt */
    usart_interrupt_enable(USART2, USART_INT_IDLE);
    nvic_irq_enable(USART2_IRQn, 2, 0);
    
    /* start receive with dma methods */
    usart_receive_dma(USART2, sUartApp.RecvBuf, UART_RECV_MAX_NUM);
}

void MX_USART3_UART_Init_Weak(void)
{
//     huart3.Instance = USART3;
//     huart3.Init.BaudRate = 115200;
//     huart3.Init.WordLength = UART_WORDLENGTH_8B;
//     huart3.Init.StopBits = UART_STOPBITS_1;
//     huart3.Init.Parity = UART_PARITY_NONE;
//     huart3.Init.Mode = UART_MODE_TX_RX;
//     huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//     huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//     huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
// //    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//     huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
//     huart3.AdvancedInit.OverrunDisable = (1<<12);

//     if (HAL_UART_Init(&huart3) != HAL_OK)
//     {
//         _Error_Handler(__FILE__, __LINE__);
//     }

//     /* USART3 interrupt Init */ 
//     HAL_NVIC_SetPriority(USART3_IRQn, 5, 1);
//     HAL_NVIC_EnableIRQ(USART3_IRQn);

//     __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 
	 
// //	__HAL_UART_DISABLE_IT(&huart3,UART_IT_ERR | UART_IT_ORE); 
	        
//     HAL_UART_Receive_DMA(&huart3, sUartApp.RecvBuf, UART_RECV_MAX_NUM);
    
  
}

// void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
// {

//   GPIO_InitTypeDef GPIO_InitStruct;
//   if(uartHandle->Instance==USART1)
//   {
//   /* USER CODE BEGIN USART1_MspInit 0 */

//   /* USER CODE END USART1_MspInit 0 */
//     /* USART1 clock enable */
//     __HAL_RCC_USART1_CLK_ENABLE();
  
//     /**USART1 GPIO Configuration    
//     PA9     ------> USART1_TX
//     PA10     ------> USART1_RX 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* USER CODE BEGIN USART1_MspInit 1 */

//   /* USER CODE END USART1_MspInit 1 */
//   }
//   else if(uartHandle->Instance==USART3)
//   {
//   /* USER CODE BEGIN USART3_MspInit 0 */

//   /* USER CODE END USART3_MspInit 0 */
//     /* USART3 clock enable */
//     __HAL_RCC_USART3_CLK_ENABLE();
  
//     /**USART3 GPIO Configuration    
//     PD8     ------> USART3_TX
//     PD9     ------> USART3_RX 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//     HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
      
//     /* USART3 DMA Init */
//     /* USART3_RX Init */
//     hdma_usart3_rx.Instance = DMA1_Stream1;
//     hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
//     hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//     hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
//     hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//     hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//     hdma_usart3_rx.Init.Mode = DMA_NORMAL;
//     hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
//     hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
//     {
//       _Error_Handler(__FILE__, __LINE__);
//     }

//     __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

//     /* USART3_TX Init */
//     hdma_usart3_tx.Instance = DMA1_Stream3;
//     hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
//     hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//     hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
//     hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//     hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//     hdma_usart3_tx.Init.Mode = DMA_NORMAL;
//     hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
//     hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
//     {
//       _Error_Handler(__FILE__, __LINE__);
//     }

//     __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

//   }

// }

// void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
// {

//   if(uartHandle->Instance==USART1)
//   {
//   /* USER CODE BEGIN USART1_MspDeInit 0 */

//   /* USER CODE END USART1_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_USART1_CLK_DISABLE();
  
//     /**USART1 GPIO Configuration    
//     PA9     ------> USART1_TX
//     PA10     ------> USART1_RX 
//     */
//     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

//   /* USER CODE BEGIN USART1_MspDeInit 1 */

//   /* USER CODE END USART1_MspDeInit 1 */
//   }
//   else if(uartHandle->Instance==USART3)
//   {
//   /* USER CODE BEGIN USART3_MspDeInit 0 */

//   /* USER CODE END USART3_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_USART3_CLK_DISABLE();
  
//     /**USART3 GPIO Configuration    
//     PD8     ------> USART3_TX
//     PD9     ------> USART3_RX 
//     */
//     HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

//   /* USART3 DMA DeInit */
//     HAL_DMA_DeInit(uartHandle->hdmatx);

//     /* USART3 interrupt Deinit */
//     HAL_NVIC_DisableIRQ(USART3_IRQn);
//   /* USER CODE BEGIN USART3_MspDeInit 1 */

//   /* USER CODE END USART3_MspDeInit 1 */
//   }

// } 


/***********************************************************************
 * DESCRIPTION:This function handles USART3 global interrupt.
 *             NVIC_Priority : (5, 1)
 * RETURNS:
 *
***********************************************************************/

void USART1_IRQHandler(void)
{
    uint32_t temp = 0;
    uint8_t *p = NULL;
    uint16_t size = 0;

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_PERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_PERR);
//    }

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_ERR_FERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_ERR_FERR);
//    }

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_ERR_NERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_ERR_NERR);
//    }

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_ERR_ORERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_ERR_ORERR);
//    }

    if (usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_IDLE);
        
        usart_receive_dma_abort(USART1);
        temp = USART_DATA(USART1);
        temp = usart_receive_dma_number_get(USART1);
        sUartApp.RecvBufNum += UART_RECV_MAX_NUM - temp;

        if ((0xAA == sUartApp.RecvBuf[0]) && (sUartApp.RecvBufNum > 0))
        {
            if (sUartApp.RecvBufNum > 3)
            {
                uint16_t FrameLen = sUartApp.RecvBuf[1] | (sUartApp.RecvBuf[2] << 8);
                if ((FrameLen+1) <= sUartApp.RecvBufNum)
                {
                    UartRecvDispatch(sUartApp.RecvBuf);

                    sUartApp.RecvBufNum = 0;
                    p = sUartApp.RecvBuf;
                    size = UART_RECV_MAX_NUM;
                }
                else {
                    p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
                    size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
                } // end if((FrameLen+1)..
            }
            else {
                p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
                size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
            } // end if(sUartApp.RecvBufNum..
        }
        /*华大串口透传colson临时使用 start*/
//        else if ((0x57 == sUartApp.RecvBuf[0]) && (sUartApp.RecvBufNum > 0))
//        {
//            if (sUartApp.RecvBufNum >= 15)
//            {
//                void Uart_AutoChargeApp_RecvDispatch(UINT8 *data, UINT16 datalen);
//                    Uart_AutoChargeApp_RecvDispatch(sUartApp.RecvBuf,sUartApp.RecvBufNum );

//                    sUartApp.RecvBufNum = 0;
//                    p = sUartApp.RecvBuf;
//                    size = UART_RECV_MAX_NUM;

//            }
//            else {
//                p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
//                size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
//            } // end if(sUartApp.RecvBufNum..
//        }
//        else {
//            sUartApp.RecvBufNum = 0;
//            p = sUartApp.RecvBuf;
//            size = UART_RECV_MAX_NUM;
//        } // end if((0xAA == sUartApp..
        /*华大串口透传colson临时使用 end*/

        usart_receive_dma(USART1, p, size);
    }
}

/***********************************************************************
 * DESCRIPTION:This function handles USART3 global interrupt.
 *             NVIC_Priority : (5, 1)
 * RETURNS:
 *
***********************************************************************/

void USART2_IRQHandler(void)
{
    uint32_t temp = 0;
    uint8_t *p = NULL;
    uint16_t size = 0;

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_PERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_PERR);
//    }

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_ERR_FERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_ERR_FERR);
//    }

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_ERR_NERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_ERR_NERR);
//    }

//    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_ERR_ORERR)) {
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_ERR_ORERR);
//    }

    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE)) 
		{
//        usart_interrupt_flag_clear(USART2, USART_INT_FLAG_IDLE);
        
        usart_receive_dma_abort(USART2);
        temp = USART_DATA(USART2);
        temp = dma_transfer_number_get(__USART2_RxDMA_PERIPH, __USART2_RxDMA_CHANNEL);
        sUartApp.RecvBufNum += UART_RECV_MAX_NUM - temp;

        if ((0x57 == sUartApp.RecvBuf[0]) && (sUartApp.RecvBufNum > 0))
        {
            if (sUartApp.RecvBufNum >= 15)
            {
                void Uart_AutoChargeApp_RecvDispatch(UINT8 *data, UINT16 datalen);
                    Uart_AutoChargeApp_RecvDispatch(sUartApp.RecvBuf,sUartApp.RecvBufNum );

                    sUartApp.RecvBufNum = 0;
                    p = sUartApp.RecvBuf;
                    size = UART_RECV_MAX_NUM;

            }
            else {
                p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
                size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
            } // end if(sUartApp.RecvBufNum..
        }
        else 
				{
            sUartApp.RecvBufNum = 0;
            p = sUartApp.RecvBuf;
            size = UART_RECV_MAX_NUM;
        } // end if((0xAA == sUartApp..
        /*华大串口透传colson临时使用 end*/

        usart_receive_dma(USART2, p, size);
    }
}

void USART3_IRQHandler_Weak(void)    
{
    // uint32_t temp=0;
    // uint8_t *p=NULL;
    // uint16_t size=0;   
     
    // // if occur ORE NF FE PE error, then clear
    // if(huart3.Instance->ISR & 0xF)
    // {    
    //     huart3.Instance->ICR = 0xF;
    // }
    
    // HAL_UART_IRQHandler(&huart3);
        
    // if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))
    // { 
        
    //     __HAL_UART_CLEAR_IDLEFLAG(&huart3);   
    //     HAL_UART_DMAStop(&huart3);                     
    //     temp = huart3.Instance->ISR;  
    //     temp = huart3.Instance->RDR; 
    //     temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);  
        
    //     sUartApp.RecvBufNum +=  UART_RECV_MAX_NUM - temp; 
     
    //     if( (0xAA == sUartApp.RecvBuf[0]) && (sUartApp.RecvBufNum > 0) )
    //     {
    //         if(sUartApp.RecvBufNum > 3)
    //         {
    //             uint16_t FrameLen = sUartApp.RecvBuf[1] | (sUartApp.RecvBuf[2]<<8);
    //             if((FrameLen+1) <= sUartApp.RecvBufNum)
    //             {
                    
    //                 UartRecvDispatch(sUartApp.RecvBuf);  
                    
    //                 sUartApp.RecvBufNum = 0;
    //                 p = sUartApp.RecvBuf;
    //                 size = UART_RECV_MAX_NUM;                    
    //             }
    //             else
    //             {
    //                 p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
    //                 size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
    //             } // end if((FrameLen+1)..
    //         }
    //         else
    //         {
    //             p = sUartApp.RecvBuf + sUartApp.RecvBufNum;
    //             size = UART_RECV_MAX_NUM - sUartApp.RecvBufNum;
    //         } // end if(sUartApp.RecvBufNum..  
    //     }
    //     else
    //     {
    //         sUartApp.RecvBufNum = 0;
    //         p = sUartApp.RecvBuf;
    //         size = UART_RECV_MAX_NUM;
            
    //     } // end if((0xAA == sUartApp..
           
    //     HAL_UART_Receive_DMA(&huart3, p, size);
        
    // } // end if((__HAL_UART_GET_FLAG...

}
 

/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void UartSendData(uint8_t *buf, uint16_t size)
{
    UINT32 StartTime = ReadTimeStampTimer();

    while (SET != usart_transmit_dma_done_get(USART1))
    {
        if ((ReadTimeStampTimer() - StartTime) > 2*100*5000000)  // 5s
        {
            break;
        }
    }
    usart_transmit_dma(USART1, buf, size);
    
    StartTime = ReadTimeStampTimer();
    while (SET != usart_transmit_dma_done_get(USART1))
    {
        if ((ReadTimeStampTimer() - StartTime) > 2*100*5000000)  // 5s
        {
            break;
        }
    }
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void UartSendData2(uint8_t *buf, uint16_t size)
{
    UINT32 StartTime = ReadTimeStampTimer();

    while (SET != usart_transmit_dma_done_get(USART2))
    {
        if ((ReadTimeStampTimer() - StartTime) > 2*100*5000000)  // 5s
        {
            break;
        }
    }
    usart_transmit_dma(USART2, buf, size);
    
    StartTime = ReadTimeStampTimer();
    while (SET != usart_transmit_dma_done_get(USART2))
    {
        if ((ReadTimeStampTimer() - StartTime) > 2*100*5000000)  // 5s
        {
            break;
        }
    }
}

void UartSendData_Weak(uint8_t * buf, uint16_t size)
{
  //   UINT32 StartTime = ReadTimeStampTimer();
	// while(huart3.gState!=HAL_UART_STATE_READY) 
  //   {
  //       if((ReadTimeStampTimer() - StartTime) > 27*5000000)  // 5s
  //       {
  //           break;
  //       }
  //   }
  //   HAL_UART_Transmit_DMA(&huart3, buf, size);
    
  //   StartTime = ReadTimeStampTimer();
  //   while (HAL_DMA_GetState(&hdma_usart3_tx) != HAL_DMA_STATE_READY)
  //   {
  //       if((ReadTimeStampTimer() - StartTime) > 27*5000000)  // 5s
  //       {
  //           break;
  //       }
  //   }

}








/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
