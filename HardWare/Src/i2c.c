/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include <assert.h>
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* i2c initiliaze parameters struct */
typedef struct
{
    uint32_t clkspeed;                   /*!< I2C clock speed */ 
    uint32_t dutycyc;                    /*!< I2C duty cycle in fast mode */ 
    uint32_t mode;                       /*!< I2C working mode */
    uint32_t addformat;                  /*!< I2C 7bits or 10 bits */
    uint32_t addr;                       /*!< I2C address */
    uint32_t ack;                        /*!< I2C acknowledge enable */
} i2c_parameter_struct;

#define I2C_MEM_ADD_MSB(__ADDRESS__)    ((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0xFF00U))) >> 8U)))
#define I2C_MEM_ADD_LSB(__ADDRESS__)    ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))


i2c_parameter_struct i2c1_parameter;
i2c_parameter_struct i2c2_parameter;

// I2C_HandleTypeDef hi2c2;
// I2C_HandleTypeDef hi2c3;


/**
 * \brief      write an amount fo data blocking mode to a specific memory address
 * \prarm[in]  i2c_periph I2C periph base address
 * \prarm[in]  dev_address arget device address which contain device 7 or 10 bits address value
 *             in datasheet must be shifted to the left before calling the interface
 * \prarm[in]  mem_address Internal memory address
 * \prarm[in]  mem_addsize Size of internal memory address
 * \prarm[in]  pdata Pointer to data buffer
 * \prarm[in]  size Amount of data to be sent
 * \prarm[in]  timeout Timeout duration
 * \param[out] none
 * \retval     -1 for timeout, 0 is normal
*/
int8_t i2c_mem_write (
    uint32_t i2c_periph, 
    uint16_t dev_address, 
    uint16_t mem_address, 
    uint16_t mem_addsize, 
    uint8_t *pdata, 
    uint16_t size, 
    uint32_t timeout
)
{
    uint32_t time;
    int8_t err = 0;
    uint16_t write_size = size;
    
    assert(pdata != NULL);
    assert(size != 0);

    /* wait until I2C bus is idle */
    time = timeout;
    while (SET == i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)) {
        if (time--) {
            return -1;
        }
    }
    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
        if (time--) {
            return -1;
        }
    }

    /* send a device address (write) to I2C bus */
    i2c_master_addressing(i2c_periph, dev_address, I2C_TRANSMITTER);
    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
        if (time--) {
            return -1;
        }
    }
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
    
    /* wait until the transmit data buffer is empty */
    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
        if (time--) {
            return -1;
        }
    }

    /* send a memory address to I2C bus */
    if (mem_addsize == I2C_MEMADD_SIZE_8BIT) {
        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_LSB(mem_address));	
        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (time--) {
                return -1;
            }
        }
    }
    else {
        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_MSB(mem_address));	
        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (time--) {
                return -1;
            }
        }

        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_LSB(mem_address));	
        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (time--) {
                return -1;
            }
        }
    }

    /* send memory data to I2C bus */
    while (write_size--) {
        i2c_data_transmit(i2c_periph, (uint8_t)*pdata);	
        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (time--) {
                return -1;
            }
        }
    }

    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(i2c_periph);
    time = timeout;
    while (I2C_CTL0(i2c_periph) & 0x0200) {
        if (time--) {
            return -1;
        }
    }

    return err;
}

/**
 * \brief      write an amount fo data blocking mode to a specific memory address
 * \prarm[in]  i2c_periph I2C periph base address
 * \prarm[in]  dev_address arget device address which contain device 7 or 10 bits address value
 *             in datasheet must be shifted to the left before calling the interface
 * \prarm[in]  mem_address Internal memory address
 * \prarm[in]  mem_addsize Size of internal memory address
 * \prarm[in]  pdata Pointer to data buffer
 * \prarm[in]  size Amount of data to be sent
 * \prarm[in]  timeout Timeout duration
 * \param[out] none
 * \retval     -1 for timeout, 0 is normal
*/
int8_t i2c_mem_read (
    uint32_t i2c_periph, 
    uint16_t dev_address, 
    uint16_t mem_address, 
    uint16_t mem_addsize, 
    uint8_t *pdata, 
    uint16_t size, 
    uint32_t timeout
)
{
    uint32_t time;
    int8_t err = 0;
    uint16_t read_size = size;

    assert(pdata != NULL);
    assert(size != 0);

    /* wait until I2C bus is idle */
    time = timeout;
    while (SET == i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)) {
        if (time--) {
            return -1;
        }
    }
    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
        if (time--) {
            return -1;
        }
    }

    /* send a device address (write) to I2C bus */
    i2c_master_addressing(i2c_periph, dev_address, I2C_TRANSMITTER);
    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
        if (time--) {
            return -1;
        }
    }
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
    
    /* wait until the transmit data buffer is empty */
    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
        if (time--) {
            return -1;
        }
    }

    /* send a memory address to I2C bus */
    if (mem_addsize == I2C_MEMADD_SIZE_8BIT) {
        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_LSB(mem_address));	
        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (time--) {
                return -1;
            }
        }
    }
    else {
        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_MSB(mem_address));	
        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (time--) {
                return -1;
            }
        }

        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_LSB(mem_address));	
        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) {
            if (time--) {
                return -1;
            }
        }
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)) {
        if (time--) {
            return -1;
        }
    }

    /* send a device address (read) to I2C bus */
    i2c_master_addressing(i2c_periph, dev_address, I2C_RECEIVER);
    
    /*!< there are three methods: 
         *master_receiver_one_byte
         *master_receiver_two_byte
         *master_receiver_multi_byte */    

    if (size == 2) {
        /* disable acknowledge */
        i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
    }

    time = timeout;
    while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)) {
        if (time--) {
            return -1;
        }
    }
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    if (size == 1) {
        /* disable acknowledge */
        i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);

        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(i2c_periph);

        time = timeout;
        while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_RBNE)) {
            if (time--) {
                return -1;
            }
        }
        *(uint8_t *)pdata = i2c_data_receive(i2c_periph);

        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(i2c_periph);
        time = timeout;
        while (I2C_CTL0(i2c_periph) & 0x0200) {
            if (time--) {
                return -1;
            }
        }
        
        /* enable acknowledge */
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
    }
    else {
        if (size == 2) {
            /* wait until the last data byte is received into the shift register */
            time = timeout;
            while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_BTC)) {
                if (time--) {
                    return -1;
                }
            }
        }

        /* recv memory data to I2C bus */
        while (read_size--) {
            if ((size >= 3) && (read_size == 2)) {
                /* wait until the second last data byte is received into the shift register */
                time = timeout;
                while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_BTC)) {
                    if (time--) {
                        return -1;
                    }
                }
                /* disable acknowledge */
                i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
            }

            time = timeout;
            while (SET != i2c_flag_get(i2c_periph, I2C_FLAG_RBNE)) {
                if (time--) {
                    return -1;
                }
            }
            *(uint8_t *)pdata++ = i2c_data_receive(i2c_periph);
        }

        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(i2c_periph);
        time = timeout;
        while (I2C_CTL0(i2c_periph) & 0x0200) {
            if (time--) {
                return -1;
            }
        }

        /* enable acknowledge */
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
    }

    return err;
}

/**
 * \brief      initialize I2C parameter struct with a default value
 * \prarm[in]  p_struct: the pointer of the specific struct 
 * \param[out] none
 * \retval     none
*/
static void i2c_struct_para_init(void *p_struct)
{
    ((i2c_parameter_struct *)p_struct)->clkspeed  = 100000;
    ((i2c_parameter_struct *)p_struct)->dutycyc   = I2C_DTCY_2;
    ((i2c_parameter_struct *)p_struct)->mode      = I2C_I2CMODE_ENABLE;
    ((i2c_parameter_struct *)p_struct)->addformat = I2C_ADDFORMAT_7BITS; 
    ((i2c_parameter_struct *)p_struct)->addr      = 0x00; 
    ((i2c_parameter_struct *)p_struct)->ack       = I2C_ACK_ENABLE;
}

/**
 * \brief      initialize I2C interface
 * \prarm[in]  i2c_periph
 * \prarm[in]  i2c_parameter_init
 * \param[out] none
 * \retval     none
*/
static void i2c_interface_init(uint32_t i2c_periph, i2c_parameter_struct *i2c_parameter_init)
{
    /* reset I2C */
    i2c_deinit(i2c_periph);
    
    /* configure I2C clock */
    i2c_clock_config(i2c_periph, i2c_parameter_init->clkspeed, i2c_parameter_init->dutycyc);

    /* configure I2C address */
    i2c_mode_addr_config(i2c_periph, i2c_parameter_init->mode, i2c_parameter_init->addformat, i2c_parameter_init->addr);

    /* enable I2C */
    i2c_enable(i2c_periph);

    /* enable acknowledge */
    i2c_ack_config(i2c_periph, i2c_parameter_init->ack);
}

/**
 * \brief      initialize I2C gpio pin
 * \prarm[in]  i2c_periph
 * \param[out] none
 * \retval     none
*/
static void i2c_gpio_init(uint32_t i2c_periph)
{
    if (i2c_periph == I2C0)
    {
        #define __I2C0_SCL_PORT  GPIOB
        #define __I2C0_SCL_GPIO  GPIO_PIN_8
        #define __I2C0_SCL_AF    GPIO_AF_4
        #define __I2C0_SDA_PORT  GPIOB
        #define __I2C0_SDA_GPIO  GPIO_PIN_9
        #define __I2C0_SDA_AF    GPIO_AF_4

        /* enable can clock */
        rcu_periph_clock_enable(RCU_I2C0);
        rcu_periph_clock_enable(RCU_GPIOB);

        /* configure I2C GPIO */
        gpio_output_options_set(__I2C0_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C0_SCL_GPIO);
        gpio_mode_set(__I2C0_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __I2C0_SCL_GPIO);
        gpio_af_set(__I2C0_SCL_PORT, __I2C0_SCL_AF, __I2C0_SCL_GPIO);
        
        gpio_output_options_set(__I2C0_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C0_SDA_GPIO);
        gpio_mode_set(__I2C0_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __I2C0_SDA_GPIO);
        gpio_af_set(__I2C0_SDA_PORT, __I2C0_SDA_AF, __I2C0_SDA_GPIO);
    }
    else if (i2c_periph == I2C1) {
        #define __I2C1_SCL_PORT  GPIOB
        #define __I2C1_SCL_GPIO  GPIO_PIN_10
        #define __I2C1_SCL_AF    GPIO_AF_4
        #define __I2C1_SDA_PORT  GPIOB
        #define __I2C1_SDA_GPIO  GPIO_PIN_11
        #define __I2C1_SDA_AF    GPIO_AF_4

        /* enable can clock */
        rcu_periph_clock_enable(RCU_I2C1);
        rcu_periph_clock_enable(RCU_GPIOB);

        /* configure I2C GPIO */
        gpio_output_options_set(__I2C1_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C1_SCL_GPIO);
        gpio_mode_set(__I2C1_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __I2C1_SCL_GPIO);
        gpio_af_set(__I2C1_SCL_PORT, __I2C1_SCL_AF, __I2C1_SCL_GPIO);
        
        gpio_output_options_set(__I2C1_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C1_SDA_GPIO);
        gpio_mode_set(__I2C1_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __I2C1_SDA_GPIO);
        gpio_af_set(__I2C1_SDA_PORT, __I2C1_SDA_AF, __I2C1_SDA_GPIO);
    }
    else if (i2c_periph == I2C2) {
        #define __I2C2_SCL_PORT  GPIOA
        #define __I2C2_SCL_GPIO  GPIO_PIN_8
        #define __I2C2_SCL_AF    GPIO_AF_4
        #define __I2C2_SDA_PORT  GPIOC
        #define __I2C2_SDA_GPIO  GPIO_PIN_9
        #define __I2C2_SDA_AF    GPIO_AF_4

        /* enable can clock */
        rcu_periph_clock_enable(RCU_I2C2);
        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_GPIOC);

        /* configure I2C GPIO */
        gpio_output_options_set(__I2C2_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C2_SCL_GPIO);
        gpio_mode_set(__I2C2_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __I2C2_SCL_GPIO);
        gpio_af_set(__I2C2_SCL_PORT, __I2C2_SCL_AF, __I2C2_SCL_GPIO);
        
        gpio_output_options_set(__I2C2_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, __I2C2_SDA_GPIO);
        gpio_mode_set(__I2C2_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, __I2C2_SDA_GPIO);
        gpio_af_set(__I2C2_SDA_PORT, __I2C2_SDA_AF, __I2C2_SDA_GPIO);
    }
}


void MX_I2C2_Init(void)
{
    /* stm32 i2c2 <==> gd32 i2c1 */
    
    /* configure I2C gpio */
    i2c_gpio_init(I2C1);

    /* initialize I2C interface */
    i2c_struct_para_init(&i2c1_parameter);
    i2c1_parameter.clkspeed  = 100000;
    i2c1_parameter.dutycyc   = I2C_DTCY_2;
    i2c1_parameter.mode      = I2C_I2CMODE_ENABLE;
    i2c1_parameter.addformat = I2C_ADDFORMAT_7BITS; 
    i2c1_parameter.addr      = 0x00; 
    i2c1_parameter.ack       = I2C_ACK_ENABLE;
    i2c_interface_init(I2C1, &i2c1_parameter);
}

void MX_I2C3_Init(void)
{
    /* stm32 i2c3 <==> gd32 i2c2 */
    
    /* configure I2C gpio */
    i2c_gpio_init(I2C2);

    /* initialize I2C interface */
    i2c_struct_para_init(&i2c2_parameter);
    i2c2_parameter.clkspeed  = 100000;
    i2c2_parameter.dutycyc   = I2C_DTCY_2;
    i2c2_parameter.mode      = I2C_I2CMODE_ENABLE;
    i2c2_parameter.addformat = I2C_ADDFORMAT_7BITS; 
    i2c2_parameter.addr      = 0x00; 
    i2c2_parameter.ack       = I2C_ACK_ENABLE;
    i2c_interface_init(I2C2, &i2c2_parameter);
}

/* I2C2 init function */
void MX_I2C2_Init_Weak(void)
{

  // hi2c2.Instance = I2C2;
  // hi2c2.Init.Timing = 0x00701F5F;
  // hi2c2.Init.OwnAddress1 = 0;
  // hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  // hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  // hi2c2.Init.OwnAddress2 = 0;
  // hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  // hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  // hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  // if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  // {
  //   _Error_Handler(__FILE__, __LINE__);
  // }

}

/* I2C3 init function */
void MX_I2C3_Init_Weak(void)
{
      // hi2c3.Instance = I2C3;
      // hi2c3.Init.Timing = 0x200FC467;
      // hi2c3.Init.OwnAddress1 = 0;
      // hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
      // hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
      // hi2c3.Init.OwnAddress2 = 0;
      // hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
      // hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
      // hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
      // if (HAL_I2C_Init(&hi2c3) != HAL_OK)
      // {
      //       _Error_Handler(__FILE__, __LINE__);
      // }
      // /** Configure Analogue filter 
      // */
      // if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
      // {
      //       _Error_Handler(__FILE__, __LINE__);
      // }
      // /** Configure Digital filter 
      // */
      // if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
      // {
      //       _Error_Handler(__FILE__, __LINE__);
      // }
}


// void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
// {
//   GPIO_InitTypeDef GPIO_InitStruct;
//   if(i2cHandle->Instance==I2C2)
//   {
//     /**I2C2 GPIO Configuration    
//     PF0     ------> I2C2_SCL
//     PF1     ------> I2C2_SDA 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
//     HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

//     /* I2C2 clock enable */
//     __HAL_RCC_I2C2_CLK_ENABLE();
//   /* USER CODE BEGIN I2C2_MspInit 1 */

//   /* USER CODE END I2C2_MspInit 1 */
//   }
//   else if(i2cHandle->Instance==I2C3)
//   { 
//     /**I2C3 GPIO Configuration    
//     PA8     ------> I2C3_SCL
//     PC9     ------> I2C3_SDA 
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_8;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      
//     GPIO_InitStruct.Pin = GPIO_PIN_9;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
//     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
//     i2cHandle->Instance->CR1 = (1<<15);
//     i2cHandle->Instance->CR1 = 0;
    

//     /* I2C3 clock enable */
//     __HAL_RCC_I2C3_CLK_ENABLE();

//   }
// }

// void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
// {

//   if(i2cHandle->Instance==I2C2)
//   {
//   /* USER CODE BEGIN I2C2_MspDeInit 0 */

//   /* USER CODE END I2C2_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_I2C2_CLK_DISABLE();
  
//     /**I2C2 GPIO Configuration    
//     PF0     ------> I2C2_SCL
//     PF1     ------> I2C2_SDA 
//     */
//     HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0|GPIO_PIN_1);

//   /* USER CODE BEGIN I2C2_MspDeInit 1 */

//   /* USER CODE END I2C2_MspDeInit 1 */
//   }
//   else if(i2cHandle->Instance==I2C3)
//   { 
//       __HAL_RCC_I2C3_CLK_DISABLE();
      
//       HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
//       HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
//   }
  
// } 

/* USER CODE BEGIN 1 */
void HAL_I2C_Reset(uint32_t i2c_periph)
{
    if (i2c_periph == I2C1)
    {
        i2c_deinit(i2c_periph);
        i2c_interface_init(i2c_periph, &i2c1_parameter);
    }
    else if (i2c_periph == I2C2) {
        i2c_deinit(i2c_periph);
        i2c_interface_init(i2c_periph, &i2c2_parameter);
    }
}

void HAL_I2C_Reset_Weak(uint32_t i2c_periph)
{
//    HAL_I2C_DeInit(i2cHandle);
//    HAL_I2C_Init(i2cHandle);
} 

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
