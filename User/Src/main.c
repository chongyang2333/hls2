/*!
    \file    main.c
    \brief   Demo--GPIO
    
    \version 
*/

#include "gd32f4xx.h"
#include "delay.h"
#include <stdio.h>
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "gpio.h"
#include "can.h"
#include "HardApi.h"
#include "ControlRun.h"
#include "Param.h"
#include "Eeprom.h"
#include "UartApp.h"
#include "CanApp.h"
#include "StateMachine.h"
#include "Gyro.h"
#include "Rgb.h"
#include "ErrorLog.h"
#include "Temperature.h"
#include "BootloaderInfo.h"
#include "PowerManager.h"
#include "LedDriver.h"
#include "gd_hal.h"

extern struct AxisCtrlStruct sAxis[MAX_AXIS_NUM];

//extern UART_HandleTypeDef huart3;
//extern UART_HandleTypeDef huart6;
uint8_t aTxBuffer[10] = {0xaa,0x02,0x0a,0x0b,0x03,0x04,0x05,0xc,0x0d,0xe};

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f) 


UINT32 LoopElapsedTime = 0;
UINT32 MaxLoopTime = 0;
void HardwareInit(void);
BootLoaderInfo bootloaderInfo={0};

ST_VersionStruct NowSoftWareVersion = {21, 0, 0};

void CAN_MesIAPResetTreatment(BootLoaderInfo* pstbootloaderInfo);

PUBLIC UINT8 ApplicationMode = 0;

uint32_t TestSysClock = 0;

int main()
{  
    HardwareInit();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            HardwareInit();
    
    __set_PRIMASK( 0 ); // 开启总中断
    __set_FAULTMASK( 0 ); // 没关异常

    while (1)
    {
        UINT32 LoopStartTime = ReadTimeStampTimer();         
        DataCollectSendLoop();// uart send data collect data 
        ErrorLogExec(sAxis[0].sAlarm.ErrReg.all, sAxis[1].sAlarm.ErrReg.all);        // Error log task
        ParamLoop();
        TemperatureExec();
        BatteryInfoReadLoop();
        LoopElapsedTime = ReadTimeStampTimer() - LoopStartTime;
        if(MaxLoopTime < LoopElapsedTime)
        {
            MaxLoopTime = LoopElapsedTime;
        }
				
        CAN_MesIAPResetTreatment(&bootloaderInfo);
    }
    
}
/***********************************************************************
 * DESCRIPTION:
 *
 * RETURNS:
 *
***********************************************************************/
void CAN_MesIAPResetTreatment(BootLoaderInfo* pstbootloaderInfo)
{
		if(pstbootloaderInfo->F_Reset == 1)
		{				
				pstbootloaderInfo->F_Reset = 0;
				CMDSoftWareTreatment((CAN_RX_Message*)&pstbootloaderInfo->CANMessage[0]);
		}
}


void HardwareInit()
{
        NVIC_SetPriorityGrouping(NVIC_PRIGROUP_PRE4_SUB0);     
        
		/* Initialize all configured peripherals */
		ApplicationMode = MX_GPIO_Init();
    
		/* Initialize tim4 for timestamp*/
		TimeStampTimerInit();

        /* Initialize eeprom */
		EepromInit();
		GetLastSoftwareVersion(&bootloaderInfo);
 		WriteSoftWareVersion(&bootloaderInfo,&NowSoftWareVersion);
        /* Initialize eeprom parameter*/
		ParamInit();
		CanAppInit();
    
        /* Initialize can module*/
        MX_CAN1_Init();
        
		AdcInit();		
		PowerManagerInit(ApplicationMode);
        
		/* Initialize DMA for usart tx */
		MX_DMA_Init();  
		/* Initialize adc:for fhase current,DC current,DC voltage and temperature sample*/
		//AdcInit();
		/* Initialize Left Axis Encoder TIM*/
		MX_TIM3_Init(); 
		/* Initialize Right Axis Encoder TIM*/
		//    MX_TIM3_Init();
		MX_TIM2_Init();
		/* Initialize general timer interrupt  40Hz*/
		MX_TIM8_Init();

		/* Initialize usart3 module: for pc comm*/
		MX_USART3_UART_Init();
         
		/* Initialize Gyro module.(MPU6050) */
		GyroInit();
        
		/* Initialize LedDriver module.(tlc59108f) */
		LedDriverInit();
        
		/* Initialize motor control module*/
		ControlRunInit();
		/* Initialize pwm:for motor drive*/
		PwmInit();
		/* Enable pwm timer interrupt */
//		HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); 
//		/* Enable general timer interrupt */
//		HAL_NVIC_EnableIRQ(TIM7_IRQn);
        timer_interrupt_enable(TIMER8,TIMER_INT_UP);
        timer_interrupt_enable(TIMER0,TIMER_INT_UP);
        /* Enable EXTI4 interrupt */
//        HAL_NVIC_EnableIRQ(EXTI4_IRQn); 
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1,100);

  return ch;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

