/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uplinkcomms.h"
#include <stdio.h>
#include <vl53l0x.h>

#define SENSOR_POLL_INTERVAL_MS 50



/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandleX;

//__IO uint8_t receivedChar;
//__IO uint8_t UartReceivedFlag = RESET;
//__IO uint8_t rxBuffer[RXBUFFERSIZE];
//__IO uint16_t rxLength = 0;
//__IO uint16_t wrIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static uint8_t getModbusSlaveID(void);
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

uint8_t rec_char[1];

uint16_t distanceFiltered = 0;
uint16_t currentDistanceValue = 0;


int main(void)
{
  /* This sample code shows how to use GPIO HAL API to toggle LED3 IO
    in an infinite loop. */

  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  BSP_LED_Init(LED3);
  
  GPIO_Init();

  I2C1_Init();

  VL53L0X_init(true);

  VL53L0X_setTimeout(500);

  VL53L0X_startContinuous(0);


  /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
        - Word Length = 8 Bits (7 data bit + 1 parity bit) :
                      BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
        - Stop Bit    = One Stop bit
        - Parity      = No parity
        - BaudRate    = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandleX.Instance        = USARTx;

    UartHandleX.Init.BaudRate   = 115200;
    UartHandleX.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandleX.Init.StopBits   = UART_STOPBITS_1;
    UartHandleX.Init.Parity     = UART_PARITY_NONE;
    UartHandleX.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandleX.Init.Mode       = UART_MODE_TX_RX;
    if (HAL_UART_Init(&UartHandleX) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
    /*##-4- Put UART peripheral in reception process ###########################*/
    if(HAL_UART_Receive_IT(&UartHandleX, rec_char, 1) != HAL_OK)
    {
    Error_Handler();
    }
    /* Output a message on Hyperterminal using printf function */

	uint8_t modbusSlaveID = getModbusSlaveID();

    print("\n\r Proximity Sensor Startup with ID: ");
	print_number(modbusSlaveID, "%d");
    print("\n\r");

    uplinkComms_init();

    /* Infinite loop */
    while (1)
    {
    	static uint32_t lastSensorCheckedTime = 0;

    	uint32_t currentTime = HAL_GetTick();

    	if((currentTime - lastSensorCheckedTime) >= SENSOR_POLL_INTERVAL_MS){

    		lastSensorCheckedTime = currentTime;
           	if (VL53L0X_timeoutOccurred()) {
            		print("Sensor TIMEOUT \r\n");
            } else {
            	uint16_t distance = VL53L0X_readRangeContinuousMillimeters();

            	//Only accept values within defined range
            	if(distance != 0 && distance < 3000){
            		currentDistanceValue = distance;
            		distanceFiltered = distanceFiltered * 0.6 + currentDistanceValue * 0.4;
            	}
            }

        	print("ID: ");
        	print_number(modbusSlaveID, "%d");
        	print(", Distance: ");
        	print_number(distanceFiltered, "%d");
        	print("mm");
        	print("\r\n");

    	}





    	upLinkComms_regularProcessing(modbusSlaveID, distanceFiltered);


    }
}

static uint8_t getModbusSlaveID(void)
{

	uint8_t array[] = {0,0,0,0,0,0,0,0};

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {		// PB0 -> DIP Switch 8th bit
		array[7] = 1;
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {		// PB7 -> DIP Switch 7th bit
		array[6] = 1;

	}
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) {		// PB6 -> DIP Switch 6th bit
//		array[5] = 1;
//		}
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {   	// PB1 -> DIP Switch 5th bit
//		array[4] = 1;
//	}
//	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) {	// PC14 -> DIP switch 4th bit
//		array[3] = 1;
//	}
//	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET) {		// PC15 -> DIP Switch third bit
//		array[2] = 1;
//	}
//	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) {		// PA8 -> DIP Switch second bit
//		array[1] = 1;
//	}
//	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {		// PA7 -> DIP Switch first bit
//		array[0] = 1;
//	}


    uint8_t result = ((array[0] << 7) | (array[1] << 6) | (array[2] << 5) | (array[3] << 4) |
    		(array[4] << 3) | (array[5] << 2) | (array[6] << 1) | (array[7]));


    return result + 1;
}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &UartHandleX){
		HAL_UART_Receive_IT(UartHandle, &rec_char[0], 1);
		HAL_UART_Transmit(&UartHandleX, &rec_char[0], 1, 0x0FFFF);
	} else {
		upLinkComms_receiveCallback(UartHandle);
	}


}
/**
 * simple print function, string must be end by 0
 * to save program space
 */
void print(const char * str)
{
  char * ptr = (char*)str;
  uint16_t len = 0;
  while(*ptr != '\0')
  {
  ptr++;
  len++;
  }
  HAL_UART_Transmit(&UartHandleX, (uint8_t *)str, len, 0x0FFFF);
}
/**
 * Print a number (radix 10)
 */
void print_number(int number, const char * fmt)
{
  char str[20];
  sprintf(str, fmt, number);   //make the number into string using sprintf function
  print(str);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};



  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while (1)
  {
  }
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
