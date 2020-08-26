/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_envision_proximity.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


/* define USART2 for communication with Serial Monitor */

#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_15
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF3_USART2
/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Size of Reception buffer */
#define RXBUFFERSIZE                      32


/* define USART1 for communication with RS485 */

#define UPLINK_USART_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
//#define UPLINK_USART_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
//#define UPLINK_USART_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define UPLINK_USART_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define UPLINK_USART_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define UPLINK_USART_TX_PIN                    GPIO_PIN_9
#define UPLINK_USART_TX_GPIO_PORT              GPIOA
#define UPLINK_USART_TX_AF                     GPIO_AF7_USART1
#define UPLINK_USART_RX_PIN                    GPIO_PIN_10
#define UPLINK_USART_RX_GPIO_PORT              GPIOA
//#define UPLINK_USART_RX_AF                     GPIO_AF7_USART1
/* Definition for USARTx's NVIC */
#define UPLINK_USART_IRQn                      USART1_IRQn
#define UPLINK_USART_IRQHandler                USART1_IRQHandler

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Function to print a string into your Serial Monitor via USART
  *
  * @param  str: The string that you would like to print
  *
  * @note	don't forget to first configure the UART peripheral; we use USART2 for
  * 		communication with serial monitor
  *
  * @retval None
  */
void print(const char * str);

/**
  * @brief  Print a number (radix 10)
  *
  * @param  number: Value that you would like to print
  * 		fmt: Format of the number (i.e. "%d" for integers)
  *
  * @retval None
  */
void print_number(int number, const char * fmt);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
