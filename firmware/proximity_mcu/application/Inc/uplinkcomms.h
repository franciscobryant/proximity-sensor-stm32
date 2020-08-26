/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UPLINKCOMMS_H
#define __UPLINKCOMMS_H


#include "stm32l4xx_hal.h"
#include "stm32l4xx_envision_proximity.h"


UART_HandleTypeDef UplinkUartHandle;	// We will be using USART1 for the UplinkUart


/* Exported Functions ------------------------------------------------------------*/

/**
  * @brief  Configure the peripherals needed for uplink communications
  *
  * @param  None
  *
  * @retval None
  */
void uplinkComms_init();

/**
  * @brief  Whenever a data is received from the UplinkUart, this function gets called to store the data,
  * 		and after it receives 1 complete frame, it will call the upLinkComms_regularProcessing function
  *
  * @param  UartHandle: UART handle
  *
  * @note	don't forget to first configure the UART peripheral first, using uplinkComms_init()
  *
  * @retval None
  */
void upLinkComms_receiveCallback(UART_HandleTypeDef *UartHandle);

/**
  * @brief  After being called by the upLinkComms_receiveCallback, this function will send back a reply data
  * 		through the UplinkUart consisting of it's modbus slave ID and also the distance data
  *
  * @param  slaveID: Your modbus slave ID
  * 		distanceFiltered: The data you want to send, in this case is the range data we get from
  * 						  the VL53L0X sensor
  *
  * @retval None
  */
void upLinkComms_regularProcessing(uint8_t slaveID, uint16_t distanceFiltered);


#endif /* __UPLINKCOMMS_H */
