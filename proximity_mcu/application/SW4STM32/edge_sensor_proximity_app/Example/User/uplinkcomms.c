#include "uplinkcomms.h"
#include <stdbool.h>
#include <stdio.h>

#define RS485_RTS_PIN GPIO_PIN_12

#define UPLINK_USART USART1
#define RECV_BUFF_SIZE 10
#define TRAN_BUFF_SIZE 20


uint8_t currentCharacter;
uint8_t currentRecvBufferIndex = 0;
uint8_t recvBuffer[RECV_BUFF_SIZE];
bool receivedOneFrame = false;
bool startToReceiveFrame = false;

#define FRAME_START ':'
#define FRAME_END '\n'


// Receive Format
// FRAME_START, SLAVE_ID_byte, FRAME_END
// 58 98 10

// Send Format
// FRAME_START, SLAVE_ID_byte, Distance in Milimeters as text string, FRAME_END


static void Error_Handler(void);
static void processRequestAndReplyIfNeeded(uint8_t slaveID, uint16_t distanceFiltered);

void uplinkComms_init(){

	  GPIO_InitTypeDef  GPIO_InitStruct;

	  /* Configure the GPIO_LED pin */
	  GPIO_InitStruct.Pin = RS485_RTS_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;


	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  HAL_GPIO_WritePin(GPIOA, RS485_RTS_PIN, GPIO_PIN_RESET);


	  /*##-1- Configure the UART peripheral ######################################*/
	    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	    /* UART configured as follows:
	        - Word Length = 8 Bits (7 data bit + 1 parity bit) :
	                      BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
	        - Stop Bit    = One Stop bit
	        - Parity      = No parity
	        - BaudRate    = 115200 baud
	        - Hardware flow control disabled (RTS and CTS signals) */
		UplinkUartHandle.Instance        = UPLINK_USART;

		UplinkUartHandle.Init.BaudRate   = 115200;
		UplinkUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
		UplinkUartHandle.Init.StopBits   = UART_STOPBITS_1;
		UplinkUartHandle.Init.Parity     = UART_PARITY_NONE;
		UplinkUartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE; //Manual control for RS485 pin
		UplinkUartHandle.Init.Mode       = UART_MODE_TX_RX;

	    if (HAL_UART_Init(&UplinkUartHandle) != HAL_OK)
	    {
	      /* Initialization Error */
	      Error_Handler();
	    }

	    /*##-4- Put UART peripheral in reception process ###########################*/
	    if(HAL_UART_Receive_IT(&UplinkUartHandle, &currentCharacter, 1) != HAL_OK)
	    {
	    	Error_Handler();
	    }

}

void upLinkComms_receiveCallback(UART_HandleTypeDef *UartHandle){
	if(UartHandle != &UplinkUartHandle){
		return;
	}

	HAL_UART_Receive_IT(&UplinkUartHandle, &currentCharacter, 1);
	//HAL_UART_Transmit(&UplinkUartHandle, &currentCharacter, 1, 0x0FFFF);

	//If flag is not toggled back to false, means the frame is still processing, we should not care about new data
	if(receivedOneFrame){
		return;
	}

	if(currentCharacter == FRAME_START){
		BSP_LED_On(LED3);
		startToReceiveFrame = true;
		currentRecvBufferIndex = 0;
		return;
	}

	if(currentCharacter == FRAME_END){
		receivedOneFrame = true;
		startToReceiveFrame = false;
		return;
	}


	//Reach the end, roll back to start
	if(currentRecvBufferIndex >= RECV_BUFF_SIZE){
		currentRecvBufferIndex = 0;
	}

	recvBuffer[currentRecvBufferIndex] = currentCharacter;
	currentRecvBufferIndex++;



}

void upLinkComms_regularProcessing(uint8_t slaveID, uint16_t distanceFiltered){


	if(receivedOneFrame){
		processRequestAndReplyIfNeeded(slaveID, distanceFiltered);
		currentRecvBufferIndex = 0;
		receivedOneFrame = false;
		BSP_LED_Off(LED3);
	}
}


static void processRequestAndReplyIfNeeded(uint8_t slaveID, uint16_t distanceFiltered){


	if(currentRecvBufferIndex == 0){
		return;
	}

	int slaveIDOfReceivedPacket = recvBuffer[0];

	// Frame not meant for me
	if(slaveIDOfReceivedPacket != slaveID){
		return;
	}

	char reply[TRAN_BUFF_SIZE];

	if (VL53L0X_timeoutOccurred()) {
		print("Sensor TIMEOUT \r\n");
		return;
	}


	uint8_t upper_byte = (distanceFiltered >> 8) & 0xFF;
	uint8_t lower_byte = distanceFiltered & 0xFF;

	int replyLength = snprintf(reply, TRAN_BUFF_SIZE, "%c%c%c%c%c", FRAME_START, slaveID, upper_byte, lower_byte, FRAME_END);

	//int replyLength = snprintf(reply, TRAN_BUFF_SIZE, "%c%c%d%c", FRAME_START, slaveID, distance, FRAME_END);

	HAL_GPIO_WritePin(GPIOA, RS485_RTS_PIN, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_UART_Transmit(&UplinkUartHandle, (uint8_t*) reply, replyLength, 0x0FFFF);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA, RS485_RTS_PIN, GPIO_PIN_RESET);

}


static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while (1)
  {
  }
}

