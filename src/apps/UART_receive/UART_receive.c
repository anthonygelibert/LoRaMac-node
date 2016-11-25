#include <string.h>
#include <math.h>
#include <stdio.h>
#include "board.h"
#include <uart_func.h>
#include "stm32l1xx_hal_uart.h"


#define BAUDRATE              9600
#define TXPIN                 GPIO_PIN_9
#define RXPIN                 GPIO_PIN_10
#define DATAPORT              GPIOA
 
int main(void)
{    
	 char rec[50];
	
	HAL_Init();
	
	SystemClock_Config();
	
	__GPIOA_CLK_ENABLE();
	
  GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = TXPIN | RXPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(DATAPORT, &GPIO_InitStruct);
	
	__USART1_CLK_ENABLE();
    
  UART_HandleTypeDef huart1;

	huart1.Instance = USART1;
	huart1.Init.BaudRate = BAUDRATE;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&huart1);
 
  while(1)
	{
		if(HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
		{
		HAL_UART_Receive(&huart1, (uint8_t *)rec, sizeof(rec), 1000);
		}
		HAL_UART_Transmit(&huart1, (uint8_t*)rec, strlen(rec), 1000);
	//	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	}
	}
