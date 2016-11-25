/* 
   Bibliothéque maison relative à l'uart_func
   Date : 21/09/2016
   Auteurs : André LAGREZE, Youness LAMI
   Work in progress
   
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "board.h"
#include "stm32l1xx_hal_uart.h"
#include "uart_func.h"

UART_HandleTypeDef openUart (uint8_t numUart, uint16_t baudRate)
	{
	UART_HandleTypeDef currentHuart;
	GPIO_InitTypeDef currentGpio;
	GPIO_TypeDef* currentDataport;
	switch (numUart)
	{
		case  UART_NUM_UART1:
			__GPIOA_CLK_ENABLE();
			currentGpio.Pin = TXPIN_UART1 | RXPIN_UART1;
			currentGpio.Alternate = GPIO_AF7_USART1;
			currentDataport = GPIOA;
			__USART1_CLK_ENABLE();
			currentHuart.Instance = USART1;	
			break;
		case  UART_NUM_UART2:
			__GPIOA_CLK_ENABLE();
			currentGpio.Pin = TXPIN_UART2 | RXPIN_UART2;
			currentGpio.Alternate = GPIO_AF7_USART2;
			currentDataport = GPIOA;
			__USART2_CLK_ENABLE();
			currentHuart.Instance = USART2;	
		case  UART_NUM_UART3:
			__GPIOB_CLK_ENABLE();
			currentGpio.Pin = TXPIN_UART3 | RXPIN_UART3;
			currentGpio.Alternate = GPIO_AF7_USART3;
			currentDataport = GPIOB;
			__USART3_CLK_ENABLE();
			currentHuart.Instance = USART3;	
	}
	
	currentGpio.Mode = GPIO_MODE_AF_PP;
	currentGpio.Pull = GPIO_NOPULL;
	currentGpio.Speed = GPIO_SPEED_LOW;
	
	HAL_GPIO_Init(currentDataport, &currentGpio);
		
	currentHuart.Init.BaudRate = baudRate;
	currentHuart.Init.WordLength = UART_WORDLENGTH_8B;
	currentHuart.Init.StopBits = UART_STOPBITS_1;
	currentHuart.Init.Parity = UART_PARITY_NONE;
	currentHuart.Init.Mode = UART_MODE_TX_RX;
	currentHuart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	currentHuart.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&currentHuart);
	return currentHuart;
	
	}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

