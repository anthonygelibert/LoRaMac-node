/* 
	 Emission continue avec nouvelle bibliothéque maison
   Date : 21/09/2016
   Auteurs : André LAGREZE, Youness LAMI 
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "board.h"
#include "stm32l1xx_hal_uart.h"
#include <uart_func.h>

int main(void)
{    
	char *msg = "abcd\n\r";
	UART_HandleTypeDef huart;
	BoardInitMcu( );
	BoardInitPeriph( );
	HAL_Init();
	
	SystemClock_Config();
	
	huart = openUart (UART_NUM_UART1, 9600);
 
  while(1)
	{
		HAL_Delay(1000);  // 1s de délai
		GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
		HAL_UART_Transmit(&huart, (uint8_t*)msg, strlen(msg), 0xFFFF);
		GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
	}
}
