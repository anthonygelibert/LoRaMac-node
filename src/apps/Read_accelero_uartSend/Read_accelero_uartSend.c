/* 
	 Récupération données acceléro et affichage sur UART
  
   Auteurs : André LAGREZE, Youness LAMI 
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "board.h"
#include "stm32l1xx_hal_uart.h"
#include <uart_func.h>
#include <mma8451.h>
#include <mpl3115.h>

UART_HandleTypeDef huart;

void _send_byte(uint8_t byte);
void printFloat(double number);
void printString(char *data);

void printNumberLn(int32_t number, uint8_t base);
void printUnsignedNumber(uint32_t n, uint8_t base);
void printStringLn(char *data);
void printLn(void);
	
int main(void)
{    
	uint16_t pressure = 0;
  int16_t altitudeBar = 0;
  int16_t temperature = 0;
	//uint8_t Buffer[6];
	
	BoardInitMcu( );
	//BoardInitPeriph( );
	//HAL_Init();
	
	MPL3115Init();
	
	while(1)
	{
	pressure = ( uint16_t )( MPL3115ReadPressure( )/100);             // in hPa / 10
	temperature = ( int16_t )( MPL3115ReadTemperature());       // in °C * 100
	altitudeBar = ( int16_t )( MPL3115ReadAltitude( ));           // in m * 10
	
//	Buffer[0] = ( pressure >> 8 ) & 0xFF;
//  Buffer[1] = pressure & 0xFF;
//  Buffer[2] = ( temperature >> 8 ) & 0xFF;
//  Buffer[3] = temperature & 0xFF;
//  Buffer[4] = ( altitudeBar >> 8 ) & 0xFF;
//  Buffer[5] = altitudeBar & 0xFF;
	
	//HAL_Init();
	SystemClock_Config();
	huart = openUart(UART_NUM_UART1,9600);

	printString("Donnees capteurs:\n\r");
	printLn();
	printString("Temperature (en degres) : ");
	printNumberLn(temperature, 10);
	printString("Pression    (en hPa)    : ");
	printNumberLn(pressure, 10);
	printString("Altitude    (en m)      : ");
	printNumberLn(altitudeBar, 10);
	HAL_Delay(1000);
	printLn();
	}
}

void _send_byte(uint8_t byte){
	HAL_UART_Transmit(&huart, &byte, 1, 0xffff);
}

void printString(char *data){
	uint16_t i;
	uint16_t str_length = strlen(data);
	for(i = 0; i < str_length; i++)
	{
		_send_byte(data[i]);
	}
}

void printFloat(double number){
	char float_as_string[20];
	sprintf(float_as_string, "%f", number);
  printString(float_as_string);
}

void printUnsignedNumber(uint32_t n, uint8_t base){
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];
	unsigned long m;
	char c;
  *str = '\0';

  //prevent crash if called with base == 1
  if (base < 2) base = 10;

  do 
	{
    m = n;
    n /= base;
    c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while(n);

	printString(str); 
}


void printLn(){
	printString("\n\r");
}

void printNumberLn(int32_t number, uint8_t base){
	if (number < 0) 
	{
		printString("-");
		number = -number;
		printUnsignedNumber(number, base);
	}
	else 
	{
		printUnsignedNumber(number, base);
	}
	printLn();
}

void printStringLn(char *data){
	uint16_t i;
	uint16_t str_length = strlen(data);
	for(i = 0; i < str_length; i++)
	{
		_send_byte(data[i]);
	}
	printLn();
}
