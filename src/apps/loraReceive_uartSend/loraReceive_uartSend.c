/*

LCIS, 2016

Description: Reception de trame LoRa et affichage sur UART1 (toggle Led)
Auteurs : André LAGREZE, Youness LAMI
Date : 21/09/2016

*/

#include <string.h>
#include "board.h"
#include "radio.h"
#include <uart_func.h>


#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;


#define RX_TIMEOUT_VALUE                            4000    
#define BUFFER_SIZE                                 64 // Define the payload size here



#define FIFO_TX_SIZE                                128
#define FIFO_RX_SIZE                                128
uint8_t TxBuf[FIFO_TX_SIZE];
uint8_t RxBuf[FIFO_RX_SIZE];

UART_HandleTypeDef huart;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

void _send_byte(uint8_t byte);
void printFloat(double number);
void printString(char *data);

void printNumberLn(int32_t number, uint8_t base);
void printUnsignedNumber(uint32_t n, uint8_t base);
void printStringLn(char *data);
void printLn(void);
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

	
const uint8_t entete[] = "LCIS";

/**
 * Main application entry point.
 */
int main( void )
{
		//uint8_t i;
		char chaine[100];
		uint16_t pressure;
		uint16_t temperature; 
		uint16_t altitude;
	
		UART_HandleTypeDef huart;
    // Target board initialisation
    BoardInitMcu( );
    BoardInitPeriph( );
	
		HAL_Init();
	
		SystemClock_Config();
	
		huart = openUart (UART_NUM_UART1, 9600);
	
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

  Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );
    
    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif
    
		HAL_UART_Transmit(&huart, "Debut test \n\r", 13, 0xFFFF);
		Radio.Rx( 0 );
																	
		while(1)
		{			
			///while(State != RX && State != RX_ERROR && State != RX_TIMEOUT)
			//{
			//	TimerLowPowerHandler( );	 
			//}
			
			switch (State)
			{
				case RX :
					memcpy(chaine, Buffer, 4);
					chaine[4] = 0;
			
          if( strcmp(chaine, entete)== 0 )
					{
				  pressure = 0;
					temperature = 0; 
					altitude = 0;

					pressure = (uint16_t) (Buffer[4] << 8) + Buffer[5];
					temperature = (uint16_t) (Buffer[6] << 8) + Buffer[7];
					altitude = (uint16_t) (Buffer[8] << 8) + Buffer[9];
						
				  sprintf(chaine, "pression = %d, temperature = %4.1f, altitude = %5.1f", pressure, (float) temperature/100.0, (float)(altitude + 400.0)/10);
					HAL_UART_Transmit(&huart, chaine, strlen(chaine), 0xFFFF);
					HAL_UART_Transmit(&huart, "\r\n", 2, 0xFFFF);
					State = LOWPOWER;
					}
					else {}
				break;
				
				case RX_TIMEOUT: 
					HAL_UART_Transmit(&huart, "TO\n\r",4,0xFFFF);
					State = LOWPOWER;
					break;
				case RX_ERROR: 
					HAL_UART_Transmit(&huart, "RXE\n\r",5,0xFFFF);
					State = LOWPOWER;
					break;
			}
				TimerLowPowerHandler( );	 
	  }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
		HAL_Delay(10);
		GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
 //   Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( ); // clear buffer
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
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
