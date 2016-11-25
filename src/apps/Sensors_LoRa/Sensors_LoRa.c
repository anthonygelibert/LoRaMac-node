/* 

	 Récupération données accéleromètre et autres capteurs
	 Emission continu des données capteurs 
	 Respect format Thalés
	 
		4 octets adresse du capteur, valeur arbitraire fixe codée en big endian, par exemple 0x1234. 
		2 octets numéro de message, valeur évolutive incrémentée à chaque message 
		2 octets température, 12 bits utiles 
		2 octets accélération X, 12 bits utiles 
		2 octets accélération Y, 12 bits utiles 
		2 octets accélération Z, 12 bits utiles 
		1 octet alarmes. Bit 0 = température dépassée, bit 1 = axe X, bit 2 axe Y, bit 3 axe Z. ces bits sont à 1 si alarme active 

   Auteurs : André LAGREZE, Youness LAMI, Grégory NOCERA
	 Date : 22/11/2016
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "board.h"
#include "radio.h"
#include "stm32l1xx_hal_uart.h"

#include <mma8451.h>
#include <mpl3115.h>
#include <uart-board.h>

// Defines for MMA8451Q
#define REG_STATUS 0x00
#define REG_XYZ_DATA_CFG 0x0E
#define REG_WHO_AM_I 0x0D
#define REG_CTRL_REG1 0x2A //ODR = 800 Hz, standby mode
#define REG_CTRL_REG2 0x2B //Sleep enable, OS modes, RST, ST
#define REG_CTRL_REG3 0x2C //Wake from sleep, IPOL, PP_OD
#define REG_CTRL_REG4 0x2D //Interrupt enable register
#define REG_CTRL_REG5 0x2E //Interrupt pin (INT1/INT2) map
#define REG_OUT_X_MSB 0x01 
#define REG_OUT_X_LSB 0x02
#define REG_OUT_Y_MSB 0x03 
#define REG_OUT_Y_LSB 0x04
#define REG_OUT_Z_MSB 0x05 
#define REG_OUT_Z_LSB 0x06

#define ALARME_TEMP  0x18
#define ALARME_AXE_X 2
#define ALARME_AXE_Y 4
#define ALARME_AXE_Z 8

// Defines for LoRa parameters

#define RF_FREQUENCY                                868100000 // Hz
#define TX_OUTPUT_POWER                             10        // dBm
#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

typedef struct { 
			uint8_t X_MSB;
			uint8_t X_LSB;
			uint8_t Y_MSB;
			uint8_t Y_LSB;
			uint8_t Z_MSB;
			uint8_t Z_LSB;				
		}  RegAcc_t;
		
typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

#define FIFO_TX_SIZE                                128
#define FIFO_RX_SIZE                                128
#define SENSOR_ADR 																	0X1234

uint8_t TxBuf[FIFO_TX_SIZE];
uint8_t RxBuf[FIFO_RX_SIZE];

int16_t dataAcc[3]; //Données de l'accelero
uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;


int8_t RssiValue = 0;
int8_t SnrValue = 0;


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
void ConfigMMA8451Q( void );
void ReadAllAxisMMA8451Q(void);
uint16_t buildFrameBuffer( uint32_t sensorAdr,uint16_t numFrame,uint16_t dataAcc[],
         uint16_t temp,uint8_t alarm);

void printLoRaConfig (uint32_t RF_freq, uint8_t SF, uint8_t TxPow, uint16_t Bandwidth, uint8_t ERCa, uint8_t ERCb );
void printSensorsData (uint16_t temperature, uint16_t pressure, uint16_t altitude , bool acc, bool othersensors );
int16_t getSingleAxeAcc(uint8_t accLSB, uint8_t accMSB);

int main(void)
{
	uint16_t pressure = 0;
  int16_t altitudeBar = 0;
  int16_t temperature = 0;
	uint8_t alarm = ALARME_TEMP;
	int16_t numFrame = 0;
	
	char adrCapteur[] = "0128";
	int i;
    // Target board initialisation
    BoardInitMcu( );
    BoardInitPeriph( );
		
		FifoInit( &Uart1.FifoTx, TxBuf, FIFO_TX_SIZE );
    FifoInit( &Uart1.FifoRx, RxBuf, FIFO_RX_SIZE );
		UartInit(&Uart1, UART_1, UART_TX, UART_RX);
		UartConfig(&Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
	
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
																	
   	BufferSize = 15;						// 7 est la taille minimale pouvant etre envoyé
		MPL3115Init();
		MMA8451Init();	
		uint8_t val;	
		DelayMs(1); 										//GREG v2												
		MMA8451Read( 0x0d,&val );																								
		ConfigMMA8451Q();
																															
		printLoRaConfig (RF_FREQUENCY,LORA_SPREADING_FACTOR, TX_OUTPUT_POWER ,LORA_BANDWIDTH,4 ,5);					
																			
		while( 1 )
		{
			
			pressure = ( uint16_t )( MPL3115ReadPressure( )/100);             // in hPa / 10
			temperature = ( int16_t )( MPL3115ReadTemperature()*100);       // in °C * 100
			altitudeBar = ( int16_t )( MPL3115ReadAltitude( )*10);           // in m * 10
			ReadAllAxisMMA8451Q();	
			
  		strcpy((char *)Buffer, adrCapteur); 
//			Buffer[4] = 0xFF;//( pressure >> 8 ) & 0xFF;
//			Buffer[5] = pressure & 0xFF;
			
			Buffer[4] = (numFrame >> 8) & 0xff;
			Buffer[5] = numFrame++ & 0xFF;
			
			Buffer[6] = ( temperature >> 8 ) & 0xFF;
			Buffer[7] = temperature & 0xFF;
			
			Buffer[8] = (dataAcc[0] >> 8) & 0xFF;
			Buffer[9] = dataAcc[0] & 0xFF;
			Buffer[10] = (dataAcc[1] >> 8) & 0xFF;
			Buffer[11] = dataAcc[1] & 0xFF;
			Buffer[12] =(dataAcc[2] >> 8) & 0xFF;
			Buffer[13] = dataAcc[2] & 0xFF;
			Buffer[13] = dataAcc[2] & 0xFF;
			Buffer[14] = alarm;
			
//			Buffer[8] = ( altitudeBar >> 8 ) & 0xFF;
//			Buffer[9] = altitudeBar & 0xFF;
			
			DelayMs( 1 ); 
			GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
      Radio.Send( Buffer, BufferSize );
			
			printf("Data sent by LoRa\r\n\r\n");
			for (i=0; i< BufferSize; i++)
			{
				printf(" %x ",Buffer[i]);
			}
			printf("\n\r\n\r");
			printSensorsData ( temperature,  pressure,  altitudeBar , true , true );
			GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
		//	Radio.Rx(0);
			Delay(120);
			TimerLowPowerHandler();
		}														
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	int i;
	
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
			printf("Data received by LoRa\r\n\r\n");
			for (i=0; i< size; i++)
			{
				printf(" %x ",payload[i]);
			}
						printf("\n\r\n\r");

			printf("RSSI : %d\r\n",rssi);
			printf("SNR : %d\r\n",snr);
			
			
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
	printf("RX Timeout\r\n");

}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
	printf("RX Error\r\n");
	
}

void ConfigMMA8451Q( void )
{
    MMA8451Write(REG_CTRL_REG1, 0x00);// Puts acc in standby for configuring
		MMA8451Write(REG_XYZ_DATA_CFG,0x00);// Writing 00 turns off high-pass filter and sets full scale range to 2g
    // data[1] = 0x01; for 4g
    // data[1] = 0x02; for 8g
		MMA8451Write(REG_CTRL_REG2,0x00);// Disable self-test, software reset and auto-sleep; operates in normal mode
		MMA8451Write(REG_CTRL_REG3,0x00);// Interrupt polarity low, push-pull output
		MMA8451Write(REG_CTRL_REG4,0x01);// Enables interrupt for data Ready
		MMA8451Write(REG_CTRL_REG5,0x01);// Routes Data Ready interrupt to INT1
		MMA8451Write(REG_CTRL_REG1,0x09);// Data rate is 800Hz
}

void ReadAllAxisMMA8451Q()
{
		RegAcc_t regs;
		
    MMA8451ReadBuffer(REG_OUT_X_MSB, (uint8_t *)&regs, 6);

		dataAcc[0] = getSingleAxeAcc(regs.X_LSB, regs.X_MSB);
		dataAcc[1] = getSingleAxeAcc(regs.Y_LSB, regs.Y_MSB);
		dataAcc[2] = getSingleAxeAcc(regs.Z_LSB, regs.Z_MSB);
}

void printLoRaConfig (uint32_t RF_freq, uint8_t SF, uint8_t TxPow, uint16_t Bandwidth, uint8_t ERCa, uint8_t ERCb )  // A améliorer
{
	switch( Bandwidth )
        {
        case 0: Bandwidth = 125; break;
				case 1: Bandwidth = 250; break;
				case 2: Bandwidth = 500; break;
				}
	printf("LoRa module configuration\r\n\r\n Radio frequency : %dHz \n\r Spreading Factor: SF%d \n\r Transmit power  : %ddBm \n\r Bandwidth       : %dKHz \n\r Error Coding    : %d/%d\n\r\n\r", RF_freq, SF, TxPow, Bandwidth, ERCa, ERCb);
}

void printSensorsData (uint16_t temperature, uint16_t pressure, uint16_t altitude , bool acc, bool othersensors )
{
	
	if ( acc && othersensors)
	{
		printf("Sensors data : \r\n\r\n");
		printf(" Accelerometer data : \r\n\r\n AccX: %.3f \r\n AccY: %.3f \r\n AccZ: %.3f \r\n\r\n Other sensors data : \n\r Pressure = %d hPa, temperature = %4.1f °, altitude = %5.1f m \r\n\r\n",
		(float)dataAcc[0]/4096,(float)dataAcc[1]/4096,(float)dataAcc[2]/4096,pressure, (float) temperature/100.0, (float)(altitude + 400.0)/10);
	}
	else if (othersensors)
	{
		printf("Sensors data : \r\n\r\n");
		printf(" Pressure = %d hPa, temperature = %4.1f °, altitude = %5.1f m \r\n\r\n", pressure, (float) temperature/100.0, (float)(altitude + 400.0)/10);
	}
	else if (acc)
	{
		printf("Sensors data : \r\n\r\n");
		printf(" Accelerometer data : \r\n\r\n AccX: %.3f \r\n AccY: %.3f \r\n AccZ: %.3f \r\n\r\n",(float)(dataAcc[0]/4096),(float)(dataAcc[1]/4096),(float)(dataAcc[2]/4096));
	//	printf(" Accelerometer data : \r\n\r\n AccX: %x \r\n AccY: %x \r\n AccZ: %x \r\n\r\n",(signed)dataAcc[0],(signed)dataAcc[1],(signed)dataAcc[2]);
	}
}

int16_t getSingleAxeAcc(uint8_t accLSB, uint8_t accMSB)
{
	int16_t res = 0;
	res = accMSB;
	res = (res << 6) + (accLSB >> 2); // les poids faibles et forts sont cadrés à gauche 
	
	if (res & 0x2000)       // Si le bit de signe est positionné , on le propage
		res = res | 0xc000;   
	return res;
}

