#ifndef __UART_FUNC_H__
#define __UART_FUNC_H__

#define UART_NUM_UART1 1
#define UART_NUM_UART2 2
#define UART_NUM_UART3 3

#define TXPIN_UART1               GPIO_PIN_9
#define RXPIN_UART1               GPIO_PIN_10

#define TXPIN_UART2 			  GPIO_PIN_2
#define RXPIN_UART2 			  GPIO_PIN_3	

#define TXPIN_UART3 			  GPIO_PIN_10
#define RXPIN_UART3 			  GPIO_PIN_11

UART_HandleTypeDef openUart (uint8_t numUart, uint16_t baudRate);
void SystemClock_Config(void);

#endif
