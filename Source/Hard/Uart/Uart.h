/*
 * Uart.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_UART_UART_H_
#define _SOURCE_HARD_UART_UART_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(UART_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define UART_ID_1                      0
#define UART_ID_2                      1
#define UART_ID_3                      2

enum _STM32F103_UART_{
	#if(UART1_ENABLED)
	UART_COUNT_1,
	#endif
	#if(UART2_ENABLED)
	UART_COUNT_2,
	#endif
	#if(UART3_ENABLED)
	UART_COUNT_3,
	#endif
	UART_COUNT
};

typedef struct{
	uint8_t data[UART_DEFAULT_CONFIG_RX_FIFO_SIZE];  // store data
	uint16_t head;                                   // index to write data
	uint16_t tail;                                   // index to read data
	uint16_t count;                                  // number of bytes in buffer
} UartRxFifo_t;



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void UART_Init(uint8_t uartNum);
void UART_WriteByte(uint8_t uartNum, uint8_t byte);
void UART_WriteData(uint8_t uartNum, const uint8_t* data, uint16_t length);
uint16_t UART_ReadAvailableByte(uint8_t uartNum);
uint8_t UART_ReadByte(uint8_t uartNum, uint8_t* byte);
void UART_Flush(uint8_t uartNum);

/******************************************************************************/
#endif

#endif /* _SOURCE_HARD_GPIO_GPIO_H_ */
