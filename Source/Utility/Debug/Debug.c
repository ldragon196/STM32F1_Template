/*
 *  Debug.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Debug.h"

#if(DEBUG_ENABLED)
#include "Hard/Uart/Uart.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

struct __FILE{
	int handle;
	/* Whatever you require here. If the only file you are using is */
	/* standard output using printf() for debugging, no file handling */
	/* is required. */
};

/* FILE is typedef’d in stdio.h. */
FILE __stdout;

int fputc(int ch, FILE *f){
	uint8_t send = (uint8_t) ch;
	UART_WriteByte(DEBUG_UART_COM_CONFIG, send);
	return ch;
}

int ferror(FILE *f){
	/* Your implementation of ferror(). */
	return 0;
}

void DEBUG_PrintHex(uint8_t *data, uint8_t size){
	printf("Hex string: ");
	for(uint8_t i = 0; i < size; i++){
		printf("%02X ", data[i]);
	}
	
	printf("\r\n");
}

#endif
