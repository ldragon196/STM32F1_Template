/*
 * Fifo.h
 *
*  Created on: August 13, 2020
 *      Author: LongHD
 */

#ifndef SOURCE_UTILITY_FIFO_FIFO_H_
#define SOURCE_UTILITY_FIFO_FIFO_H_

/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(FIFO_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef struct{
	uint8_t length;
	uint8_t data[FIFO_UNIT_SIZE];
} fifo_unit_t;

typedef struct{
	uint8_t head;
	uint8_t tail;
	uint8_t count;
	uint8_t size;
} fifo_sequence_t;

typedef struct{
	fifo_sequence_t sequence;
	fifo_unit_t* units;
} fifo_entry_t;

typedef uint8_t fifo_error;
enum {
	FIFO_OK = 0,
	FIFO_IS_FULL,
	FIFO_IS_EMPTY,
};

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void FIFO_Init(fifo_entry_t* fifo, fifo_unit_t* store_unit, uint8_t number_unit);
fifo_error FIFO_Push(fifo_entry_t* fifo, fifo_unit_t src_unit);
fifo_error FIFO_Pop(fifo_entry_t* fifo, fifo_unit_t* des_unit);

/******************************************************************************/
#endif

#endif /* SOURCE_UTILITY_FIFO_FIFO_H_ */
