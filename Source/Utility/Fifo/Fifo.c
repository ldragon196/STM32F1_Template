/*
 * fifo.c
 *
*  Created on: August 13, 2020
 *      Author: LongHD
 */

/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Fifo.h"

#if(FIFO_ENABLED)

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

static uint8_t FIFO_IsFull(fifo_entry_t* fifo);
static uint8_t FIFO_IsEmpty(fifo_entry_t* fifo);

/******************************************************************************/

/**
 * @function :  FIFO_Init
 * @brief    :  Initialized FIFO
 * @parameter:  fifo: pointer to fifo entry
 *              store_unit: pointer to memory store all fifo unit
 *              number_unit: max unit in fifo
 * @retVal   :  None
 */

void FIFO_Init(fifo_entry_t* fifo, fifo_unit_t* store_unit, uint8_t number_unit){
	uint8_t* init = (uint8_t*) store_unit;
	uint16_t size_of_fifo = sizeof(fifo_unit_t) * number_unit;

	// Set all to zero
	for(uint16_t i = 0; i < size_of_fifo; i++){
		*init++ = 0;
	}
	fifo->sequence.head = 0;
	fifo->sequence.tail = 0;
	fifo->sequence.count = 0;
	fifo->sequence.size = number_unit;
	fifo->units = store_unit;
}

/**
 * @function :  FIFO_IsFull
 * @brief    :  Check fifo full or not
 * @parameter:  fifo: pointer to fifo entry which checked
 * @retVal   :  True if fifo is full. Otherwise return false
 */

static uint8_t FIFO_IsFull(fifo_entry_t* fifo){
	if(fifo->sequence.count != fifo->sequence.size){
		return 0;
	}
	return 1;
}

/**
 * @function :  FIFO_IsEmpty
 * @brief    :  Check fifo full or not
 * @parameter:  fifo: pointer to fifo entry which checked
 * @retVal   :  True if fifo is empty. Otherwise return false
 */

static uint8_t FIFO_IsEmpty(fifo_entry_t* fifo){
	if(fifo->sequence.count != 0){
		return 0;
	}
	return 1;
}

/**
 * @function :  FIFO_Push
 * @brief    :  Push unit in fifo
 * @parameter:  fifo: pointer to fifo entry
 *              src_unit: unit push
 * @retVal   :  fifo_error
 */

fifo_error FIFO_Push(fifo_entry_t* fifo, fifo_unit_t src_unit){
	fifo_unit_t* unit_fifo = &fifo->units[fifo->sequence.head];

	// Check fifo full. If not, push in. Else return error
	if(!FIFO_IsFull(fifo)){
		unit_fifo->length = src_unit.length;
		memcpy(unit_fifo->data, src_unit.data, src_unit.length);
		fifo->sequence.count++;
		fifo->sequence.head = (fifo->sequence.head + 1) % fifo->sequence.size;

		return FIFO_OK;
	}
	return FIFO_IS_FULL;
}

/**
 * @function :  FIFO_Pop
 * @brief    :  Pop out fifo unit
 * @parameter:  fifo: pointer to fifo entry
 *              des_unit: pointer store unit pop
 * @retVal   :  fifo_error
 */

fifo_error FIFO_Pop(fifo_entry_t* fifo, fifo_unit_t* des_unit){
	fifo_unit_t* unit_fifo = &fifo->units[fifo->sequence.tail];

	// Check fifo empty. If not, pop out. Else return error
	if(!FIFO_IsEmpty(fifo)){
		des_unit->length = unit_fifo->length;
		memcpy(des_unit->data, unit_fifo->data, unit_fifo->length);
		fifo->sequence.count--;
		fifo->sequence.tail = (fifo->sequence.tail + 1) % fifo->sequence.size;

		return FIFO_OK;
	}
	return FIFO_IS_EMPTY;
}

/********************************* EXAMPLE ************************************/

/*
#define NUMBER_UNIT                         4
fifo_entry_t testFifoEntry;                   // fifo entry
fifo_unit_t testFifoUnit[NUMBER_UNIT];        // store data

void test_init(void){
	FIFO_Init(&testFifoEntry, testFifoUnit, NUMBER_UNIT);
	
	fifo_unit_t source = {
		.data = {1,2,3},
		.length = 3
	};
	fifo_unit_t dest = {0};
	
	FIFO_Push(&testFifoEntry, source);
	FIFO_Pop(&testFifoEntry, &dest);
}
*/

/******************************************************************************/
#endif

