/*
 * Memory.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Hard/Flash/Flash.h"
#include "Memory.h"

#if(MEMORY_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static uint32_t addressStart = 0;      // address of data block in flash

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static uint8_t MEMORY_CheckMatch(uint8_t* data, uint8_t size, uint8_t value);

/******************************************************************************/

/**
 * @func   MEMORY_CheckMatch
 * @brief  Initialized flash to store custom data
 * @param  None
 * @retval 1 if match all bytes
 */
 
static uint8_t MEMORY_CheckMatch(uint8_t* data, uint8_t size, uint8_t value){
	uint8_t i;
	for(i = 0; i < size; i++){
		if(data[i] != value){
			return 0;
		}
	}
	return 1;
}
/**
 * @func   MEMORY_Init
 * @brief  Read current data block address 
 * @param  None
 * @retval None
 */

void MEMORY_Init(void){
	uint32_t i;
	uint8_t buffer[BLOCK_USE_SIZE];
	
	// Check last
	for(i = 0; i < FLASH_USE_SIZE; i += BLOCK_USE_SIZE){
		FLASH_ReadBytes(FLASH_START_ADDRESS + i, buffer, BLOCK_USE_SIZE);
		if(MEMORY_CheckMatch(buffer, BLOCK_USE_SIZE, MAX_UINT8_T)){
			break;
		}
	}
	
	// i = 0 if first times or last adrress is maximum block
	if(i == 0){
		addressStart = FLASH_END_ADDRESS - BLOCK_USE_SIZE;
	}
	else{
		addressStart = FLASH_START_ADDRESS + i - BLOCK_USE_SIZE;
	}
}


/**
 * @func   MEMORY_Erase
 * @brief  Erase flash use
 * @param  None
 * @retval None
 */

void MEMORY_Erase(void){
	FLASH_ErasePages(FLASH_START_ADDRESS, FLASH_USE_SIZE);
    addressStart = FLASH_END_ADDRESS - BLOCK_USE_SIZE;
}

/**
 * @func   MEMORY_Write
 * @brief  Write block data to flash
 * @param  pointer point to block data
 * @retval None
 */

void MEMORY_Write(uint8_t* block){
	addressStart += BLOCK_USE_SIZE;
    if (addressStart >= FLASH_END_ADDRESS) {
        MEMORY_Erase();
        addressStart = FLASH_START_ADDRESS;
    }
	// Write to flash
	FLASH_WriteBytes(addressStart, block, BLOCK_USE_SIZE);
}

/**
 * @func   MEMORY_Read
 * @brief  Read block data to flash
 * @param  pointer point to store block data
 * @retval None
 */

void MEMORY_Read(uint8_t* block){
	FLASH_ReadBytes(addressStart, block, BLOCK_USE_SIZE);
}

#endif

