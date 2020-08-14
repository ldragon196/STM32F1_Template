/*
 * Memory.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_COMPONENT_MEMORY_MEMORY_H_
#define _SOURCE_COMPONENT_MEMORY_MEMORY_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(MEMORY_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

/* Block size must be a multiple of 4*/
#define BLOCK_USE_SIZE                  MEMORY_BLOCK_SIZE

/* Use for storing data, start address = FLASH_END_ADDRESS - FLASH_USE_SIZE
 * This example use 1024 bytes (FLASH_PAGE_SIZE), end of flash block
 */
#define FLASH_END_ADDRESS              (FLASH_START_ADDRESS + FLASH_USE_SIZE)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void MEMORY_Init(void);
void MEMORY_Write(uint8_t* block);
void MEMORY_Read(uint8_t* block);
void MEMORY_Erase(void);

/******************************************************************************/

#endif

#endif /* _SOURCE_COMPONENT_MEMORY_MEMORY_H_ */
