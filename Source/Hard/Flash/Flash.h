/*
 * Flash.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_FLASH_FLASH_H_
#define _SOURCE_HARD_FLASH_FLASH_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(FLASH_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define FLASH_USE_SIZE                 (FLASH_PAGE_SIZE * FLASH_NUMBER_PAGE)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void FLASH_WriteWord(uint32_t address, uint32_t data);
void FLASH_ReadWord(uint32_t address, uint32_t* data);
void FLASH_WriteBytes(uint32_t address, uint8_t* data, uint32_t length);
void FLASH_ReadBytes(uint32_t address, uint8_t* data, uint32_t length);
void FLASH_ErasePages(uint32_t address, uint32_t size);

/******************************************************************************/

#endif

#endif /* _SOURCE_HARD_FLASH_FLASH_H_ */
