/*
 * Flash.c
 *
*  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Flash.h"

#if(FLASH_ENABLED)

#include "stm32f10x_flash.h"

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

/**
 * @brief  Programs a word at a specified address
 *         See FLASH_ProgramWord description
 * @param  Specifies the address to be programmed and data
 * @retval None
 */

void FLASH_WriteWord(uint32_t address, uint32_t data){
	ENTER_CRITICAL;
	
	FLASH_Unlock();
    FLASH_ProgramWord(address, data);
    FLASH_Lock();
	
	EXIT_CRITICAL;
}


/**
 * @brief  Read a word at a specified address
 * @param  Specifies the address to be programmed and data output
 * @retval None
 */

void FLASH_ReadWord(uint32_t address, uint32_t* data){
	FLASH_Unlock();
    *data = READ32(address);
    FLASH_Lock();
}


/**
 * @brief  Programs n bytes at a specified address
 * @param  Specifies the address to be programmed, pointer and length of data
 * @retval None
 */

void FLASH_WriteBytes(uint32_t address, uint8_t* data, uint32_t length){
	uint32_t tempData = 0;
	uint32_t startAddress, size;
	uint8_t i;
	
	startAddress = address;
	size = length >> 2;  // Write n * 4 bytes
	
	FLASH_Unlock();
	while(size > 0){
		tempData = 0;
		for(i = 0; i < 4; i++){
			tempData |= (*data) << (i << 3);
			data++;
		}
		FLASH_ProgramWord(startAddress, tempData);       
		size--;
		startAddress += 4;
	}
	FLASH_Lock();
}

/**
 * @brief  Read n bytes at a specified address
 * @param  Specifies the address to be programmed, pointer and length of data
 * @retval None
 */

void FLASH_ReadBytes(uint32_t address, uint8_t* data, uint32_t length){
	uint32_t startAddress;
	
	startAddress = address;
	FLASH_Unlock();
    while(length > 0) {
        *data = READ8(startAddress);
        startAddress++;
        data++;
        length--;
    }
    FLASH_Lock();
}


/**
 * @brief  Erase number of pages
 * @param  Specifies the address of pages and size
 * @retval None
 */

void FLASH_ErasePages(uint32_t address, uint32_t size){
	uint8_t numberPages = 0;
	uint32_t startAddress;
	
	startAddress = address;
	numberPages = size / FLASH_PAGE_SIZE;
	if(size > (numberPages * FLASH_PAGE_SIZE)){
		numberPages++;
	}
	
	ENTER_CRITICAL;
	
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_Unlock();
    for (uint8_t i = 0; i < numberPages; i++) {
        FLASH_ErasePage(startAddress);
        startAddress += FLASH_PAGE_SIZE;
    }
    FLASH_Lock();

    EXIT_CRITICAL;
}

#endif
