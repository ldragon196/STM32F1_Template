/*
 * ExtFlash.c
 *
 *  Created on: June 23, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Hard/SPI/SPI.h"
#include "Hard/GPIO/GPIO.h"
#include "ExtFlash.h"

#if(EXT_FLASH_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define EXT_FLASH_TIMEOUT_MS                                   5000
#define EXTFLASH_Transfer(a)                                   SPI_Transfer(EXT_FLASH_SPI, (a))

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static void EXTFLASH_Enable(boolean enable);
static void EXTFLASH_Poll(void);
static void EXTFLASH_Command(uint8_t cmd, uint32_t param);

/******************************************************************************/

/**
 * @func   EXTFLASH_Enable
 * @brief  Init spi and external flash
 * @param  None
 * @retval True if success
 */

static void EXTFLASH_Enable(boolean enable){
	if(enable){
		GPIO_PinLow(EXT_FLASH_NSS_PORT, EXT_FLASH_NSS_PIN);
	}
	else{
		GPIO_PinHigh(EXT_FLASH_NSS_PORT, EXT_FLASH_NSS_PIN);
	}
}

/**
 * @func   EXTFLASH_Command
 * @brief  Send command
 * @param  Command and parameters
 * @retval None
 */

static void EXTFLASH_Command(uint8_t cmd, uint32_t param){
	EXTFLASH_Enable(FALSE);
	EXTFLASH_Enable(TRUE);
	
	(void) EXTFLASH_Transfer(cmd);
	switch (cmd){
		case READ_DATA:
		case PAGE_PROGRAM:
			(void) EXTFLASH_Transfer(GET_BIT_16_23(param));
			(void) EXTFLASH_Transfer(GET_BIT_8_15(param));
			(void) EXTFLASH_Transfer(GET_BIT_0_7(param));
			break;
		
		case BLOCK_ERASE:
		case SECTOR_ERASE:
			(void) EXTFLASH_Transfer(GET_BIT_16_23(param));
			(void) EXTFLASH_Transfer(GET_BIT_8_15(param));
			(void) EXTFLASH_Transfer(GET_BIT_0_7(param));
			EXTFLASH_Enable(FALSE);
			break;
		case WRITE_ENABLE:
		case WRITE_DISABLE:
		case CHIP_ERASE:
		case POWER_DOWN:
			EXTFLASH_Enable(FALSE);
			break;
		case WRITE_STATUS:
			break;
		case READ_ID:
			break;
		default:
			break;
	}	
}

/**
 * @func   EXTFLASH_Poll
 * @brief  Check and wait flash is busy
 * @param  None
 * @retval None
 */

static void EXTFLASH_Poll(void){
	uint8_t status;
	uint32_t timeout = EXT_FLASH_TIMEOUT_MS;
	
	EXTFLASH_Command(READ_STATUS, 0);
	do {
		status = EXTFLASH_Transfer(EXT_FLASH_DUMPY_BYTE);
	} while ( ((status & 0x01) || (status == 0xFF)) && ((timeout--) != 0) );
	
	EXTFLASH_Enable(FALSE);
}


/**
 * @func   EXTFLASH_ReadData
 * @brief  Read data from flash
 * @param  address
 *         data
 *         length of data
 * @retval none
 */

void EXTFLASH_ReadData(uint32_t address, uint8_t* data, uint8_t size){
	EXTFLASH_Poll();                  // check external flash busy
	EXTFLASH_Command(READ_DATA, address); // send read command
	
	while(size--){
		*data ++ = EXTFLASH_Transfer(0xFF);
	}
	EXTFLASH_Enable(FALSE);
}


/**
 * @func   EXTFLASH_WriteData
 * @brief  Write data to flash
 * @param  address
 *         data
 *         length of data
 * @retval none
 */

void EXTFLASH_WriteData(uint32_t address, uint8_t* data, uint8_t size){
	EXTFLASH_Poll();
	EXTFLASH_Command(WRITE_ENABLE, 0);
	EXTFLASH_Command(PAGE_PROGRAM, address);
	
	while(size--){
		(void) EXTFLASH_Transfer(*data++);
	}
	
	EXTFLASH_Enable(FALSE);
}


/**
 * @func   Erase_flash
 * @brief  Erase sector, block, chip
 * @param  Address
 * @retval none
 */

void EXTFLASH_EraseSector(uint32_t address){
	EXTFLASH_Poll();
	EXTFLASH_Command(WRITE_ENABLE, 0);
	EXTFLASH_Command(SECTOR_ERASE, address);
}

void EXTFLASH_EraseBlock(uint32_t address){
	EXTFLASH_Poll();
	EXTFLASH_Command(WRITE_ENABLE, 0);
	EXTFLASH_Command(BLOCK_ERASE, address);
}

// Must wait complete. Read datashet, time erase ~ 10 - 15s
void EXTFLASH_EraseChip(void){
	EXTFLASH_Poll();
	EXTFLASH_Command(WRITE_ENABLE, 0);
	EXTFLASH_Command(CHIP_ERASE, 0);
}



/**
 * @func   EXTFLASH_ReadManufactureID
 * @brief  Read id
 * @param  None
 * @retval Device ID
 */

uint32_t EXTFLASH_ReadManufactureID(void){
	uint32_t deviceID = 0;
	uint8_t count = 3;
	
	EXTFLASH_Poll();
	EXTFLASH_Command(READ_ID, 0);
	
	do {
		deviceID |= ( (EXTFLASH_Transfer(EXT_FLASH_DUMPY_BYTE)) << (count << 3) );
	} while(count--);
	
	EXTFLASH_Enable(FALSE);
	return deviceID;
}

/**
 * @func   EXTFLASH_Init
 * @brief  Init spi and external flash
 * @param  None
 * @retval True if success
 */

boolean EXTFLASH_Init(void){
	// Init SPI
	SPI_InitAsMaster(EXT_FLASH_SPI);
	
	// Set NSS Pin
	GPIO_SetPinAsOutput(EXT_FLASH_NSS_PORT, EXT_FLASH_NSS_PIN);
	EXTFLASH_Enable(FALSE);
	
	if(EXTFLASH_ReadManufactureID() == EXT_FLASH_DEVICE_ID){
		return TRUE;
	}
	return FALSE;
}



#endif
