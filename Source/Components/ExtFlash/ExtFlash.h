/*
 * EXTFlash.h
 *
 *  Created on: August 17, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_COMPONENTS_EXTFLASH_EXTFLASH_H_
#define _SOURCE_COMPONENTS_EXTFLASH_EXTFLASH_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h" 

#if(EXT_FLASH_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define EXT_FLASH_TYPE                 EXT_FLASH_W25Q32JV

#if(EXT_FLASH_TYPE == EXT_FLASH_W25Q32JV)
#define EXT_FLASH_DEVICE_ID            0xEF401600
#define CHIP_ERASE                     0x60
#define BLOCK_ERASE                    0x52           // 32KB
#else
#define CHIP_ERASE                     0xC7
#define BLOCK_ERASE                    0xD8           // 64 KB
#endif




#define WRITE_ENABLE                   0x06
#define WRITE_DISABLE                  0x04
#define READ_STATUS                    0x05
#define WRITE_STATUS                   0x01
#define READ_DATA                      0x03
#define FAST_READ                      0x0B
#define FAST_READ_DUAL                 0x3B
#define PAGE_PROGRAM                   0x02
#define SECTOR_ERASE                   0x20
#define POWER_DOWN                     0xB9

#define	READ_ID                        0x9F

#define EXT_FLASH_DUMPY_BYTE           0xFF

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

boolean EXTFLASH_Init(void);
uint32_t EXTFLASH_ReadManufactureID(void);
void EXTFLASH_ReadData(uint32_t address, uint8_t* data, uint8_t size);
void EXTFLASH_WriteData(uint32_t address, uint8_t* data, uint8_t size);

void EXTFLASH_EraseSector(uint32_t address);
void EXTFLASH_EraseBlock(uint32_t address);
void EXTFLASH_EraseChip(void);

/******************************************************************************/
#endif

#endif /* _SOURCE_COMPONENTS_EXTFLASH_EXTFLASH_H_ */
