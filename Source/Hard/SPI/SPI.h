/*
 * SPI.h
 *
 *  Created on: August 17, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_SPI_SPI_H_
#define _SOURCE_HARD_SPI_SPI_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(SPI_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define SPI_DUMPY_BYTE                         0xFF

enum{
	#if(SPI1_ENABLED)
	SPI_ID_1,
	#endif
	
	#if(SPI2_ENABLED)
	SPI_ID_2,
	#endif
	
	SPI_COUNT
};

typedef struct{
	uint8_t Id;
	SPI_TypeDef* SPI;
} SPIBase_t;

#define SPI_INSTANCE(num)                      \
{                                              \
	.Id = CONCAT_2_(SPI_ID_, num),             \
	.SPI = CONCAT_2_(SPI, num)                 \
}

// Use for slave mode
#if(SPI1_SLAVE_MODE_ENABLED|SPI1_SLAVE_MODE_ENABLED)
typedef void (*SPI_ReceiveHandler)(uint8_t, uint8_t);
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void SPI_InitAsMaster(SPIBase_t* spiConfig);
uint8_t SPI_Transfer(SPIBase_t* spi, uint8_t byte);

#if(SPI1_ENABLED&SPI1_SLAVE_MODE_ENABLED)
void SPI_InitAsSlave(SPIBase_t* spi, SPI_ReceiveHandler initFunc);
void SPI_SlaveResponse(SPIBase_t* spi, uint8_t byte);
#endif

/******************************************************************************/
#endif

#endif /* _SOURCE_HARD_SPI_SPI_H_ */
