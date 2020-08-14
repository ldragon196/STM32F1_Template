/*
 * I2C.h
 *
 *  Created on: August 14, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_I2C_I2C_H_
#define _SOURCE_HARD_I2C_I2C_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(I2C_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define I2C_TOTAL_TIMEOUT                            3000
#define I2C_DUMPY_BYTE                               0x00

typedef struct{
	uint8_t SCLPort;
	uint8_t SCLPin;
	uint8_t SDAPort;
	uint8_t SDAPin;
	uint32_t RCCClock;
	I2C_TypeDef* I2C;
} I2CBase_t;


#define I2C_INSTANCE(num)                               \
{                                                       \
	.SCLPort = CONCAT_3_(I2C, num, _SCL_PORT),          \
	.SCLPin = CONCAT_3_(I2C, num, _SCL_PIN),            \
	.SDAPort = CONCAT_3_(I2C, num, _SDA_PORT),          \
	.SDAPin = CONCAT_3_(I2C, num, _SDA_PIN),            \
	.RCCClock = CONCAT_2_(RCC_APB1Periph_I2C, num),     \
	.I2C = CONCAT_2_(I2C, num),                         \
}



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void I2C_InitAsMaster(I2CBase_t* i2cConfig);

void I2C_WriteData(I2CBase_t* i2cConfig, uint8_t slaveAdd, uint8_t *data, uint8_t size);
void I2C_ReadData(I2CBase_t* i2cConfig, uint8_t slaveAdd, uint8_t *data, uint8_t size);

/******************************************************************************/
#endif

#endif /* _SOURCE_HARD_I2C_I2C_H_ */
