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

enum{
	#if(I2C1_ENABLED)
	I2C_ID_1,
	#endif
	
	#if(I2C2_ENABLED)
	I2C_ID_2,
	#endif
	
	I2C_COUNT
};

typedef struct{
	uint8_t Id;
	I2C_TypeDef* I2C;
} I2CBase_t;


#define I2C_INSTANCE(num)                               \
{                                                       \
	.Id = CONCAT_2_(I2C_ID_, num),                      \
	.I2C = CONCAT_2_(I2C, num)                          \
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
