/*
 * MAX6675.c
 *
 *  Created on: March 18, 2021
 *      Author: LongHD
 *
 */

/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "MAX6675.h"

#if MAX6675_ENABLED
#include "Hard/GPIO/GPIO.h"

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

uint8_t MAX6675_SpiRead(void);

/******************************************************************************/

/**
 * @brief  Read a byte from Max6675 (See datasheet for more detail)
 * @param  None
 * @retval Read byte
 */
 
uint8_t MAX6675_SpiRead(void){
	uint8_t retVal = 0;
	
	for(int8_t i = 7; i >= 0; i--){
		GPIO_PinLow(MAX6675_CLK_PORT, MAX6675_CLK_PIN);
		delay_us(MAX6675_PULSE_US);
		
		if(GPIO_GetPinInput(MAX6675_SO_PORT, MAX6675_SO_PIN)){
			retVal |= (1 << i);
		}
		
		GPIO_PinHigh(MAX6675_CLK_PORT, MAX6675_CLK_PIN);
		delay_us(MAX6675_PULSE_US);
	}
	
	return retVal;
}

/**
 * @brief  Read temperature in celsius
 *         Temperature read = 12 bits reading (14-3) * 0.25
 * @param  None
 * @retval Temperature (0 - 1024 C)
 */

float MAX6675_ReadCelsius(void){
	uint16_t read = 0;
	
	MAX6675_ENABLE();
	delay_us(MAX6675_PULSE_US);
	
	read = MAX6675_SpiRead() << 8;
	read |= MAX6675_SpiRead();
	
	MAX6675_DISABLE();
	
	if(read & 0x04){
		return -1;
	}
	
	read >>= 3;
	return read * 0.25;
}


/**
 * @brief  MAX6675 initialization
 * @param  None
 * @retval None
 */

void MAX6675_Init(void){
	GPIO_PinMode(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_Mode_Out_PP);
	GPIO_PinMode(MAX6675_CLK_PORT, MAX6675_CLK_PIN, GPIO_Mode_Out_PP);
	GPIO_PinMode(MAX6675_SO_PORT, MAX6675_SO_PIN, GPIO_Mode_IPD);
	
	delay_us(MAX6675_PULSE_US);
	MAX6675_DISABLE();
}

#endif
