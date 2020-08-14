/*
 * GPIO.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "GPIO.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

uint32_t const RCC_GPIOx[GPIO_MAX_PORT] = RCC_GPIOx_CONFIG;
GPIO_TypeDef* const PORT_GPIOx[GPIO_MAX_PORT] = PORT_GPIOx_CONFIG;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/**
 * @func   GPIO_PinMode
 * @brief  Set pin as mode
 * @param  Port and Pin
 *         mode
 * @retval None
 */

void GPIO_PinMode(uint8_t port, uint8_t pin, GPIOMode_TypeDef mode){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* GPIOx Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_GPIOx[port], ENABLE);
	if( (mode == GPIO_Mode_AIN) || (mode == GPIO_Mode_AF_OD) || (mode == GPIO_Mode_AF_PP) ){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	}
	
	GPIO_InitStructure.GPIO_Pin = MAKE_PIN(pin);
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(PORT_GPIOx[port], &GPIO_InitStructure);
}

/**
 * @func   GPIO_PinHigh
 *         GPIO_PinLow
 *         GPIO_PinToggle
 * @brief  Set pin output
 * @param  Port and Pin
 * @retval None
 */

void GPIO_PinHigh(uint8_t port, uint8_t pin){
	PORT_GPIOx[port]->BSRR = MAKE_PIN(pin);
}

void GPIO_PinLow(uint8_t port, uint8_t pin){
	PORT_GPIOx[port]->BRR = MAKE_PIN(pin);
}

void GPIO_PinToggle(uint8_t port, uint8_t pin){
	PORT_GPIOx[port]->ODR ^= MAKE_PIN(pin);
}

/**
 * @func   GPIO_GetPinOutput
 * @brief  Get pin output
 * @param  Port and Pin
 * @retval Pin state
 */

uint8_t GPIO_GetPinOutput(uint8_t port, uint8_t pin){
	return ((PORT_GPIOx[port]->ODR & MAKE_PIN(pin)) == 0) ? 0 : 1;
}

/**
 * @func   GPIO_GetPinInput
 * @brief  Get pin input
 * @param  Port and Pin
 * @retval Pin state
 */

uint8_t GPIO_GetPinInput(uint8_t port, uint8_t pin){
	return ((PORT_GPIOx[port]->IDR & MAKE_PIN(pin)) == 0) ? 0 : 1;
}



