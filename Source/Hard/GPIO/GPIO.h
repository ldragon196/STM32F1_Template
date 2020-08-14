/*
 * GPIO.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_GPIO_GPIO_H_
#define _SOURCE_HARD_GPIO_GPIO_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

// GPIO Common

typedef struct {
	uint8_t port;
	uint8_t pin;
} GPIO_Pin_t;

#define GPIO_IS_INVALID(port, pin)                 ( ((port) >= GPIO_MAX_PORT) || ((pin) >= GPIO_MAX_PIN) )


/* GPIO Mode
GPIO_Mode_AIN = 0x0,
GPIO_Mode_IN_FLOATING = 0x04,
GPIO_Mode_IPD = 0x28,
GPIO_Mode_IPU = 0x48,
GPIO_Mode_Out_OD = 0x14,
GPIO_Mode_Out_PP = 0x10,
GPIO_Mode_AF_OD = 0x1C,
GPIO_Mode_AF_PP = 0x18
*/

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void GPIO_PinMode(uint8_t port, uint8_t pin, GPIOMode_TypeDef mode);
void GPIO_PinHigh(uint8_t port, uint8_t pin);
void GPIO_PinLow(uint8_t port, uint8_t pin);
void GPIO_PinToggle(uint8_t port, uint8_t pin);
uint8_t GPIO_GetPinOutput(uint8_t port, uint8_t pin);
uint8_t GPIO_GetPinInput(uint8_t port, uint8_t pin);

/******************************************************************************/

#endif /* _SOURCE_HARD_GPIO_GPIO_H_ */
