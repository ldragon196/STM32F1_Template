/*
 *  Board.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define MAKE_PIN(pin)                       ( 1 << (pin) )            //  GPIO_Pin_x
#define MAKE_SOURCE(pin)                    ((uint8_t)(pin))          //  GPIO_PinSourcex
#define MAKE_EXTI_LINE(pin)                 ( 1 << (pin) )            //  EXTI_Linex

enum _IRQn_prio_ {
    IRQn_LevelHighest = 0,
    IRQn_LevelHigh = 1,
    IRQn_LevelNomal = 2,
    IRQn_LevelLow = 3,
	IRQn_LevelLowest = 4,
};


enum _STM32F103_GPIO_PORT_{
	GPIO_PORTA = 0,
	GPIO_PORTB,
	GPIO_PORTC,
	GPIO_PORT_COUNT
};

#define RCC_GPIOx_CONFIG                     { RCC_APB2Periph_GPIOA, \
                                               RCC_APB2Periph_GPIOB, \
                                               RCC_APB2Periph_GPIOC  \
                                             }

#define GPIO_MAX_PORT                        GPIO_PORT_COUNT
#define PORT_GPIOx_CONFIG                    { GPIOA, GPIOB, GPIOC }

#define EXTI_MAX_CHANNEL                     16
#define EXTI_PORT_SOURCE_CONFIG              { GPIO_PortSourceGPIOA, \
									           GPIO_PortSourceGPIOB, \
									           GPIO_PortSourceGPIOC  \
								             }				 
											 
#define MAX_UART_COUNT                       3
#define COM_UART_CONFIG                      {USART1, USART2, USART3}



											 

/* ---------- Pins ---------- */

// USART
#define COM_USART1                           USART1
#define COM_USART1_TX_PORT                   GPIO_PORTA
#define COM_USART1_TX_PIN                    9
#define COM_USART1_RX_PORT                   GPIO_PORTA
#define COM_USART1_RX_PIN                    10
		 
#define COM_USART2                           USART2
#define COM_USART2_TX_PORT                   GPIO_PORTA
#define COM_USART2_TX_PIN                    2
#define COM_USART2_RX_PORT                   GPIO_PORTA
#define COM_USART2_RX_PIN                    3

#define COM_USART3                           USART3
#define COM_USART3_TX_PORT                   GPIO_PORTB
#define COM_USART3_TX_PIN                    10
#define COM_USART3_RX_PORT                   GPIO_PORTB
#define COM_USART3_RX_PIN                    11



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/


#define ENTER_CRITICAL                      _enter_critical()
#define EXIT_CRITICAL                       _exit_critical()

void delay_us( uint32_t us);
void _enter_critical(void);
void _exit_critical(void);

/******************************************************************************/

#endif /* _BOARD_H_ */
