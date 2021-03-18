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
											 
#define UART_MAX_COUNT                       3
#define COM_UART_CONFIG                      {USART1, USART2, USART3}

#define I2C_MAX_COUNT                        2
#define I2C_CONFIG                           {I2C1, I2C2}

#define SPI_MAX_COUNT                        2
#define SPI_CONFIG                           {SPI1, SPI2}


#define ADC_MAX_COUNT                        2
#define ADC_CONFIG                           {ADC1, ADC2}


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

// I2C
#if(I2C1_REMAP_ENABLED)
#define I2C1_SCL_PORT                        GPIO_PORTB
#define I2C1_SCL_PIN                         8
#define I2C1_SDA_PORT                        GPIO_PORTB
#define I2C1_SDA_PIN                         9
#else
#define I2C1_SCL_PORT                        GPIO_PORTB
#define I2C1_SCL_PIN                         6
#define I2C1_SDA_PORT                        GPIO_PORTB
#define I2C1_SDA_PIN                         7
#endif

#define I2C2_SCL_PORT                        GPIO_PORTB
#define I2C2_SCL_PIN                         10
#define I2C2_SDA_PORT                        GPIO_PORTB
#define I2C2_SDA_PIN                         11

// SPI
#define COM_SPI1_NSS_PORT                    GPIO_PORTA
#define COM_SPI1_NSS_PIN                     4
#define COM_SPI1_SCK_PORT                    GPIO_PORTA
#define COM_SPI1_SCK_PIN                     5
#define COM_SPI1_MISO_PORT                   GPIO_PORTA
#define COM_SPI1_MISO_PIN                    6
#define COM_SPI1_MOSI_PORT                   GPIO_PORTA
#define COM_SPI1_MOSI_PIN                    7

#define COM_SPI2_NSS_PORT                    GPIO_PORTB
#define COM_SPI2_NSS_PIN                     12
#define COM_SPI2_SCK_PORT                    GPIO_PORTB
#define COM_SPI2_SCK_PIN                     13
#define COM_SPI2_MISO_PORT                   GPIO_PORTB
#define COM_SPI2_MISO_PIN                    14
#define COM_SPI2_MOSI_PORT                   GPIO_PORTB
#define COM_SPI2_MOSI_PIN                    15


// ADC
#define ADC1_CHANNEL0_PORT                   GPIO_PORTA
#define ADC1_CHANNEL0_PIN                    0
#define ADC1_CHANNEL1_PORT                   GPIO_PORTA
#define ADC1_CHANNEL1_PIN                    1
#define ADC1_CHANNEL2_PORT                   GPIO_PORTA
#define ADC1_CHANNEL2_PIN                    2
// ...


#define ADC2_CHANNEL0_PORT                   GPIO_PORTA
#define ADC2_CHANNEL0_PIN                    0
#define ADC2_CHANNEL1_PORT                   GPIO_PORTA
#define ADC2_CHANNEL1_PIN                    1
// ...

// DAC
#define DAC1_CHANNEL_PORT                    GPIO_PORTA
#define DAC1_CHANNEL_PIN                     4
#define DAC2_CHANNEL_PORT                    GPIO_PORTA
#define DAC2_CHANNEL_PIN                     5



// External flash
#define EXT_FLASH_NSS_PORT                   GPIO_PORTA
#define EXT_FLASH_NSS_PIN                    4





/****************** Components ******************/

// RA02 Lora
#define RA02_LORA_NSS_PORT                   GPIO_PORTA
#define RA02_LORA_NSS_PIN                    4
#define RA02_LORA_RESET_PORT                 GPIO_PORTA
#define RA02_LORA_RESET_PIN                  8

// MAX6675

#define MAX6675_CS_PORT                      GPIO_PORTB
#define MAX6675_CS_PIN                       12
#define MAX6675_CLK_PORT                     GPIO_PORTB
#define MAX6675_CLK_PIN                      13
#define MAX6675_SO_PORT                      GPIO_PORTB
#define MAX6675_SO_PIN                       14





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
