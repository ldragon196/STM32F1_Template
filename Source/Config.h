/*
 * Config.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_CONFIG_CONFIG_H_
#define _SOURCE_CONFIG_CONFIG_H_
// <<< Use Configuration Wizard in Context Menu >>>

#include "Utility/Utility.h"
#include "Board.h"

// <h> Drivers

//==========================================================
// <e> UART_ENABLED - UART peripheral driver
//==========================================================
#ifndef UART_ENABLED
#define UART_ENABLED 1
#endif

// <o> UART_DEFAULT_CONFIG_HWFC  - Hardware Flow Control
 
// <0=> Disabled 
// <256=> HWFC RTS 
// <512=> HWFC CTS 
// <768=> HWFC RTS + CTS 

#ifndef UART_DEFAULT_CONFIG_HWFC
#define UART_DEFAULT_CONFIG_HWFC 0
#endif

// <o> UART_DEFAULT_CONFIG_PARITY  - Parity

// <0=> None 
// <1024=> Even 
// <1536=> Odd 

#ifndef UART_DEFAULT_CONFIG_PARITY
#define UART_DEFAULT_CONFIG_PARITY 0
#endif

// <o> UART_DEFAULT_CONFIG_STOPBITS  - Stop bits

// <0=> Stopbits 1 
// <4096=> Stopbits 0 
// <8192=> Stopbits 2 
// <12288=> Stopbits 1_5 

#ifndef UART_DEFAULT_CONFIG_STOPBITS
#define UART_DEFAULT_CONFIG_STOPBITS 0
#endif

// <o> UART_DEFAULT_CONFIG_WORDLENGTH  - Wordlength

// <0=> 8b 
// <4096=> 9b 

#ifndef UART_DEFAULT_CONFIG_WORDLENGTH
#define UART_DEFAULT_CONFIG_WORDLENGTH 0
#endif

// <o> UART_CONFIG_RX_IRQ_PRIORITY  - Interrupt priority

// <0=> Highest 
// <1=> High 
// <2=> Medium 
// <3=> Low 
// <4=> Lowest 
// <5=> Disable 

#ifndef UART_CONFIG_RX_IRQ_PRIORITY
#define UART_CONFIG_RX_IRQ_PRIORITY 3
#endif

// <o> UART_DEFAULT_CONFIG_RX_FIFO_SIZE  - Rx fifo size

#ifndef UART_DEFAULT_CONFIG_RX_FIFO_SIZE
#define UART_DEFAULT_CONFIG_RX_FIFO_SIZE 256
#endif

// <e> UART1_ENABLED - Enable UART1 instance
//==========================================================
#ifndef UART1_ENABLED
#define UART1_ENABLED 1
#endif

// <q> UART1_CONFIG_RX_IRQ_ENABLED  - Interrupt priority


#ifndef UART1_CONFIG_RX_IRQ_ENABLED
#define UART1_CONFIG_RX_IRQ_ENABLED 1
#endif

// <o> UART1_DEFAULT_CONFIG_BAUDRATE  - Default Baudrate

#ifndef UART1_DEFAULT_CONFIG_BAUDRATE
#define UART1_DEFAULT_CONFIG_BAUDRATE 115200
#endif

// </e>

// <e> UART2_ENABLED - Enable UART2 instance
 //==========================================================
#ifndef UART2_ENABLED
#define UART2_ENABLED 0
#endif

// <q> UART2_CONFIG_RX_IRQ_ENABLED  - Interrupt priority


#ifndef UART2_CONFIG_RX_IRQ_ENABLED
#define UART2_CONFIG_RX_IRQ_ENABLED 1
#endif

// <o> UART2_DEFAULT_CONFIG_BAUDRATE  - Default Baudrate


#ifndef UART2_DEFAULT_CONFIG_BAUDRATE
#define UART2_DEFAULT_CONFIG_BAUDRATE 115200
#endif

// </e>

// <e> UART3_ENABLED - Enable UART3 instance
//==========================================================
#ifndef UART3_ENABLED
#define UART3_ENABLED 0
#endif

// <q> UART3_CONFIG_RX_IRQ_ENABLED  - Interrupt priority


#ifndef UART3_CONFIG_RX_IRQ_ENABLED
#define UART3_CONFIG_RX_IRQ_ENABLED 1
#endif

// <o> UART3_DEFAULT_CONFIG_BAUDRATE  - Default Baudrate


#ifndef UART3_DEFAULT_CONFIG_BAUDRATE
#define UART3_DEFAULT_CONFIG_BAUDRATE 115200
#endif

// </e>

// </e>

// <e> FLASH_ENABLED - Internal flash
//==========================================================
#ifndef FLASH_ENABLED
#define FLASH_ENABLED 0
#endif

// <o> FLASH_START_ADDRESS  - Start address


#ifndef FLASH_START_ADDRESS
#define FLASH_START_ADDRESS 0x08008000
#endif

// <o> FLASH_PAGE_SIZE  - Page size


#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 1024
#endif

// <o> FLASH_NUMBER_PAGE  - Number of pages used


#ifndef FLASH_NUMBER_PAGE
#define FLASH_NUMBER_PAGE 1
#endif

// </e>

// <e> IWDG_ENABLED - Watchdog
//==========================================================
#ifndef IWDG_ENABLED
#define IWDG_ENABLED 1
#endif

// <o> WDG_TIMEOUT_MS  - Watchdog timeout, maximum 10000 (millisecond)


#ifndef WDG_TIMEOUT_MS
#define WDG_TIMEOUT_MS 3000
#endif

// </e>

// <e> EXTINT_ENABLED - External interrupt
//==========================================================
#ifndef EXTINT_ENABLED
#define EXTINT_ENABLED 1
#endif

// <o> EXTINT_PRIORITY  - External interrupt priority

// <0=> Highest	
// <1=> High 
// <2=> Medium 
// <3=> Low	
// <4=> Lowest 

#ifndef EXTINT_PRIORITY
#define EXTINT_PRIORITY 4
#endif

// <o> EXTINT_PIN_MODE  - External interrupt pins mode

// <4=> Floating 
// <40=> Pull down 
// <72=> Pull up 

#ifndef EXTINT_PIN_MODE
#define EXTINT_PIN_MODE 72
#endif

// <o> EXTINT_TRIGGER  - External interrupt trigger

// <8=> Rising 
// <12=> Falling 
// <16=> Rising Falling 

#ifndef EXTINT_TRIGGER
#define EXTINT_TRIGGER 16
#endif

// <q> EXTINT0_ENABLED  - EXTI Instance 0


#ifndef EXTINT0_ENABLED
#define EXTINT0_ENABLED 0
#endif

// <q> EXTINT1_ENABLED  - EXTI Instance 1


#ifndef EXTINT1_ENABLED
#define EXTINT1_ENABLED 1
#endif

// <q> EXTINT2_ENABLED  - EXTI Instance 2


#ifndef EXTINT2_ENABLED
#define EXTINT2_ENABLED 0
#endif

// <q> EXTINT3_ENABLED  - EXTI Instance 3


#ifndef EXTINT3_ENABLED
#define EXTINT3_ENABLED 0
#endif

// <q> EXTINT4_ENABLED  - EXTI Instance 4


#ifndef EXTINT4_ENABLED
#define EXTINT4_ENABLED 0
#endif

// <q> EXTINT5_ENABLED  - EXTI Instance 5


#ifndef EXTINT5_ENABLED
#define EXTINT5_ENABLED 0
#endif

// <q> EXTINT6_ENABLED  - EXTI Instance 6


#ifndef EXTINT6_ENABLED
#define EXTINT6_ENABLED 0
#endif

// <q> EXTINT7_ENABLED  - EXTI Instance 7


#ifndef EXTINT7_ENABLED
#define EXTINT7_ENABLED 0
#endif

// <q> EXTINT8_ENABLED  - EXTI Instance 8


#ifndef EXTINT8_ENABLED
#define EXTINT8_ENABLED 1
#endif

// <q> EXTINT9_ENABLED  - EXTI Instance 9


#ifndef EXTINT9_ENABLED
#define EXTINT9_ENABLED 0
#endif

// <q> EXTINT10_ENABLED  - EXTI Instance 10


#ifndef EXTINT10_ENABLED
#define EXTINT10_ENABLED 0
#endif

// <q> EXTINT11_ENABLED  - EXTI Instance 11


#ifndef EXTINT11_ENABLED
#define EXTINT11_ENABLED 0
#endif

// <q> EXTINT12_ENABLED  - EXTI Instance 12


#ifndef EXTINT12_ENABLED
#define EXTINT12_ENABLED 0
#endif

// <q> EXTINT13_ENABLED  - EXTI Instance 13


#ifndef EXTINT13_ENABLED
#define EXTINT13_ENABLED 0
#endif

// <q> EXTINT14_ENABLED  - EXTI Instance 14


#ifndef EXTINT14_ENABLED
#define EXTINT14_ENABLED 1
#endif

// <q> EXTINT15_ENABLED  - EXTI Instance 15


#ifndef EXTINT15_ENABLED
#define EXTINT15_ENABLED 0
#endif

// </e>


// <e> TIMER_ENABLED - Timer driver
//==========================================================
#ifndef TIMER_ENABLED
#define TIMER_ENABLED 0
#endif

// <o> TIMER_IRQ_PRIORITY  - Timer interrupt priority

// <0=> Highest	
// <1=> High 
// <2=> Medium 
// <3=> Low	
// <4=> Lowest 

#ifndef TIMER_IRQ_PRIORITY
#define TIMER_IRQ_PRIORITY 1
#endif

// <e> TIMER1_ENABLED - Timer instance 1
//==========================================================
#ifndef TIMER1_ENABLED
#define TIMER1_ENABLED 1
#endif

// <q> TIMER1_IRQ_ENABLED  - Timer 1 IRQ enable


#ifndef TIMER1_IRQ_ENABLED
#define TIMER1_IRQ_ENABLED 1
#endif

// </e>

// <e> TIMER2_ENABLED - Timer instance 2
//==========================================================
#ifndef TIMER2_ENABLED
#define TIMER2_ENABLED 1
#endif

// <q> TIMER2_IRQ_ENABLED  - Timer 2 IRQ enable


#ifndef TIMER2_IRQ_ENABLED
#define TIMER2_IRQ_ENABLED 1
#endif

// </e>

// <e> TIMER3_ENABLED - Timer instance 3
//==========================================================
#ifndef TIMER3_ENABLED
#define TIMER3_ENABLED 1
#endif

// <q> TIMER3_IRQ_ENABLED  - Timer 3 IRQ enable


#ifndef TIMER3_IRQ_ENABLED
#define TIMER3_IRQ_ENABLED 1
#endif

// </e>

// <e> TIMER4_ENABLED - Timer instance 4
//==========================================================
#ifndef TIMER4_ENABLED
#define TIMER4_ENABLED 1
#endif

// <q> TIMER4_IRQ_ENABLED  - Timer 4 IRQ enable


#ifndef TIMER4_IRQ_ENABLED
#define TIMER4_IRQ_ENABLED 1
#endif

// </e>

// </e>


// <e> PWM_ENABLED - PWM driver
//==========================================================
#ifndef PWM_ENABLED
#define PWM_ENABLED 1
#endif

// </e>


// <e> I2C_ENABLED - I2C driver
//==========================================================
#ifndef I2C_ENABLED
#define I2C_ENABLED 1
#endif

// <o> I2C_CLOCK_SPEED  - I2C clock speed

#ifndef I2C_CLOCK_SPEED
#define I2C_CLOCK_SPEED 400000
#endif

// <o> I2C_ACK_ENABLED  - I2C ACK enabled

// <0=> Disable 
// <1024=> Enable 

#ifndef I2C_ACK_ENABLED
#define I2C_ACK_ENABLED 1024
#endif



// <q> I2C1_ENABLED  - I2C instance 1


#ifndef I2C1_ENABLED
#define I2C1_ENABLED 1
#endif

// <q> I2C2_ENABLED  - I2C instance 2


#ifndef I2C2_ENABLED
#define I2C2_ENABLED 0
#endif

// </e>

// <e> SPI_ENABLED - SPI driver
//==========================================================
#ifndef SPI_ENABLED
#define SPI_ENABLED 1
#endif

// <o> SPI_SLAVE_IRQ_PRIORITY  - SPI Slave receive IRQ priority

// <0=> Highest	
// <1=> High 
// <2=> Medium 
// <3=> Low	
// <4=> Lowest 

#ifndef SPI_SLAVE_IRQ_PRIORITY
#define SPI_SLAVE_IRQ_PRIORITY 3
#endif

// <e> SPI1_ENABLED  - SPI instance 1


#ifndef SPI1_ENABLED
#define SPI1_ENABLED 1
#endif

// <q> SPI1_SLAVE_MODE_ENABLED  - SPI instance 1 enable slave mode


#ifndef SPI1_SLAVE_MODE_ENABLED
#define SPI1_SLAVE_MODE_ENABLED 1
#endif

// </e>

// <e> SPI2_ENABLED  - SPI instance 2


#ifndef SPI2_ENABLED
#define SPI2_ENABLED 0
#endif

// <q> SPI2_SLAVE_MODE_ENABLED  - SPI instance 2 enable slave mode


#ifndef SPI2_SLAVE_MODE_ENABLED
#define SPI2_SLAVE_MODE_ENABLED 1
#endif

// </e>

// </e>

// <e> ADC_ENABLED - ADC driver
//==========================================================
#ifndef ADC_ENABLED
#define ADC_ENABLED 1
#endif


// <q> ADC1_ENABLED  - ADC instance 1


#ifndef ADC1_ENABLED
#define ADC1_ENABLED 1
#endif

// <q> ADC2_ENABLED  - ADC instance 2


#ifndef ADC2_ENABLED
#define ADC2_ENABLED 1
#endif

// </e>








// </h> 

//==========================================================

// <h> Libraries

//==========================================================
// <e> EVENT_ENABLED - Events scheduler
//==========================================================
#ifndef EVENT_ENABLED
#define EVENT_ENABLED 1
#endif

// <o> EVENT_MAX_CONTROL - Max event control. The more events, the slower system run

#ifndef EVENT_MAX_CONTROL
#define EVENT_MAX_CONTROL 8
#endif

// <o> EVENT_TIMER_USED - Event tick use timer?

// <0=> System tick 
// <1=> TIMER 1 
// <2=> TIMER 2 
// <3=> TIMER 3 
// <4=> TIMER 4 

#ifndef EVENT_TIMER_USED
#define EVENT_TIMER_USED 0
#endif

// </e>

// <e> MEMORY_ENABLED - Write data to internal flash
//==========================================================
#ifndef MEMORY_ENABLED
#define MEMORY_ENABLED 0
#endif

// <o> MEMORY_BLOCK_SIZE - Block size must be a multiple of 4


#ifndef MEMORY_BLOCK_SIZE
#define MEMORY_BLOCK_SIZE 8
#endif


// </e>

// <e> EXT_FLASH_ENABLED - External flash driver
//==========================================================
#ifndef EXT_FLASH_ENABLED
#define EXT_FLASH_ENABLED 0
#endif

// <o> EXT_FLASH_SPI_USE - SPI id used

// <1=> SPI 1 
// <2=> SPI 2 

#ifndef EXT_FLASH_SPI_USE
#define EXT_FLASH_SPI_USE 1
#endif

// </e>


// <e> FIFO_ENABLED - Fifo - Fisrt in first out
//==========================================================
#ifndef FIFO_ENABLED
#define FIFO_ENABLED 0
#endif

// <o> FIFO_UNIT_SIZE - Unit data size


#ifndef FIFO_UNIT_SIZE
#define FIFO_UNIT_SIZE 16
#endif


// </e>



// </h> 

// <h> Components

//==========================================================

// <e> VL53L0X_ENABLED - VL53L0X sensor enable

#ifndef VL53L0X_ENABLED
#define VL53L0X_ENABLED 0
#endif

// <q> VL53L0X_DEBUG_ENABLED - VL53L0X debug


#ifndef VL53L0X_DEBUG_ENABLED
#define VL53L0X_DEBUG_ENABLED 1
#endif

// <o> VL53L0X_I2C_USED  - I2C used

// <1=> I2C 1  
// <2=> I2C 2 

#ifndef VL53L0X_I2C_USED
#define VL53L0X_I2C_USED 1
#endif

// <o> VL53L0X_TIME_MEASURE  - Time measure in ms

#ifndef VL53L0X_TIME_MEASURE
#define VL53L0X_TIME_MEASURE 10
#endif



// </e>

// </h> 





// <h> Debug

//==========================================================

// <e> DEBUG_ENABLED - Serial debug

#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED 1
#endif

// <o> DEBUG_UART_COM_CONFIG  - Debug com

// <0=> UART 1 
// <1=> UART 2 
// <2=> UART 3 

#ifndef DEBUG_UART_COM_CONFIG
#define DEBUG_UART_COM_CONFIG 0
#endif

// </e>

// </h> 

//==========================================================

// <<< end of configuration section >>>
#endif /* _SOURCE_CONFIG_CONFIG_H_ */
