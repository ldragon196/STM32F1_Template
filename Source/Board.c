/*
 *  Board.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Board.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static volatile uint8_t IrqNestLevel = 0;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/**
 * @brief  Delay time us
 * @param  us: microsecond
 * @retval None
 */
void delay_us( uint32_t us){
    uint32_t i;
    uint32_t fms = (SystemCoreClock / 4000000) * us ;
    for (i = 0; i < fms; i++){}
}

/**
 * @brief  Disable all interrupt
 * @param  None
 * @retval None
 */
void _enter_critical(void){
    __disable_irq();
    IrqNestLevel++;  
}

/**
 * @brief  Enable all interrupt
 * @param  None
 * @retval None
 */
void _exit_critical(void){
    IrqNestLevel--;
    if ( IrqNestLevel == 0 ) {
        __enable_irq();
    }   
}


