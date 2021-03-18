/*
 *  WDG.c
 *
 *  Created on: August 13, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "WDG.h"

#if(IWDG_ENABLED)

#include "stm32f10x_iwdg.h"

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



/******************************************************************************/

/**
 * @brief  Initialized independent watchdog
 * @param  None
 * @retval None
 */

void IWDG_Init(void){
	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);
	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){}
	
	/* Enable Watchdog*/
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_PRESCALER);    // 4, 8, 16 ... 256
	IWDG_SetReload(IWDG_RELOAD_VALUE);    //This parameter must be a number between 0 and 0x0FFF.
	IWDG_ReloadCounter();
	IWDG_Enable();
}

/**
 * @brief  Reset watchdog
 * @param  None
 * @retval None
 */


void IWDG_ResetWatchdog(void){
	IWDG_ReloadCounter();
}	

#endif
