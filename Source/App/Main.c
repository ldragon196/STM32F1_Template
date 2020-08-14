/*
 * Main.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
 
/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#define DEBUG_PRINT_ENABLE
#include "Utility/Debug/Debug.h"

#include "Hard/Uart/Uart.h"
#include "Hard/Event/Event.h"
#include "Hard/WDG/WDG.h"

#include "Components/VL53L0X/VL53L0X.h"


/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

EventControl mainProcessEventControl;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static void MAIN_Init(void);

void mainProcessEventFunction(void);

/******************************************************************************/

/**
 * @func   MAIN_Init
 * @brief  Initializes main
 * @param  None
 * @retval None
 */
 
static void MAIN_Init(void){
	/* Initializes system clock */
    SystemCoreClockUpdate();
	
	/* 4 bits for pre-emption priority, 0 bits for subpriority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	UART_Init(DEBUG_UART_COM_CONFIG);
	DEBUG_PRINTLN("Power up");
	
	EVENT_Init();
	IWDG_Init();
	
	VL53L0X_Init();
}

/**
 * @func   Main
 * @brief  Main
 * @param  None
 * @retval None
 */

int main(void){
	// Main init
	MAIN_Init();
	
	EVENT_Creat(&mainProcessEventControl, mainProcessEventFunction);
	EVENT_SetDelayMS(mainProcessEventControl, 1000);
	
	while(1){
		EVENT_Task();
		IWDG_ResetWatchdog();
	}
}

/**
 * @func   mainProcessEventFunction
 * @brief  Main process event
 * @param  None
 * @retval None
 */

void mainProcessEventFunction(void){
	EVENT_SetInactive(mainProcessEventControl);
	
//	DEBUG_PRINTLN(".");
	VL53L0X_Task();
	
	EVENT_SetDelayMS(mainProcessEventControl, 1000);
}










