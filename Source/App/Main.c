/*
 * Main.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 
 * Keil C 5: Debug -> Setting -> Pack -> Unchecked "Enable"
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

#include "Components/RA02Lora/RA02Lora.h"
#include "Components/MAX6675/MAX6675.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define LORA_TX 0
#define LORA_RX 0//(!LORA_TX)

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
	
	if(RA02LORA_Init()){
		RA02LORA_SetFrequency(434000000);      // 434MHZ
	}
	else{
		DEBUG_PRINTLN("RA02 Lora initialized failure");
	}
	
	MAX6675_Init();
}

/**
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
		#if LORA_RX
		RA02LORA_Task();
		#endif
		
		EVENT_Task();
		IWDG_ResetWatchdog();
	}
}

/**
 * @brief  Main process event
 * @param  None
 * @retval None
 */

void mainProcessEventFunction(void){
	EVENT_SetInactive(mainProcessEventControl);
	
	#if LORA_TX
	char data[] = "LongHD";
	RA02LORA_SendData((uint8_t*) data, strlen(data));
	
	#elif LORA_RX
		uint8_t read;
		while(RA02LORA_Read(&read)){
			DEBUG_PRINT("%02X ", read);
		}
	#endif
	
	float temp = MAX6675_ReadCelsius();
	DEBUG_PRINT("%.02f", temp);
	
	EVENT_SetDelayMS(mainProcessEventControl, 1000);
}










