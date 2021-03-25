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
#include "Components/ADXL357/ADXL357.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

EventControl mainProcessEventControl;

static char loraSendBuffer[1024];
uint32_t timeStamp = 0;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static void MAIN_Init(void);
void MAIN_CreatJsonPacket(void);
void mainProcessEventFunction(void);

/******************************************************************************/

#define MMIO8(addr)                       (*(volatile uint8_t *)(addr))
#define MMIO16(addr)                      (*(volatile uint16_t *)(addr))
#define MMIO32(addr)                      (*(volatile uint32_t *)(addr))
#define U_ID                              0x1FFFF7E8

/**
 * @brief  Get uid (See 30.2 Unique device ID register (96 bits))
 *         https://www.st.com/resource/en/reference_manual/CD00171190-.pdf
 * @param  First 8 bytes in UID 12 bytes
 * @retval None
 */

void MAIN_GetUID(char* uid){
	uint32_t address = U_ID;
	uint8_t index = 0;
	
	uid[0] = 0;
	for(uint8_t i = 0; i < 8; i++){
		index = strlen(uid);
		sprintf(&uid[index], "%02X", MMIO8(address));
		address++;
	}
}

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
	
	// Sensors initialization
	MAX6675_Init();
	if(ADXL357_Init()){
		DEBUG_PRINTLN("ADXL355 initialized success");
	}
	if(RA02LORA_Init()){
		RA02LORA_SetFrequency(434000000);      // 434MHZ
	}
	else{
		DEBUG_PRINTLN("RA02 Lora initialized failure");
	}
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
	
	MAIN_CreatJsonPacket();
	RA02LORA_SendData((uint8_t*) loraSendBuffer, strlen(loraSendBuffer));
	
	DEBUG_PRINTLN("%s", loraSendBuffer);
	timeStamp++;
	
	EVENT_SetDelayMS(mainProcessEventControl, 1000);
}

/**
 * @brief  Creat json packet to send
 * @param  None
 * @retval None
 */

void MAIN_CreatJsonPacket(void){
	uint16_t index = 0;
	float temperature = 0;
	char uid[16];
	ADXL357_Sensor_t sensor = {0};
	
	ADXL357_Measure(&sensor);
	temperature = MAX6675_ReadCelsius();
	MAIN_GetUID(uid);
	
	loraSendBuffer[0] = 0;
	sprintf(loraSendBuffer, "{\"type\":\"info\",\"module\":\"LD196\",");
	index = strlen(loraSendBuffer);
	sprintf(&loraSendBuffer[index], "\"id\":\"%s\",\"time\":%u,", uid, timeStamp);
	index = strlen(loraSendBuffer);
	sprintf(&loraSendBuffer[index], "\"vibration\":{\"module\":\"ADXL357\",\"x\":%.02f,\"y\":%.02f,\"z\":%.02f},", sensor.x, sensor.y, sensor.z);
	index = strlen(loraSendBuffer);
	sprintf(&loraSendBuffer[index], "\"temperature\":{\"module\":\"MAX6675\",\"value\":%.02f},", temperature);
	index = strlen(loraSendBuffer);
	sprintf(&loraSendBuffer[index], "\"battery\":3000}");
}








