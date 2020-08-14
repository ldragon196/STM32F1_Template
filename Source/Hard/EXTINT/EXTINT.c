/*
 * EXTINT.c
 *
 *  Created on: August 13, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Hard/GPIO/GPIO.h"
#include "EXTINT.h"

#if(EXTINT_ENABLED)

#include "stm32f10x_exti.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define InvokeHandler(channel)            do{ if(EXTINT_IRQHandler[(channel)] != NULL) { EXTINT_IRQHandler[(channel)]((channel)); } } while(0)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

uint8_t const EXTI_PORT_SOURCE[GPIO_PORT_COUNT] = EXTI_PORT_SOURCE_CONFIG;
static EXTINT_Handler EXTINT_IRQHandler[EXTI_MAX_CHANNEL] = {0};

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/**
 * @func   GPIO_SetModeEXTI
 * @brief  Set pin as input and register interrup handler
 *         See manual, 10.2 External interrupt/event controller (EXTI)
 *         GPIO_Portx, pin n  ->  GPIO_PinSource n, EXTI_Line n
 *         GPIO_Portx, pin 0,1,..4  ->  EXTI0_IRQn0,1,..4
 *         GPIO_Portx, pin 5,...9  ->  EXTI9_5_IRQHandler
 *         GPIO_Portx, pin 10,...15  ->  EXTI15_10_IRQHandler
 *
 * @param  Port and Pin
 *         Callback handler irq
 * @retval None
 */

void EXTINT_Init(uint8_t port, uint8_t pin, EXTINT_Handler initFunc){
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// Set pin as input
	GPIO_PinMode(port, pin, (GPIOMode_TypeDef) EXTINT_PIN_MODE);
	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* Connect EXTIx Line to PX.x pin */
	GPIO_EXTILineConfig(EXTI_PORT_SOURCE[port], MAKE_SOURCE(pin)); 
	
	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = MAKE_EXTI_LINE(pin);
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = (EXTITrigger_TypeDef) EXTINT_TRIGGER;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set EXTI0 Interrupt to the lowest priority */
	if(pin < 5){
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn + pin;
	}
	else if(pin < 10){
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	}
	else{
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	}
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTINT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	EXTINT_IRQHandler[pin] = initFunc;
}

/**
 * @func   EXTIx_IRQHandler
 * @brief  EXTIx IRQ
 * @param  None
 * @retval None
 */

#if(EXTINT0_ENABLED)
void EXTI0_IRQHandler(void){
	InvokeHandler(EXTI_CHANNEL_0);
	EXTI_ClearFlag(EXTI_Line0);
}
#endif

#if(EXTINT1_ENABLED)
void EXTI1_IRQHandler(void){
	InvokeHandler(EXTI_CHANNEL_1);
	EXTI_ClearFlag(EXTI_Line1);
}
#endif

#if(EXTINT2_ENABLED)
void EXTI2_IRQHandler(void){
	InvokeHandler(EXTI_CHANNEL_2);
	EXTI_ClearFlag(EXTI_Line2);
}
#endif

#if(EXTINT3_ENABLED)
void EXTI3_IRQHandler(void){
	InvokeHandler(EXTI_CHANNEL_3);
	EXTI_ClearFlag(EXTI_Line3);
}
#endif

#if(EXTINT4_ENABLED)
void EXTI4_IRQHandler(void){
	InvokeHandler(EXTI_CHANNEL_4);
	EXTI_ClearFlag(EXTI_Line4);
}
#endif

#if(EXTINT9_5_ENABLED)
void EXTI9_5_IRQHandler(void){

	#if(EXTINT9_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
		InvokeHandler(EXTI_CHANNEL_9);
		EXTI_ClearFlag(EXTI_Line9);
	}
	#endif
	
	#if(EXTINT8_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
		InvokeHandler(EXTI_CHANNEL_8);
		EXTI_ClearFlag(EXTI_Line8);
	}
	#endif
	
	#if(EXTINT7_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		InvokeHandler(EXTI_CHANNEL_7);
		EXTI_ClearFlag(EXTI_Line7);
	}
	#endif
	
	#if(EXTINT6_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		InvokeHandler(EXTI_CHANNEL_6);
		EXTI_ClearFlag(EXTI_Line6);
	}
	#endif
	
	#if(EXTINT5_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		InvokeHandler(EXTI_CHANNEL_5);
		EXTI_ClearFlag(EXTI_Line5);
	}
	#endif
}
#endif


#if(EXTINT15_10_ENABLED)
void EXTI15_10_IRQHandler(void){
	#if(EXTINT15_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
		InvokeHandler(EXTI_CHANNEL_15);
		EXTI_ClearFlag(EXTI_Line15);
	}
	#endif

	#if(EXTINT14_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
		InvokeHandler(EXTI_CHANNEL_14);
		EXTI_ClearFlag(EXTI_Line14);
	}
	#endif
	
	#if(EXTINT13_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
		InvokeHandler(EXTI_CHANNEL_13);
		EXTI_ClearFlag(EXTI_Line13);
	}
	#endif
	
	#if(EXTINT12_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
		InvokeHandler(EXTI_CHANNEL_12);
		EXTI_ClearFlag(EXTI_Line12);
	}
	#endif
	
	#if(EXTINT11_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		InvokeHandler(EXTI_CHANNEL_11);
		EXTI_ClearFlag(EXTI_Line11);
	}
	#endif
	
	#if(EXTINT10_ENABLED)
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		InvokeHandler(EXTI_CHANNEL_10);
		EXTI_ClearFlag(EXTI_Line10);
	}
	#endif
}
#endif

#endif
