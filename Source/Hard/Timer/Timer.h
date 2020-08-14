/*
 * Timer.h
 *
 *  Created on: August 13, 2020
 *      Author: LongHD
 */
 
/******************************************************************************/

#ifndef _SOURCE_HARD_TIMER_TIMER_H_
#define _SOURCE_HARD_TIMER_TIMER_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(TIMER_ENABLED)
#include "stm32f10x_tim.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef void (*TimerIRQHandler)(void);

enum{
	#if(TIMER1_ENABLED)
	TIMER_1 = 0,
	#endif
	
	#if(TIMER2_ENABLED)
	TIMER_2 = 1,
	#endif
	
	#if(TIMER3_ENABLED)
	TIMER_3 = 2,
	#endif
	
	#if(TIMER4_ENABLED)
	TIMER_4 = 3,
	#endif
	
	TIMER_MAX
};

typedef struct {
	uint8_t Id;
	uint8_t IRQChannel;
	uint32_t TimerRCC;
	TIM_TypeDef* Timer;
	TimerIRQHandler Handler;
} TimerBase_t;

/* Frequence = SystemCoreClock / ( (PreScaler + 1) * ( Periods + 1 ) ) */

typedef struct{
	uint16_t Division;          // enum TIM_CKD_DIV1, TIM_CKD_DIV2, TIM_CKD_DIV4
	uint16_t Prescaler;         // 0 - 0xFFFF
	uint16_t Periods;           // 0 - 0xFFFF
} TimerConfig_t;

#define TIMER1_8_INSTANCE(num)                          \
{                                                       \
	.Id = CONCAT_2_(TIMER_, num),                       \
	.IRQChannel = CONCAT_3_(TIM, num, _UP_IRQn),        \
	.TimerRCC = CONCAT_2_(RCC_APB2Periph_TIM, num),     \
	.Timer = CONCAT_2_(TIM, num),                       \
	.Handler = NULL                                     \
}

#define TIMER2_5_INSTANCE(num)                          \
{                                                       \
	.Id = CONCAT_2_(TIMER_, num),                       \
	.IRQChannel = CONCAT_3_(TIM, num, _IRQn),           \
	.TimerRCC = CONCAT_2_(RCC_APB1Periph_TIM, num),     \
	.Timer = CONCAT_2_(TIM, num),                       \
	.Handler = NULL                                     \
}



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void TIMER_Init(TimerBase_t* timer, TimerConfig_t* config);
uint16_t TIMER_GetCounter(TimerBase_t* timer);

/******************************************************************************/
#endif

#endif /* _SOURCE_HARD_TIMER_TIMER_H_ */
