/*
 * Timer.c
 *
 *  Created on: August 13, 2020
 *      Author: LongHD
 */
 
/* Example TIME1-5*/

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Timer.h"

#if(TIMER_ENABLED)

#include "stm32f10x_tim.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

#if((TIMER1_IRQ_ENABLED&TIMER1_ENABLED)|(TIMER2_IRQ_ENABLED&TIMER2_ENABLED)|(TIMER3_IRQ_ENABLED&TIMER3_ENABLED)|(TIMER4_IRQ_ENABLED&TIMER4_ENABLED))
static TimerIRQHandler TimerHandler[TIMER_MAX] = {0};
#endif

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/**
 * @brief  Init timer
 * @param  Timer and parameters
 * @retval None
 */

void TIMER_Init(TimerBase_t* timer, TimerConfig_t* config){
	/* TIMx clock enable */
	if( (timer->Timer == TIM1) || (timer->Timer == TIM8) ){   // TIM1 or TIM8
		RCC_APB2PeriphClockCmd(timer->TimerRCC, ENABLE);
	}
	else{
		RCC_APB1PeriphClockCmd(timer->TimerRCC, ENABLE);
	}
	
	/* Frequence = SystemCoreClock / ( (PreScaler + 1) * ( Periods + 1 ) ) */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = config->Periods;
	TIM_TimeBaseStructure.TIM_Prescaler = config->Prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = config->Division;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = TIM_CounterMode_Up;
	
	/* Time base configuration */
	TIM_TimeBaseInit(timer->Timer, &TIM_TimeBaseStructure);
	
	/* TIM enable counter */
	TIM_Cmd(timer->Timer, ENABLE);
	
	if(timer->Handler != NULL){
		#if((TIMER1_IRQ_ENABLED&TIMER1_ENABLED)|(TIMER2_IRQ_ENABLED&TIMER2_ENABLED)|(TIMER3_IRQ_ENABLED&TIMER3_ENABLED)|(TIMER4_IRQ_ENABLED&TIMER4_ENABLED))
		TimerHandler[timer->Id] = timer->Handler;
		#endif
		
		// Enable interrupt
		NVIC_InitTypeDef nvicStructure;
		nvicStructure.NVIC_IRQChannel = timer->IRQChannel;
		nvicStructure.NVIC_IRQChannelPreemptionPriority = TIMER_IRQ_PRIORITY;
		nvicStructure.NVIC_IRQChannelSubPriority = 0;
		nvicStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvicStructure);
		
		TIM_ITConfig(timer->Timer, TIM_IT_Update, ENABLE);
	}
}

/**
 * @brief  Get timer counter
 * @param  TimerBase_t
 * @retval Timer counter
 */

uint16_t TIMER_GetCounter(TimerBase_t* timer){
	return timer->Timer->CNT;
}


/* TIMERx IRQ Handler */

#if(TIMER1_IRQ_ENABLED&TIMER1_ENABLED)
void TIM1_UP_IRQHandler(void){
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){
		CALLBACK_VOID(TimerHandler[TIMER_ID_1]);
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
#endif

#if(TIMER2_IRQ_ENABLED&TIMER2_ENABLED)
void TIM2_IRQHandler(void){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		CALLBACK_VOID(TimerHandler[TIMER_ID_2]);
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
#endif

#if(TIMER3_IRQ_ENABLED&TIMER3_ENABLED)
void TIM3_IRQHandler(void){
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		CALLBACK_VOID(TimerHandler[TIMER_ID_3]);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}
#endif


#if(TIMER4_IRQ_ENABLED&TIMER4_ENABLED)
void TIM4_IRQHandler(void){
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		CALLBACK_VOID(TimerHandler[TIMER_ID_4]);
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}
#endif

#endif

