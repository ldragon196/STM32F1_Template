/*
 * PWM.c
 *
 *  Created on: June 8, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Hard/GPIO/GPIO.h"
#include "PWM.h"

#if(PWM_TIMER_USED_COUNT)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

TIM_TypeDef* const TIMER_BASE[TIME_USED_COUNT] = PWM_TIMER_BASE;
uint32_t const TIMER_RCC[TIME_USED_COUNT] = RCC_TIMER_CONFIG;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/


/**
 * @func   PWM_Init
 * @brief  Init PWM use timer output compare
 * @param  Frequence
 * @retval None
 */

void PWM_Init(PWM_Params_t* PWMInit){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint8_t timerNum, channel;
	
	/* Frequence = SystemCoreClock / ( (PreScaler + 1) * ( Periods + 1 ) )
	*  PreScaler and PreScaler value range 0 - 65535
	*  TIM1,8-11...  clock: AHB2   (See reference manual)
	*  TIM2-7... clock: AHB1
	*/
	
	timerNum = PWMInit->num;
	channel = PWMInit->channel;
	
	/* Config GPIO */
	GPIO_SetPinAsAF(PWMInit->port, PWMInit->pin);
	
	/* TIMx clock enable */
	if( (timerNum == TIMER_1) || (timerNum == TIMER_8) ){
		RCC_APB2PeriphClockCmd(TIMER_RCC[timerNum], ENABLE);
		
		/* Main Output Enable */
		TIM_CtrlPWMOutputs(TIMER_BASE[timerNum], ENABLE);
	}
	else{
		RCC_APB1PeriphClockCmd(TIMER_RCC[timerNum], ENABLE);
	}
	
	TIM_TimeBaseStructure.TIM_Period = (uint16_t) (SystemCoreClock / PWMInit->frequence) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;  // Prescaler 1
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = TIM_CounterMode_Up;
	
	/* Time base configuration */
	TIM_TimeBaseInit(TIMER_BASE[timerNum], &TIM_TimeBaseStructure);
	
	/* OC Mode
	   PWM1: 0 - CC: low, CC - auto reload register: high
	   PWM2: 0 - CC: high, CC - auto reload register: low
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	/* PWMx configuration */
	switch(channel){
		case PWM_CHANNEL_1:
			TIM_OC1Init(TIMER_BASE[timerNum], &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIMER_BASE[timerNum], TIM_OCPreload_Enable);
			break;
		case PWM_CHANNEL_2:
			TIM_OC2Init(TIMER_BASE[timerNum], &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIMER_BASE[timerNum], TIM_OCPreload_Enable);
			break;
		case PWM_CHANNEL_3:
			TIM_OC3Init(TIMER_BASE[timerNum], &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIMER_BASE[timerNum], TIM_OCPreload_Enable);
			break;
		case PWM_CHANNEL_4:
			TIM_OC4Init(TIMER_BASE[timerNum], &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(TIMER_BASE[timerNum], TIM_OCPreload_Enable);
			break;
		default:
			// Invalid
			break;
	}
	
	TIM_ARRPreloadConfig(TIMER_BASE[timerNum], ENABLE);

	/* TIM enable counter */
	TIM_Cmd(TIMER_BASE[timerNum], ENABLE);
}


/**
 * @func   PWM_SetCCValue
 * @brief  Set output compare value
 * @param  Timer number, channel set and compare percentage (0 - PWM_COMPARE_MAX)
 * @retval None
 */

void PWM_SetCCValue(uint8_t timerNum, uint8_t channel, uint32_t compare){
	uint32_t arrValue, setValue;
	
	arrValue = TIMER_BASE[timerNum]->ARR;
	if(compare >= PWM_COMPARE_MAX){
		setValue = arrValue + 1;
	}
	else{
		setValue = compare * arrValue / PWM_COMPARE_MAX;		
	}
	
	/* PWMx configuration */
	switch(channel){
		case PWM_CHANNEL_1:
			TIM_SetCompare1(TIMER_BASE[timerNum], setValue);
			break;
		case PWM_CHANNEL_2:
			TIM_SetCompare2(TIMER_BASE[timerNum], setValue);
			break;
		case PWM_CHANNEL_3:
			TIM_SetCompare3(TIMER_BASE[timerNum], setValue);
			break;
		case PWM_CHANNEL_4:
			TIM_SetCompare4(TIMER_BASE[timerNum], setValue);
			break;
		default:
			// Invalid
			break;
	}
}

/**
 * @func   PWM_Deinit
 * @brief  Deinit timer
 * @param  Timer number
 * @retval None
 */

void PWM_Deinit(uint8_t timerNum){
	TIM_DeInit(TIMER_BASE[timerNum]);
}

#endif
