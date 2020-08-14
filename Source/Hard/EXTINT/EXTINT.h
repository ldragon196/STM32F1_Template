/*
 * EXTINT.h
 *
 *  Created on: August 13, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_EXTINT_EXTINT_H_
#define _SOURCE_HARD_EXTINT_EXTINT_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(EXTINT_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef void (*EXTINT_Handler)(uint8_t);

#define EXTINT9_5_ENABLED      (EXTINT9_ENABLED | EXTINT8_ENABLED | EXTINT7_ENABLED | EXTINT6_ENABLED | EXTINT5_ENABLED)
#define EXTINT15_10_ENABLED    (EXTINT15_ENABLED | EXTINT14_ENABLED | EXTINT13_ENABLED | EXTINT12_ENABLED | EXTINT11_ENABLED | EXTINT10_ENABLED)

enum{
	#if(EXTINT0_ENABLED)
	EXTI_CHANNEL_0 = 0,
	#endif
	
	#if(EXTINT1_ENABLED)
	EXTI_CHANNEL_1 = 1,
	#endif
	
	#if(EXTINT2_ENABLED)
	EXTI_CHANNEL_2 = 2,
	#endif
	
	#if(EXTINT3_ENABLED)
	EXTI_CHANNEL_3 = 3,
	#endif
	
	#if(EXTINT4_ENABLED)
	EXTI_CHANNEL_4 = 4,
	#endif
	
	#if(EXTINT5_ENABLED)
	EXTI_CHANNEL_5 = 5,
	#endif
	
	#if(EXTINT6_ENABLED)
	EXTI_CHANNEL_6 = 6,
	#endif
	
	#if(EXTINT7_ENABLED)
	EXTI_CHANNEL_7 = 7,
	#endif
	
	#if(EXTINT8_ENABLED)
	EXTI_CHANNEL_8 = 8,
	#endif
	
	#if(EXTINT9_ENABLED)
	EXTI_CHANNEL_9 = 9,
	#endif
	
	#if(EXTINT10_ENABLED)
	EXTI_CHANNEL_10 = 10,
	#endif
	
	#if(EXTINT11_ENABLED)
	EXTI_CHANNEL_11 = 11,
	#endif
	
	#if(EXTINT12_ENABLED)
	EXTI_CHANNEL_12 = 12,
	#endif
	
	#if(EXTINT13_ENABLED)
	EXTI_CHANNEL_13 = 13,
	#endif
	
	#if(EXTINT14_ENABLED)
	EXTI_CHANNEL_14 = 14,
	#endif
	
	#if(EXTINT15_ENABLED)
	EXTI_CHANNEL_15 = 15,
	#endif
};

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void EXTINT_Init(uint8_t port, uint8_t pin, EXTINT_Handler initFunc);

/******************************************************************************/
#endif

#endif /* _SOURCE_HARD_EXTINT_EXTINT_H_ */
