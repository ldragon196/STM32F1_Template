/*
 *  WDG.h
 *
 *  Created on: August 13, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_WDG_WDG_H_
#define _SOURCE_HARD_WDG_WDG_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(IWDG_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/


/* See Table 96. Min/max IWDG timeout period at 40 kHz (LSI)
 * These timings are given for a 40 kHz clock but
 * the microcontroller?sinternal RC frequency can vary from 30 to 60 kHz
 * IWDG_RELOAD_VALUE must in 0 - 0x0FFF
 * wdg_timeout = (IWDG_RELOAD_VALUE * prescaler) / f_LSI
 */
 
 // ~ 0.9s
#define IWDG_PRESCALER                       IWDG_Prescaler_128
#define IWDG_MAX_TIMEOUT                     10000               // in prescaler 128
#define IWDG_LSI_FREQUENCE                   40000

#if(WDG_TIMEOUT_MS>IWDG_MAX_TIMEOUT)
	#undef WDG_TIMEOUT_MS
	#define WDG_TIMEOUT_MS IWDG_MAX_TIMEOUT
#endif

#define IWDG_RELOAD_VALUE                    ((WDG_TIMEOUT_MS * IWDG_LSI_FREQUENCE) / ((4 << IWDG_PRESCALER) * 1000))

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void IWDG_Init(void);
void IWDG_ResetWatchdog(void);

/******************************************************************************/

#endif

#endif /* _SOURCE_HARD_WDG_WDG_H_ */
