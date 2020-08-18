/*
 * ADC.h
 *
 *  Created on: August 17, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_ADC_ADC_H_
#define _SOURCE_HARD_ADC_ADC_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(ADC_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define ADC1_EXAMPLE_CHANNEL                              ADC_Channel_0
#define ADC1_EXAMPLE_PORT                                 ADC1_CHANNEL0_PORT
#define ADC1_EXAMPLE_PIN                                  ADC1_CHANNEL0_PIN

#define ADC2_EXAMPLE_CHANNEL                              ADC_Channel_1
#define ADC2_EXAMPLE_PORT                                 ADC2_CHANNEL1_PORT
#define ADC2_EXAMPLE_PIN                                  ADC2_CHANNEL1_PIN


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

#if(ADC1_ENABLED)
void ADC1_Init(void);
void ADC1_Deinit(void);
uint16_t ADC1_GetConvertedValue(void);
#endif

#if(ADC2_ENABLED)
void ADC2_Init(void);
void ADC2_Deinit(void);
uint16_t ADC2_GetConvertedValue(void);
#endif

/******************************************************************************/
#endif

#endif /* _SOURCE_HARD_ADC_ADC_H_ */
