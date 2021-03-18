/*
 * ADC.c
 *
 *  Created on: August 17, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "ADC.h"

#if(ADC_ENABLED)

#include "stm32f10x_adc.h"
#include "Hard/GPIO/GPIO.h"

#include "stm32f10x_dma.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define ADC1_DR_Address          ((uint32_t)0x4001244C)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

#if(ADC1_ENABLED)
static volatile uint16_t ADC1ConvertedValue = 0;
#endif

#if(ADC2_ENABLED)
static volatile uint16_t ADC2ConvertedValue = 0;
#endif

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

#if(ADC1_ENABLED)

/**
 * @brief  Init adc1, use DMA
 *         See STM32F1 Manual Reference: DMA request mapping
 *         ADC1 -> DMA1 Channel 1, ADC3 -> DMA2 Channel 5
 *         ADC2 not support DMA
 * @param  None
 * @retval None
 */

void ADC1_Init(void){
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	/* ADC1 DeInit */  
    ADC_DeInit(ADC1);
	
	/* ADCCLK = PCLK2/2 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	
	/* Enable peripheral clocks */
	/* Enable DMA1 and DMA2 clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	/* GPIO configuration */
	GPIO_PinMode(ADC1_EXAMPLE_PORT, ADC1_EXAMPLE_PIN, GPIO_Mode_AIN);
	
	/* DMA1 channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADC1ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  
	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC1_EXAMPLE_CHANNEL, 1, ADC_SampleTime_28Cycles5);    
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/* Enable ADC1 reset calibration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
 * @brief  Deinit ADC1
 * @param  None
 * @retval None
 */

void ADC1_Deinit(void){
	/* ADC1 DeInit */
    ADC_DeInit(ADC1);
}

/**
 * @brief  Get ADC1 converted value
 * @param  None
 * @retval None
 */

uint16_t ADC1_GetConvertedValue(void){
	return ADC1ConvertedValue;
}


#endif


#if(ADC2_ENABLED)

/**
 * @brief  Init adc2 (not use DMA)
 * @param  None
 * @retval None
 */

void ADC2_Init(void){
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ADC DeInit */  
    ADC_DeInit(ADC2);
	
	/* ADCCLK = PCLK2/2 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	
	/* Enable ADC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	/* Configure and enable ADC interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* GPIO configuration */
	GPIO_PinMode(ADC2_EXAMPLE_PORT, ADC2_EXAMPLE_PIN, GPIO_Mode_AIN);
	
	// ADC configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);
	
	/* ADC regular channels configuration */ 
	ADC_RegularChannelConfig(ADC2, ADC2_EXAMPLE_CHANNEL, 1, ADC_SampleTime_28Cycles5);
	
	/* Enable ADC2 EOC interrupt */
	ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
	
	/* Enable ADC */
	ADC_Cmd(ADC2, ENABLE);
	
	/* Enable ADC reset calibration register */   
	ADC_ResetCalibration(ADC2);
	/* Check the end of ADC reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC2));

	/* Start ADC calibration */
	ADC_StartCalibration(ADC2);
	/* Check the end of ADC calibration */
	while(ADC_GetCalibrationStatus(ADC2));
	
	/* Start ADC Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}

/**
 * @brief  This function handles ADC1 and ADC2 global interrupts requests
 * @param  None
 * @retval None
 */

void ADC1_2_IRQHandler(void){
	/* Get injected ADC converted value */
	ADC2ConvertedValue = ADC_GetConversionValue(ADC2);
}

/**
 * @brief  Deinit ADC2
 * @param  None
 * @retval None
 */

void ADC2_Deinit(void){
	/* ADC2 DeInit */
    ADC_DeInit(ADC2);
}

/**
 * @brief  Get ADC2 converted value
 * @param  None
 * @retval None
 */

uint16_t ADC2_GetConvertedValue(void){
	return ADC2ConvertedValue;
}


#endif

#endif
