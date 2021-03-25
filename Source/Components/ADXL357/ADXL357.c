/*
 * ADXL357.c
 *
 *  Created on: March 22, 2021
 *      Author: LongHD
 *
 * See SPI PROTOCOL in datasheet https://www.analog.com/media/en/technical-documentation/data-sheets/adxl356-357.pdf
 * Pin ADXL357 -> MCU (Page 22)
 *     NSS -> NSS
 *     SCLK -> SCLK
 *     MISO -> MISO, MOSI -> MOSI
 *     VDDIO -> 3.3V
 */

/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "ADXL357.h"

#if ADXL357_ENABLED

#include "Hard/GPIO/GPIO.h"
#include "Hard/SPI/SPI.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

SPIBase_t adxlSPI = SPI_INSTANCE(ADXL357_SPI_USED);

float ADXL357Scale = ADXL357_20G_SCALE;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void ADXL357_WriteRegister(uint8_t reg, uint8_t value);
uint8_t ADXL357_ReadRegister(uint8_t reg);
void ADXL357_ReadMultiple(uint8_t reg, uint8_t *data, uint8_t size);

int32_t ADXL357_DataConversion(uint8_t* data);

/******************************************************************************/

/**
 * @brief  Write register
 * @param  Register address and value to write
 * @retval None
 */
 
void ADXL357_WriteRegister(uint8_t reg, uint8_t value){
	ADXL357_ENABLE();
	SPI_Transfer(&adxlSPI, (reg << 1) | ADXL357_WRITE_BYTE);
	SPI_Transfer(&adxlSPI, value);
	ADXL357_DISABLE();
}

/**
 * @brief  Read register
 * @param  Register address
 * @retval Register value
 */

uint8_t ADXL357_ReadRegister(uint8_t reg){
	uint8_t retVal;
	
	ADXL357_ENABLE();
	SPI_Transfer(&adxlSPI, (reg << 1) | ADXL357_READ_BYTE);
	retVal = SPI_Transfer(&adxlSPI, SPI_DUMPY_BYTE);
	ADXL357_DISABLE();
	
	return retVal;
}

/**
 * @brief  Read multiple register
 * @param  Start register address, data output and size of data
 * @retval None
 */

void ADXL357_ReadMultiple(uint8_t reg, uint8_t *data, uint8_t size){
	ADXL357_ENABLE();
	SPI_Transfer(&adxlSPI, (reg << 1) | ADXL357_READ_BYTE);
	for(uint8_t i = 0; i < size; i++){
		data[i] = SPI_Transfer(&adxlSPI, SPI_DUMPY_BYTE);
	}
	ADXL357_DISABLE();
}

/**
 * @brief  Convert read data to x,y,z value in int32_t type
 * @param  Data
 * @retval Value
 */

int32_t ADXL357_DataConversion(uint8_t* data){
	int32_t retVal = ((uint32_t) data[2] >> 4) | ((uint32_t) data[1] << 4) | ((uint32_t) data[0] << 12);
	
	if (retVal & 0x80000) {
        retVal = (retVal & 0x7FFFF) - 0x80000;
    }
	return retVal;
}

/**
 * @brief  Write Code 0x52 to reset the device, similar to a power-on reset (POR)
 * @param  Data
 * @retval Value
 */

void ADXL357_Reset(void){
	ADXL357_WriteRegister(ADXL357_RESET, 0x52);
}

/******************************************************************************/

/**
 * @brief  Set ADXL357 mode (measure or standby mode) by set bit 0
 *         1. Standby mode. In standby mode, the device is in a low power state, and the
 *         temperature and acceleration datapaths are not operating. In addition, digital
 *         functions, including FIFO pointers, reset. Changes to the configuration setting of the
 *         device must be made when standby = 1. An exception is a high-pass filter that can be
 *         changed when the device is operating.
 *         0. Measurement mode
 * @param  None
 * @retval None
 */

void ADXL357_SetMode(uint8_t mode){
	uint8_t power = ADXL357_ReadRegister(POWER_CTL);
	power = (power & 0xFE) | mode;
	ADXL357_WriteRegister(POWER_CTL, power);
}

/**
 * @brief  ADXL357 measure x, y, z
 * @param  Output data
 * @retval None
 */

boolean ADXL357_Measure(ADXL357_Sensor_t* sensor){
	uint8_t data[SENSOR_ADDRESS_SIZE];
	int32_t x, y, z, t;
	
	// Check data ready
	if(!(ADXL357_ReadRegister(STATUS_REG) & 0x01)){
		return FALSE;
	}
	ADXL357_ReadMultiple(TEMP2, data, SENSOR_ADDRESS_SIZE);
	
	x = ADXL357_DataConversion(&data[2]);
	y = ADXL357_DataConversion(&data[5]);
	z = ADXL357_DataConversion(&data[8]);
	t = (data[0] << 8) | data[1];
	
	sensor->t = ((((float)t - ADXL357_TEMP_BIAS)) / ADXL357_TEMP_SLOPE) + 25.0;
	sensor->x = ((float) x / ADXL357Scale);
	sensor->y = ((float) y / ADXL357Scale);
	sensor->z = ((float) z / ADXL357Scale);
	
	return TRUE;
}

/**
 * @brief  Set ADXL357 in standa mode
 * @param  None
 * @retval None
 */

boolean ADXL357_Init(void){
	SPI_InitAsMaster(&adxlSPI);
	
	// Initializes NSS
	GPIO_PinMode(ADXL357_NSS_PORT, ADXL357_NSS_PIN, GPIO_Mode_Out_PP);
	
	// Configure ADXL357, see page 39
	ADXL357_SetMode(ADXL357_STANDBY_MODE);
	
	#ifdef ADXL357_IN_RANGE_20G
		ADXL357_WriteRegister(RANGE, RANGE_20G);
		ADXL357Scale = ADXL357_20G_SCALE;
	#endif
	
	#ifdef ADXL357_IN_RANGE_40G
		ADXL357_WriteRegister(RANGE, RANGE_40G);
		ADXL357Scale = ADXL357_40G_SCALE;
	#endif
	
	#ifdef ADXL357_IN_RANGE_8G
		ADXL357_WriteRegister(RANGE, RANGE_80G);
		ADXL357Scale = ADXL357_80G_SCALE;
	#endif
	
	ADXL357_SetMode(ADXL357_MEASURE_MODE);
	
	if(ADXL357_ANALOG_DEVIDE_ID == ADXL357_ReadRegister(ADXL357_DEVICE_ID)){
		return TRUE;
	}
	
	return FALSE;
}
























#endif
