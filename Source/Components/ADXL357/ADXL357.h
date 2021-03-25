/*
 * ADXL357.h
 *
 *  Created on: March 22, 2021
 *      Author: LongHD
 */

/******************************************************************************/

#ifndef _SOURCE_ADXL357_ADXL357_H_
#define _SOURCE_ADXL357_ADXL357_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if ADXL357_ENABLED

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define ADXL357_IN_RANGE_40G

#define ADXL357_ANALOG_DEVIDE_ID  0xAD

// Memory register addresses
#define ADXL357_DEVICE_ID         0x00
#define ADXL357_REVISION_ID       0x01

#define STATUS_REG                0x04
#define TEMP2                     0x06
#define TEMP1                     0x07
#define XDATA3                    0x08
#define XDATA2                    0x09
#define XDATA1                    0x0A
#define YDATA3                    0x0B
#define YDATA2                    0x0C
#define YDATA1                    0x0D
#define ZDATA3                    0x0E
#define ZDATA2                    0x0F
#define ZDATA1                    0x10
#define RANGE                     0x2C
#define POWER_CTL                 0x2D
#define ADXL357_RESET             0x2F

// Operations
#define ADXL357_READ_BYTE         0x01
#define ADXL357_WRITE_BYTE        0x00

// Set sensor range within RANGE registe
#define RANGE_20G                 0x01
#define RANGE_40G                 0x02
#define RANGE_80G                 0x03

#define ADXL357_10G_SCALE         51200.0f
#define ADXL357_20G_SCALE         25600.0f
#define ADXL357_40G_SCALE         12800.0f

// Temperature parameters
#define ADXL357_TEMP_BIAS       (float)1855.0      // Accelerometer temperature bias(in ADC codes) at 25 Deg C
#define ADXL357_TEMP_SLOPE      (float)-9.05       // Accelerometer temperature change from datasheet (LSB/degC)

#define ADXL357_STANDBY_MODE      0x01
#define ADXL357_MEASURE_MODE      0x00

#define SENSOR_ADDRESS_SIZE       11

typedef struct {
	float x;
	float y;
	float z;
	float t;
} ADXL357_Sensor_t;


#define ADXL357_ENABLE()          GPIO_PinLow(ADXL357_NSS_PORT, ADXL357_NSS_PIN)
#define ADXL357_DISABLE()         GPIO_PinHigh(ADXL357_NSS_PORT, ADXL357_NSS_PIN)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

boolean ADXL357_Init(void);
void ADXL357_Reset(void);
void ADXL357_SetMode(uint8_t mode);
boolean ADXL357_Measure(ADXL357_Sensor_t* sensor);

/******************************************************************************/
#endif

#endif /* _SOURCE_ADXL357_ADXL357_H_ */
