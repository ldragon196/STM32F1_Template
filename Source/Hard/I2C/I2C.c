/*
 * I2C.c
 *
 *  Created on: August 14, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Hard/GPIO/GPIO.h"
#include "I2C.h"

#if(I2C_ENABLED)

#include "stm32f10x_i2c.h"
#include "Hard/GPIO/GPIO.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define I2C_CHECK_TIMEOUT(a)                         do{if( ((a)--) == 0) goto I2C_ERROR;} while(0)
#define CHECK_NOT_NULL(a)                            do{if( (a) == 0) return;} while(0)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static void I2C_Start(I2CBase_t* i2cConfig);
static void I2C_Stop(I2CBase_t* i2cConfig);

/******************************************************************************/

/**
 * @func   I2C_InitAsMaster
 * @brief  Init I2C as master mode
 * @param  I2C number
 * @retval None
 */

void I2C_InitAsMaster(I2CBase_t* i2cConfig){
	I2C_InitTypeDef I2C_InitStructure;
	
	// Enable clock
	RCC_APB1PeriphClockCmd(i2cConfig->RCCClock, ENABLE);
	
	// Config gpio as *open drain*
	GPIO_PinMode(i2cConfig->SCLPort, i2cConfig->SCLPin, GPIO_Mode_AF_OD);
	GPIO_PinMode(i2cConfig->SDAPort, i2cConfig->SDAPin, GPIO_Mode_AF_OD);
	
	// Config I2C
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_ACK_ENABLED;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_CLOCK_SPEED;
	
	I2C_DeInit(i2cConfig->I2C);
	I2C_Init(i2cConfig->I2C, &I2C_InitStructure);
	
	/* I2C enable */
	I2C_Cmd(i2cConfig->I2C, ENABLE);
}

/**
 * @func   I2C_Start
 * @brief  I2C start
 * @param  None
 * @retval None
 */

static void I2C_Start(I2CBase_t* i2cConfig){
	uint32_t timeout = I2C_TOTAL_TIMEOUT;
	
	// Wait until I2Cx is not busy anymore
	while(I2C_GetFlagStatus(i2cConfig->I2C, I2C_FLAG_BUSY)){I2C_CHECK_TIMEOUT(timeout);}
	
	/* Wait for I2C EV5. It means that the start condition has been correctly released 
	 * on the I2C bus (the bus is free, no other devices is communicating))
	*/
	I2C_GenerateSTART(i2cConfig->I2C, ENABLE);
	while(!I2C_CheckEvent(i2cConfig->I2C, I2C_EVENT_MASTER_MODE_SELECT)){I2C_CHECK_TIMEOUT(timeout);}
	
I2C_ERROR:
	return;
}

/**
 * @func   I2C_Stop
 * @brief  I2C stop
 * @param  None
 * @retval None
 */

static void I2C_Stop(I2CBase_t* i2cConfig){
	uint32_t timeout = I2C_TOTAL_TIMEOUT;
	
	// Generate I2C stop condition
	I2C_GenerateSTOP(i2cConfig->I2C, ENABLE);
	while(!I2C_GetFlagStatus(i2cConfig->I2C, I2C_FLAG_STOPF)){I2C_CHECK_TIMEOUT(timeout);}
	
I2C_ERROR:
	return;
}


/**
 * @func   I2C_WriteData
 * @brief  Write n bytes
 * @param  I2C number
 *         slave address
 *         register address
 *         data and size
 * @retval None
 */

void I2C_WriteData(I2CBase_t* i2cConfig, uint8_t slaveAdd, uint8_t *data, uint8_t size){
	uint32_t timeout = I2C_TOTAL_TIMEOUT;
	
	CHECK_NOT_NULL(size);
	CHECK_NOT_NULL(data);
	
	// Start
	I2C_Start(i2cConfig);
	
	// Set direction
	I2C_Send7bitAddress(i2cConfig->I2C, slaveAdd, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(i2cConfig->I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){I2C_CHECK_TIMEOUT(timeout);}
	
	// Transmit
	for(uint8_t i = 0; i < size; i++){
		I2C_SendData(i2cConfig->I2C, data[i]);
		while(!I2C_GetFlagStatus(i2cConfig->I2C, I2C_FLAG_BTF)){I2C_CHECK_TIMEOUT(timeout);}
	}
	
	// Stop
	I2C_Stop(i2cConfig);
	
I2C_ERROR:
	return;
}


/**
 * @func   I2C_ReadRegMulti
 * @brief  Read register
 * @param  I2C number
 *         slave address
 *         register address
 *         data out and size
 * @retval None
 */

void I2C_ReadData(I2CBase_t* i2cConfig, uint8_t slaveAdd, uint8_t *data, uint8_t size){
	uint32_t timeout = I2C_TOTAL_TIMEOUT;
	
	CHECK_NOT_NULL(size);
	CHECK_NOT_NULL(data);
	
	// Start
	I2C_Start(i2cConfig);
	
	// Set direction
	I2C_Send7bitAddress(i2cConfig->I2C, slaveAdd, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(i2cConfig->I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){I2C_CHECK_TIMEOUT(timeout);}
	
	for(uint8_t i = 0; i < size; i++){
		if(i == (size - 1) ){
			// Disable ACK of received data
			I2C_AcknowledgeConfig(i2cConfig->I2C, DISABLE);
			// Wait for I2C EV7
			// It means that the data has been received in I2C data register
			while (!I2C_CheckEvent(i2cConfig->I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
			
			// Read byte from I2C data register
			data[i] = I2C_ReceiveData(i2cConfig->I2C);
		}
		else{
			// Disable ACK of received data
			I2C_AcknowledgeConfig(i2cConfig->I2C, ENABLE);
			// Wait for I2C EV7
			// It means that the data has been received in I2C data register
			while (!I2C_CheckEvent(i2cConfig->I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
			
			// Read byte from I2C data register
			data[i] = I2C_ReceiveData(i2cConfig->I2C);
		}
	}
	
	// Stop
	I2C_Stop(i2cConfig);
	return;
	
I2C_ERROR:
	memset(data, 0, size);
	return;
}



#endif
