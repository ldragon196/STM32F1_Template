/*
 * SPI.c
 *
 *  Created on: August 17, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "SPI.h"

#if(SPI_ENABLED)

#include "stm32f10x_spi.h"
#include "Hard/GPIO/GPIO.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

#if(SPI1_SLAVE_MODE_ENABLED|SPI1_SLAVE_MODE_ENABLED)
SPI_ReceiveHandler handlerSPISlaveReceive = NULL;
#endif

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/**
 * @func   SPI_Init
 * @brief  Init spi as master mode
 * @param  SPI number
 * @retval None
 */

void SPI_InitAsMaster(SPIBase_t* spiConfig){
	SPI_InitTypeDef SPI_InitStructure;
	
	switch(spiConfig->Id){
		#if(SPI1_ENABLED)
		case SPI_ID_1:
			// Enable clock
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			
			// Config gpio
			GPIO_PinMode(COM_SPI1_SCK_PORT, COM_SPI1_SCK_PIN, GPIO_Mode_AF_PP);
			GPIO_PinMode(COM_SPI1_MOSI_PORT, COM_SPI1_MOSI_PIN, GPIO_Mode_AF_PP);
			GPIO_PinMode(COM_SPI1_MISO_PORT, COM_SPI1_MISO_PIN, GPIO_Mode_IN_FLOATING);
			break;
		#endif
		
		#if(SPI2_ENABLED)
		case SPI_ID_2:
			// Enable clock
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			
			// Config gpio
			GPIO_PinMode(COM_SPI2_SCK_PORT, COM_SPI2_SCK_PIN, GPIO_Mode_AF_PP);
			GPIO_PinMode(COM_SPI2_MOSI_PORT, COM_SPI2_MOSI_PIN, GPIO_Mode_AF_PP);
			GPIO_PinMode(COM_SPI2_MISO_PORT, COM_SPI2_MISO_PIN, GPIO_Mode_IN_FLOATING);
			break;
		#endif
		
		default:
			return;
	}
	
	/* SPI_MASTER configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(spiConfig->SPI, &SPI_InitStructure);
	
	/* Enable SPI_MASTER */
	SPI_Cmd(spiConfig->SPI, ENABLE);
}

/**
 * @func   SPI_Transfer
 * @brief  SPI transfer
 * @param  SPI number, Tx data
 * @retval Rx Data
 */

uint8_t SPI_Transfer(SPIBase_t* spi, uint8_t byte){
	/* Wait for SPI Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(spi->SPI, SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send SPI data */
	SPI_I2S_SendData(spi->SPI, byte);
	
	/* Wait for SPI data reception */
	while (SPI_I2S_GetFlagStatus(spi->SPI, SPI_I2S_FLAG_RXNE) == RESET ){}
	
	/* Wait until SPI is not busy anymore */
    while (SPI_I2S_GetFlagStatus(spi->SPI, SPI_I2S_FLAG_BSY) != RESET);	
		
	/* Read SPIy received data */
    return SPI_I2S_ReceiveData(spi->SPI);
}

/******************* SPI SLAVE MODE *******************/

#if(SPI1_SLAVE_MODE_ENABLED|SPI1_SLAVE_MODE_ENABLED)
/**
 * @func   SPI_Init
 * @brief  Init spi as slave mode
 * @param  SPI number
 * @retval None
 */

void SPI_InitAsSlave(SPIBase_t* spiConfig, SPI_ReceiveHandler initFunc){
	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	switch(spiConfig->Id){
		#if(SPI1_ENABLED)
		case SPI_ID_1:
			// Enable clock
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			
			// Config gpio
			GPIO_PinMode(COM_SPI1_SCK_PORT, COM_SPI1_SCK_PIN, GPIO_Mode_IN_FLOATING);
			GPIO_PinMode(COM_SPI1_MOSI_PORT, COM_SPI1_MOSI_PIN, GPIO_Mode_IN_FLOATING);
			GPIO_PinMode(COM_SPI1_MISO_PORT, COM_SPI1_MISO_PIN, GPIO_Mode_AF_PP);
		
			NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
			break;
		#endif
		
		#if(SPI2_ENABLED)
		case SPI_ID_2:
			// Enable clock
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			
			// Config gpio
			GPIO_PinMode(COM_SPI2_SCK_PORT, COM_SPI2_SCK_PIN, GPIO_Mode_IN_FLOATING);
			GPIO_PinMode(COM_SPI2_MOSI_PORT, COM_SPI2_MOSI_PIN, GPIO_Mode_IN_FLOATING);
			GPIO_PinMode(COM_SPI2_MISO_PORT, COM_SPI2_MISO_PIN, GPIO_Mode_AF_PP);
		
			NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
			break;
		#endif
		
		default:
			return;
	}
	
	/* Configure and enable SPI_SLAVE interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SPI_SLAVE_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* SPI_SLAVE configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(spiConfig->SPI, &SPI_InitStructure);
	
	/* Enable RXNE interrupt */
	SPI_I2S_ITConfig(spiConfig->SPI, SPI_I2S_IT_RXNE, ENABLE);
	
	/* Enable SPI_SLAVE */
	SPI_Cmd(spiConfig->SPI, ENABLE);
	
	#if(SPI1_SLAVE_MODE_ENABLED|SPI1_SLAVE_MODE_ENABLED)
	handlerSPISlaveReceive = initFunc;
	#endif
}

/**
 * @func   SPI_SlaveResponse
 * @brief  SPI slave transfer response
 * @param  SPI number, Tx data
 * @retval None
 */

void SPI_SlaveResponse(SPIBase_t* spi, uint8_t byte){
	/* Wait for SPI Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(spi->SPI, SPI_I2S_FLAG_TXE) == RESET);
	/* Send SPI data */
	SPI_I2S_SendData(spi->SPI, byte);
	
	/* Wait until SPI is not busy anymore */
    while (SPI_I2S_GetFlagStatus(spi->SPI, SPI_I2S_FLAG_BSY) != RESET);
}


#if(SPI1_ENABLED)
/* This function handles SPI1 global interrupt request */
void SPI1_IRQHandler(void){
	uint8_t read = SPI_I2S_ReceiveData(SPI1);
	if(handlerSPISlaveReceive != NULL){
		handlerSPISlaveReceive(SPI_ID_1, read);
	}
}
#endif


#if(SPI2_ENABLED)
/* This function handles SPI2 global interrupt request */
void SPI2_IRQHandler(void){
	uint8_t read = SPI_I2S_ReceiveData(SPI2);
	if(handlerSPISlaveReceive != NULL){
		handlerSPISlaveReceive(SPI_ID_2, read);
	}
}
#endif
#endif

/******************* TEST *******************/

//SPIBase_t SPIMaster = SPI_INSTANCE(2);
//SPIBase_t SPISlave = SPI_INSTANCE(1);

//void MAIN_HandlerSPI(uint8_t spiId, uint8_t data){
//	if(spiId == SPISlave.Id){
//		SPI_SlaveResponse(&SPISlave, data + 1);
//	}
//}

//void init(void){
//	SPI_InitAsMaster(&SPIMaster);
//	SPI_InitAsSlave(&SPISlave, MAIN_HandlerSPI);
//}

//void loop(){
//	static uint8_t data;
//	
//	SPI_Transfer(&SPIMaster, data);
//	data = SPI_Transfer(&SPIMaster, SPI_DUMPY_BYTE);
//	DEBUG_PRINTLN("%02X", data);
//	
//	delay(1000);
//}


#endif
