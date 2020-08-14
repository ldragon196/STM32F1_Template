/*
 * Uart.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Hard/GPIO/GPIO.h"
#include "Uart.h"

#if(UART_ENABLED)

#include "stm32f10x_usart.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

USART_TypeDef* const COM_UART_BASE[UART_MAX_COUNT] = COM_UART_CONFIG;
static UartRxFifo_t UART_RxBuffer[UART_COUNT];

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static void UART_HandlerIRQ(uint8_t uartNum);

/******************************************************************************/

void UART_Init(uint8_t uartNum){
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	FunctionalState rxIRQEnable;
	
	rxIRQEnable = DISABLE;
	switch(uartNum){
#if(UART1_ENABLED)
		case UART_ID_1:
			// Enable uart clock
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		
			// Config GPIO
			GPIO_PinMode(COM_USART1_TX_PORT, COM_USART1_TX_PIN, GPIO_Mode_AF_PP);
			GPIO_PinMode(COM_USART1_RX_PORT, COM_USART1_RX_PIN, GPIO_Mode_IN_FLOATING);
			
			// Baudrate
			USART_InitStructure.USART_BaudRate = UART1_DEFAULT_CONFIG_BAUDRATE;
		
			/* Configures the nested vectored interrupt for rx, tx */
			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			
			#if(UART1_CONFIG_RX_IRQ_ENABLED)
			rxIRQEnable = ENABLE;
			#endif
		
			break;
#endif
		
#if(UART2_ENABLED)
		case UART_ID_2:
			// Enable uart clock
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		
			// Config GPIO
			GPIO_PinMode(COM_USART2_TX_PORT, COM_USART2_TX_PIN, GPIO_Mode_AF_PP);
			GPIO_PinMode(COM_USART2_RX_PORT, COM_USART2_RX_PIN, GPIO_Mode_IN_FLOATING);
			
			// Baudrate
			USART_InitStructure.USART_BaudRate = UART2_DEFAULT_CONFIG_BAUDRATE;
			
			/* Configures the nested vectored interrupt for rx, tx */
			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			
			#if(UART2_CONFIG_RX_IRQ_ENABLED)
			rxIRQEnable = ENABLE;
			#endif
			break;
#endif
		
#if(UART3_ENABLED)
		case UART_ID_3:
			// Enable uart clock
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		
			// Config GPIO
			GPIO_PinMode(COM_USART3_TX_PORT, COM_USART3_TX_PIN, GPIO_Mode_AF_PP);
			GPIO_PinMode(COM_USART3_RX_PORT, COM_USART3_RX_PIN, GPIO_Mode_IN_FLOATING);
			
			// Baudrate
			USART_InitStructure.USART_BaudRate = UART3_DEFAULT_CONFIG_BAUDRATE;
		
			/* Configures the nested vectored interrupt for rx, tx */
			NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
			
			#if(UART3_CONFIG_RX_IRQ_ENABLED)
			rxIRQEnable = ENABLE;
			#endif
			break;
#endif	
		default:
			// Invalid com_usart
			return;
	}
	
		
	/* Configure the NVIC Preemption Priority Bits */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART_CONFIG_RX_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_InitStructure.USART_WordLength = UART_DEFAULT_CONFIG_WORDLENGTH;
	USART_InitStructure.USART_StopBits = UART_DEFAULT_CONFIG_STOPBITS;
	USART_InitStructure.USART_Parity = UART_DEFAULT_CONFIG_PARITY;
	USART_InitStructure.USART_HardwareFlowControl = UART_DEFAULT_CONFIG_HWFC;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	/* Configure USART */
	USART_Init(COM_UART_BASE[uartNum], &USART_InitStructure);
	
	/* Enable USART Receive interrupt and disable Transmit interrupt */
	USART_ITConfig(COM_UART_BASE[uartNum], USART_IT_RXNE, rxIRQEnable);
	USART_ITConfig(COM_UART_BASE[uartNum], USART_IT_TXE, DISABLE);
	
	/* Enable the USARTy */
	USART_Cmd(COM_UART_BASE[uartNum], ENABLE);
	
	// Memset buffer
	memset(&UART_RxBuffer, 0, sizeof(UART_RxBuffer));
}

/**
 * @func   USARTx_IRQHandler
 * @brief  Handle uart irq
 * @param  None
 * @retval None
 */

#if(UART1_ENABLED)
void USART1_IRQHandler(void){
	UART_HandlerIRQ(UART_ID_1);
}
#endif

#if(UART2_ENABLED)
void USART2_IRQHandler(void){
	UART_HandlerIRQ(UART_ID_2);
}
#endif

#if(UART3_ENABLED)
void USART3_IRQHandler(void){
	UART_HandlerIRQ(UART_ID_3);
}
#endif


static void UART_HandlerIRQ(uint8_t uartNum){
	if (USART_GetITStatus(COM_UART_BASE[uartNum], USART_IT_RXNE) != RESET){
		if(UART_RxBuffer[uartNum].count < UART_DEFAULT_CONFIG_RX_FIFO_SIZE){
			/* Push data to buffer */
			UART_RxBuffer[uartNum].data[UART_RxBuffer[uartNum].head] = USART_ReceiveData(COM_UART_BASE[uartNum]);
			UART_RxBuffer[uartNum].head = (UART_RxBuffer[uartNum].head + 1) % UART_DEFAULT_CONFIG_RX_FIFO_SIZE;
			UART_RxBuffer[uartNum].count++;
		}
		USART_ClearITPendingBit(COM_UART_BASE[uartNum], USART_IT_RXNE);
	}
}

/******************************************************************************/

/**
 * @function :  UART_WriteByte
 * @brief    :  Write byte via usart com
 * @parameter:  Byte
 * @retVal   :  None
 */

void UART_WriteByte(uint8_t uartNum, uint8_t byte){
	USART_SendData(COM_UART_BASE[uartNum], byte);
	while(USART_GetFlagStatus(COM_UART_BASE[uartNum], USART_FLAG_TXE) == RESET) {}
}

/**
 * @function :  UART_WriteData
 * @brief    :  Write data via usart com
 * @parameter:  data
 *              length: length of data
 * @retVal   :  None
 */

void UART_WriteData(uint8_t uartNum, const uint8_t* data, uint16_t length){
	while(length--){
		USART_SendData(COM_UART_BASE[uartNum], *data++);
		while(USART_GetFlagStatus(COM_UART_BASE[uartNum], USART_FLAG_TXE) == RESET) {}
	}
}

/**
 * @function :  UART_ReadAvailableByte
 * @brief    :  Read number of bytes pending in app_usart rx buffer
 * @parameter:  None
 * @retVal   :  Number of bytes
 */

uint16_t UART_ReadAvailableByte(uint8_t uartNum){
	return UART_RxBuffer[uartNum].count;
}

/**
 * @function :  UART_ReadByte
 * @brief    :  Read byte in buffer
 * @parameter:  byte: destination byte
 * @retVal   :  true if success. Otherwise, buffer is empty
 */

uint8_t UART_ReadByte(uint8_t uartNum, uint8_t* byte){
	if(UART_RxBuffer[uartNum].count > 0){
		*byte = UART_RxBuffer[uartNum].data[UART_RxBuffer[uartNum].tail];
		UART_RxBuffer[uartNum].tail = (UART_RxBuffer[uartNum].tail + 1) % UART_DEFAULT_CONFIG_RX_FIFO_SIZE;
		UART_RxBuffer[uartNum].count--;
		return 1;
	}
	return 0;
}

/**
 * @function :  UART_Flush
 * @brief    :  Flush received data buffer
 * @parameter:  uart number
 * @retVal   :  None
 */

void UART_Flush(uint8_t uartNum){
	UART_RxBuffer[uartNum].count = 0;
	UART_RxBuffer[uartNum].head = 0;
	UART_RxBuffer[uartNum].tail = 0;
}


#endif
