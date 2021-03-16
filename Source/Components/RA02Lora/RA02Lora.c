/*
 * RA02Lora.h
 *
 *  Created on: March 11, 2021
 *      Author: LongHD\
 *
 * https://www.makerfabs.com/desfile/files/sx1276_77_78_79.pdf
 * 
 * THIS LIBRARY USED FOR LORA MODE
 * 
 * Start up process Sleep mode -> Stdby mode -> FSTX/ FSRx -> TX/ RX
 */

/******************************************************************************/


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "RA02Lora.h"

#if RA02_LORA_ENABLED

#include "Hard/GPIO/GPIO.h"
#include "Hard/SPI/SPI.h"



/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

SPIBase_t loraSPI = SPI_INSTANCE(RA02_LORA_SPI_USED);

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static uint8_t Lora_ReadRegister(uint8_t address);
static void Lora_WriteRegister(uint8_t address, uint8_t value);

uint8_t RA02LORA_ParsePacket(uint8_t size);
boolean RA02LORA_ReadRxData(uint8_t* data);

/******************************************************************************/

/**************** READ/ WRITE ****************/

/* Read register start page 108 in document */

/**
 * @brief  Read register
 * @param  Register address
 * @retval Register value
 */
 
static uint8_t Lora_ReadRegister(uint8_t address){
	LORA_ENABLE();
	SPI_Transfer(&loraSPI, address & 0x7F);
	uint8_t retVal = SPI_Transfer(&loraSPI, SPI_DUMPY_BYTE);
	LORA_DISABLE();
	
	return retVal;
}

/**
 * @brief  Write register
 * @param  Register address and value to write
 * @retval None
 */

static void Lora_WriteRegister(uint8_t address, uint8_t value){
	LORA_ENABLE();
	SPI_Transfer(&loraSPI, address | 0x80);
	SPI_Transfer(&loraSPI, value);
	LORA_DISABLE();
}

/**************** RESET ****************/

/**
 * @brief  Reset RA02 Lora module (Set reset pin to low)
 * @param  None
 * @retval None
 */

void RA02LORA_Reset(void){
	uint32_t delay = 1000;
	GPIO_PinLow(RA02_LORA_RESET_PORT, RA02_LORA_RESET_PIN);
	while(delay--){}
	
	delay = 10000;
	GPIO_PinHigh(RA02_LORA_RESET_PORT, RA02_LORA_RESET_PIN);
	while(delay--){}
}

/**************** FREQUENCY ****************/

/* f_RF = f_xosc . F_rf / 2 ^ 19
 * Resolution is 61.035 Hz if F(XOSC) = 32 MHz. Default value is
 * 0x6c8000 = 434 MHz. Register values must be modified only
 * when device is in SLEEP or STAND-BY mode.
 *
 * if read F_rf (from register) = 0x6C8000 -> f_RF = 32MHz * 0x6C8000 / 2 ^ 19 = 434 MHZ
 */

/**
 * @brief  Get frequency (3 registers)
 * @param  None
 * @retval Frequency
 */

uint32_t RA02LORA_GetFrequency(void){
	uint64_t frequency;
	uint32_t retVal = 0;
	
	retVal = Lora_ReadRegister(REG_FRF_MSB) << 16;
	retVal |= Lora_ReadRegister(REG_FRF_MID) << 8;
	retVal |= Lora_ReadRegister(REG_FRF_LSB);
	
	frequency = ((uint64_t) retVal * 32000000) >> 19;
	return (uint32_t) frequency;
}

/**
 * @brief  Set frequency (3 registers)
 * @param  Frequency
 * @retval None
 */

void RA02LORA_SetFrequency(uint32_t frequency){
	uint64_t setValue = ((uint64_t) frequency << 19) / 32000000;
	
	Lora_WriteRegister(REG_FRF_MSB, (setValue >> 16) & 0xFF);
	Lora_WriteRegister(REG_FRF_MID, (setValue >> 8) & 0xFF);
	Lora_WriteRegister(REG_FRF_LSB, setValue & 0xFF);
}

/**************** TX Power ****************/

/* PA configuration
 * Bit 7: Selects PA output pin
 *       0: RFO pin. Maximum power of +14 dBm
 *       1: PA_BOOST pin. Maximum power of +20 dBm
 *
 * Bit 6 - 4 : Select max output power: Pmax = 10.8 + 0.6 * MaxPower [dBm] (PA_BOOST pin)
 * Bit 3 - 0 : Pout = Pmax -(15 - OutputPower) if PaSelect = 0 (RFO pins)
 *             Pout = 17-(15 - OutputPower) if PaSelect = 1 (PA_BOOST pin)
 *
 * PA_BOOST : Optional high-power PA output, all frequency bands
 * RFO_LF   : RF output for bands 2&3
 */

/**
 * @brief  Get tx power
 * @param  None
 * @retval TX power
 */

uint8_t RA02LORA_GetTxPower(void){
	uint8_t power = Lora_ReadRegister(REG_PA_CONFIG);
	uint8_t pmax_bit = (power >> 4) & 0x07;
	
	float Pmax = 10.8 + 0.6 * pmax_bit;
	// RFO
	if(((power >> 7) & 0x01) == 0){
		power = (uint8_t) (Pmax - (15 - (power & 0x0F)));
	}
	// PA_BOOST
	else{
		power = 2 + (power & 0x0F);
	}
	return power;
}

/**
 * @brief  Set tx power
 * @param  TX power range(2, 17)
 * @retval None
 */

void RA02LORA_SetTxPower(uint8_t mode, uint8_t power){
	// RFO
	if(mode == RFO_MODE){
		// Set Pmax = 15 (bit 6-4), value = 15 - (Pmax - Pout)
		power = power & 0x0F;  // 4 bits 3-0
		power |= 0x70;         // 3 bits 6-4 -> Pmax = 10.8 + 0.6 * 7 = 15
	}
	// PA_BOOST
	else{
		if(power < 2) power = 2;
		else if(power > 17) power = 17;
		power = (power - 2) | PA_BOOST;   // bit 7 = 1
	}
	return Lora_WriteRegister(REG_PA_CONFIG, power);
}

/**************** MODE ****************/

/*
 * Transceiver modes
 * Bit 7: LongRangeMode
 *        0 -> FSK/OOK Mode (Default)
 *        1 -> LoRaTM Mode
 *        This bit can be modified only in Sleep mode. A write operation on
 *        other device modes is ignored.
 * Bit 6: AccessSharedReg
 *        0 -> Access LoRa registers page 0x0D: 0x3F (Default)
 8        1 -> Access FSK registers page (in mode LoRa) 0x0D: 0x3F
 * Bit 5 -4: Reserved
 * Bit 3: LowFrequencyModeOn
 *       0 -> High Frequency Mode (access to HF test registers)
 *       1 -> Low Frequency Mode (access to LF test registers)  (default)
 * Bit 2 - 0: Mode
 *       000 -> Sleep
 *       001 -> Standby Stdby (Default)
 *       010 -> Frequency synthesiser to Tx frequency (FSTX)
 *       011 -> Transmit (TX)
 *       100 -> Frequency synthesiser to Rx frequency (FSRX)
 *       101 -> Receive continuous mode (RX CONTINUOUS)
 *       110 -> Receive single (RX SINGLE)
 *       111 -> Channel activity detection (CAD)
 *
 * For detail, read document page 36
 */

/**
 * @brief  Get current mode
 * @param  None
 * @retval Mode
 */
 
uint8_t RA02LORA_GetMode(void){
	return Lora_ReadRegister(REG_OP_MODE);
}

/**
 * @brief  Set mode
 * @param  Mode setup, because use Lora mode -> set bit 7 = 1
 *         ex: RA02LORA_SetMode(MODE_LONG_RANGE_MODE | MODE_STDBY)
 *             RA02LORA_SetMode(MODE_SLEEP)
 * @retval None
 */

void RA02LORA_SetMode(uint8_t mode){
	Lora_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

/**************** CONFIG ****************/

/**
 * @brief  Enable/ Disable CRC check
 *         RegModemConfig2 RxPayloadCrcOn: Enable CRC generation and check on payload
 *                                         0 -> CRC disable
 *                                         1 -> CRC enable
 *         Default CRC in Explicit header mode -> Enable CRC on Tx and Rx side
 * @param  None
 * @retval Mode
 */

void RA02LORA_EnableCRC(void){
	Lora_WriteRegister(REG_MODEM_CONFIG_2, Lora_ReadRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void RA02LORA_DisableCRC(void){
	Lora_WriteRegister(REG_MODEM_CONFIG_2, Lora_ReadRegister(REG_MODEM_CONFIG_2) & 0xFB);
}

/**
 * @brief  Config LNA
 *         RegLna: LnaBoostHf: 00 -> Default LNA current
 *                            11 -> Boost on, 150% LNA current
 *
 *         RegModemConfig: AgcAutoOn  0 -> LNA gain set by register LnaGain (Default)
 *                                    1 -> LNA gain set by the internal AGC loop
 * @param  None
 * @retval Mode
 */

void RA02LORA_EnableLNABoostHF(void){
	Lora_WriteRegister(REG_LNA, Lora_ReadRegister(REG_LNA) | 0x03);
	Lora_WriteRegister(REG_MODEM_CONFIG_3, LNA_BY_INTERNAL_AGC);
}

/**************** LDO ****************/

/**
 * @brief  Set Low Data Rate Optimize
 *         RegModemConfig 3: Bit 3: 0 -> Disable
 *                                  1 ->  Enabled; mandated for when the symbol length exceeds 16ms
 * @param  None
 * @retval Mode
 */

void RA02LORA_SetLdoFlag(void){
	// Section 4.1.1.5
	uint32_t symbolDuration = 1000 / ( RA02LORA_GetSignalBandwidth() / (1L << RA02LORA_GetSpreadingFactor()) );
	
	// Section 4.1.1.6
	uint8_t ldoOn = (symbolDuration > 16) ? 1 : 0;
	uint8_t config3 = Lora_ReadRegister(REG_MODEM_CONFIG_3);
	
	// Write bit 3
	config3 = (config3 & 0xFB) | ((ldoOn << 3) & 0x04);
	Lora_WriteRegister(REG_MODEM_CONFIG_3, config3);
}

/**************** CODING RATE ****************/

/**
 * @brief  Set Coding Rate
 *         RegModemConfig 1: Bit 3-1
 *         See 4.1.1.3
 * @param  Value range [5,8] ( coding rate 4/5, 4/6, 4/7, 4/8)
 * @retval Mode
 */

void RA02LORA_SetCodingRate(uint8_t denominator){
	if (denominator < 5) {
		denominator = 5;
	}
	else if (denominator > 8) {
		denominator = 8;
	}

	uint8_t cr = denominator - 4;
	Lora_WriteRegister(REG_MODEM_CONFIG_1, (Lora_ReadRegister(REG_MODEM_CONFIG_1) & 0xF1) | (cr << 1));
}


/**************** BANDWIDTH ****************/

/**
 * @brief  Get signal bandwidth
 *         Bit 7 - 4 -> 7.8 kHz - 500 kHz (Default 125 kHz)
 *         In the lower band (169MHz), signal bandwidths 8&9 are not supported
 * @param  None
 * @retval Signal bandwidth
 */

uint32_t RA02LORA_GetSignalBandwidth(void){
	uint8_t bw = (Lora_ReadRegister(REG_MODEM_CONFIG_1) >> 4);
	switch (bw) {
		case 0: return 7.8E3;
		case 1: return 10.4E3;
		case 2: return 15.6E3;
		case 3: return 20.8E3;
		case 4: return 31.25E3;
		case 5: return 41.7E3;
		case 6: return 62.5E3;
		case 7: return 125E3;
		case 8: return 250E3;
		case 9: return 500E3;
	}
	return 0;
}

/**
 * @brief  Set signal bandwidth
 * @param  Signal bandwidth
 * @retval None
 */

void RA02LORA_SetSignalBandwidth(uint32_t sbw){
	uint8_t bw;

	if (sbw <= 7.8E3) {
		bw = 0;
	} else if (sbw <= 10.4E3) {
		bw = 1;
	} else if (sbw <= 15.6E3) {
		bw = 2;
	} else if (sbw <= 20.8E3) {
		bw = 3;
	} else if (sbw <= 31.25E3) {
		bw = 4;
	} else if (sbw <= 41.7E3) {
		bw = 5;
	} else if (sbw <= 62.5E3) {
		bw = 6;
	} else if (sbw <= 125E3) {
		bw = 7;
	} else if (sbw <= 250E3) {
		bw = 8;
	} else /*if (sbw <= 250E3)*/ {
		bw = 9;
	}
	
	// Bits 7-4
	Lora_WriteRegister(REG_MODEM_CONFIG_1, (Lora_ReadRegister(REG_MODEM_CONFIG_1) & 0x0F) | (bw << 4));
	RA02LORA_SetLdoFlag();
}

/**************** SPREADING FACTOR ****************/

/**
 * @brief  Get spreading factor (Bits 7-4, RegModemConfig)
 * @param  None
 * @retval Spreading factor
 */

uint8_t RA02LORA_GetSpreadingFactor(void){
	return Lora_ReadRegister(REG_MODEM_CONFIG_2) >> 4;
}

/**
 * @brief  Set spreading factor
 *         RegModemConfig 2: SpreadingFactor: SF rate (expressed as a base-2 logarithm)
 * @param  SF from 6 to 12
 * @retval Mode
 */

void RA02LORA_SetSpreadingFactor(uint8_t sf){
	// Range [6, 12]
	if(sf < 6){
		sf = 6;
	}
	else if (sf > 12){
		sf = 12;
	}
	
	/* RegDetectOptimize: 0xC0 | 0x03 -> SF7 to SF12
	 *                           0x05 -> SF6
	 * RegDetectionThreshold: 0x0A -> SF7 to SF12
	 *                        0x0C -> SF6
	 */
	
	if(sf == 6){
		Lora_WriteRegister(REG_DETECTION_OPTIMIZE, 0xC5);
		Lora_WriteRegister(REG_DETECTION_THRESHOLD, 0x0C);
	}
	else{
		Lora_WriteRegister(REG_DETECTION_OPTIMIZE, 0xC3);
		Lora_WriteRegister(REG_DETECTION_THRESHOLD, 0x0A);
	}
	
	// Bits 7-4
	Lora_WriteRegister(REG_MODEM_CONFIG_2, (Lora_ReadRegister(REG_MODEM_CONFIG_2) & 0x0F) | ((sf << 4) & 0xF0));
	RA02LORA_SetLdoFlag();
}

/**************** MODE ****************/

/**
 * @brief  Set/ Get header mode
 *         RegModemConfig 1: ImplicitHeaderModeOn: Bit 0: 0 -> Explicit Header mode
 *                                                        1 -> Implicit Header mode
 * @param  None
 * @retval None
 */

uint8_t RA02LORA_GetHeaderMode(void){
	return Lora_ReadRegister(REG_MODEM_CONFIG_1) & 0x01;
}

void RA02LORA_SetHeaderMode(uint8_t mode){
	uint8_t set = mode | (Lora_ReadRegister(REG_MODEM_CONFIG_1) & 0xFE);
	
	Lora_WriteRegister(REG_MODEM_CONFIG_1, set);
}

/**************** SYNCWORD ****************/

/**
 * @brief  Set Lora sync word
 * @param  Sync word (Default 0x12)
 * @retval None
 */

void RA02LORA_SetSyncWord(uint8_t sw){
	Lora_WriteRegister(REG_SYNC_WORD, sw);
}

/**************** TRANSMIT ****************/

/**
 * @brief  Check is transmitting
 * @param  None
 * @retval True if lora is transmitting
 */

boolean RA02LORA_IsTransmitting(void){
	if((Lora_ReadRegister(REG_OP_MODE) & MODE_TX) == MODE_TX){
		return TRUE;
	}
	
	// Clear IRQ
	if(Lora_ReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK){
		Lora_WriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}
	return FALSE;
}

/* RegIrqFlags 0x12: Bit 3 TxDone : FIFO Payload transmission complete interrupt: writing a 1 clears the IRQ
 */

/**
 * @brief  RA02 Lora send data
 * @param  Data and size
 * @retval None
 */

void RA02LORA_SendData(uint8_t* data, uint8_t size){
	if(RA02LORA_IsTransmitting()){
		return;
	}
	
	// Idle
	RA02LORA_SetMode(MODE_STDBY);
	
	// Set fifo address and write data to buffer
	Lora_WriteRegister(REG_FIFO_ADDR_PTR, 0);
	
	for(uint8_t i = 0; i < size; i++){
		Lora_WriteRegister(REG_FIFO, *data++);
	}
	Lora_WriteRegister(REG_PAYLOAD_LENGTH, size);
	
	// Wait trasmit completed
	RA02LORA_SetMode(MODE_TX);
	while((Lora_ReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0){ }
	
	// Clear IRQ
	Lora_WriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**************** RECEIVE ****************/

uint8_t packetIndex = 0;

/**
 * @brief  Parse data receive (must implement in a loop)
 * @param  Packet size, if > 0 -> implicit header mode
 * @retval Packet size
 */

uint8_t RA02LORA_ParsePacket(uint8_t size){
	uint8_t irqFlag = Lora_ReadRegister(REG_IRQ_FLAGS);
	uint8_t length = 0;
	
	// Config mode
	if(size > 0){
		RA02LORA_SetHeaderMode(IMPLICIT_HEADER_MODE);
		Lora_WriteRegister(REG_PAYLOAD_LENGTH, size & 0xFF);
	}
	else{
		RA02LORA_SetHeaderMode(EXPLICIT_HEADER_MODE);
	}
	
	// Clear IRQ
	Lora_WriteRegister(REG_IRQ_FLAGS, irqFlag);
	
	if((irqFlag & IRQ_RX_DONE_MASK) && ((irqFlag & IRQ_CRC_ERROR_MASK) == 0)){
		packetIndex = 0;
		
		// Read packet length
		if(RA02LORA_GetHeaderMode() == IMPLICIT_HEADER_MODE){
			length = Lora_ReadRegister(REG_PAYLOAD_LENGTH);
		}
		else{
			length = Lora_ReadRegister(REG_RX_NB_BYTES);
		}
		
		// Set FIFO address to current RX address
		Lora_WriteRegister(REG_FIFO_ADDR_PTR, Lora_ReadRegister(REG_FIFO_RX_CURRENT_ADDR));
		
		#if RX_SINGLE_MODE
		RA02LORA_SetMode(MODE_STDBY);
		#endif
	}
	
	// Set mode Rx
	#if RX_SINGLE_MODE
	else if(Lora_ReadRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
		// Reset FIFO address
		Lora_WriteRegister(REG_FIFO_ADDR_PTR, 0);
		
		// Put in single RX mode
		RA02LORA_SetMode(MODE_RX_SINGLE);
	}
	#elif RX_CONTINUOUS_MODE
	
	else if(Lora_ReadRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOS)){
		// Reset FIFO address
		Lora_WriteRegister(REG_FIFO_ADDR_PTR, 0);
		
		// Put in single RX mode
		RA02LORA_SetMode(MODE_RX_CONTINUOS);
	}
	#endif
	
	return length;
}

/**
 * @brief  Read byte received in rx fifo
 * @param  Output byte
 * @retval True if byte is available
 */

boolean RA02LORA_ReadRxData(uint8_t* data){
	if(Lora_ReadRegister(REG_RX_NB_BYTES) > packetIndex){
		packetIndex++;
		*data = Lora_ReadRegister(REG_FIFO);
		return TRUE;
	}
	return FALSE;
}

/******************************************************************************/

#define LORA_FIFO_SIZE 1024

typedef struct{
	uint8_t data[LORA_FIFO_SIZE];                    // store data
	uint16_t head;                                   // index to write data
	uint16_t tail;                                   // index to read data
	uint16_t count;                                  // number of bytes in buffer
} LoraFifo_t;

LoraFifo_t LoraFifo;

/******************************************************************************/

/**
 * @brief  RA02 Lora module initialization
 * @param  None
 * @retval None
 */

boolean RA02LORA_Init(void){
	SPI_InitAsMaster(&loraSPI);
	
	// Initializes NSS and Reset pin
	GPIO_PinMode(RA02_LORA_NSS_PORT, RA02_LORA_NSS_PIN, GPIO_Mode_Out_PP);
	GPIO_PinMode(RA02_LORA_RESET_PORT, RA02_LORA_RESET_PIN, GPIO_Mode_Out_PP);
	
	RA02LORA_Reset();
	
	// Read version
	if(Lora_ReadRegister(REG_VERSION) != RA02_LORA_VERSION){
		return FALSE;
	}
	
	// Go to sleep mode to set parameters which only config in this mode
	RA02LORA_SetMode(MODE_SLEEP);
	
	// Set base addresses
	Lora_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
	Lora_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0);
	
	// High Frequency (RFI_HF) LNA current adjustment
	RA02LORA_EnableLNABoostHF();
	RA02LORA_SetTxPower(PA_BOOST_MODE, TX_POWER_SET);   // Set Tx power in PA_BOOST mode
	RA02LORA_EnableCRC();
	
	// Spreading factor
	RA02LORA_SetSpreadingFactor(9);
	
	// Sync word
	RA02LORA_SetSyncWord(0x12);
	
	// Back to standby mode
	RA02LORA_SetMode(MODE_STDBY);
	memset(&LoraFifo, 0, sizeof(LoraFifo));
	
	return TRUE;
}

/**
 * @brief  Task for receive data
 *         Run in explicit mode
 * @param  None
 * @retval None
 */

void RA02LORA_Task(void){
	uint8_t read;
	
	if(RA02LORA_ParsePacket(0) > 0){
		while(RA02LORA_ReadRxData(&read)){
			/* Push data to buffer */
			if(LoraFifo.count < LORA_FIFO_SIZE){
				LoraFifo.data[LoraFifo.head] = read;
				LoraFifo.head = (LoraFifo.head + 1) % LORA_FIFO_SIZE;
				LoraFifo.count++;
			}
		}
	}
}

/**
 * @brief  Read data vailable
 * @param  None
 * @retval None
 */

boolean RA02LORA_Read(uint8_t* byte){
	if(LoraFifo.count > 0){
		*byte = LoraFifo.data[LoraFifo.tail];
		LoraFifo.tail = (LoraFifo.tail + 1) % LORA_FIFO_SIZE;
		LoraFifo.count--;
		return TRUE;
	}
	return FALSE;
}

/**
 * @brief  Read byte is available in rx buffer
 * @param  None
 * @retval Byte is available in rx buffer
 */

uint16_t RA02LORA_Available(void){
	return LoraFifo.count;
}

#endif

