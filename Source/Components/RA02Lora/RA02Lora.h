/*
 * RA02Lora.h
 *
 *  Created on: March 11, 2021
 *      Author: LongHD
 */

/******************************************************************************/

#ifndef _SOURCE_RA02_LORA_RA02_LORA_H_
#define _SOURCE_RA02_LORA_RA02_LORA_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if RA02_LORA_ENABLED

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define ENABLE_LOW

#ifdef ENABLE_LOW
#define LORA_ENABLE()            GPIO_PinLow(RA02_LORA_NSS_PORT, RA02_LORA_NSS_PIN)
#define LORA_DISABLE()           GPIO_PinHigh(RA02_LORA_NSS_PORT, RA02_LORA_NSS_PIN)
#else
#define LORA_ENABLE()            GPIO_PinHigh(RA02_LORA_NSS_PORT, RA02_LORA_NSS_PIN)
#define LORA_DISABLE()           GPIO_PinLow(RA02_LORA_NSS_PORT, RA02_LORA_NSS_PIN)
#endif

/* Registers address in Lora mode */

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d


/* PA config */
#define RFO                      0x00
#define PA_BOOST                 0x80

#define TX_POWER_SET             17

/* Mode */
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_FSTX                0x02
#define MODE_TX                  0x03
#define MODE_FSRX                0x04
#define MODE_RX_CONTINUOS        0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

/* LNA config */
#define LNA_BY_INTERNAL_AGC      0x04

/* IRQ masks */
#define IRQ_TX_DONE_MASK         0x08
#define IRQ_CRC_ERROR_MASK       0x20
#define IRQ_RX_DONE_MASK         0x40

#define RA02_LORA_VERSION        0x12


/* In continuous RX mode, opposite to the single RX mode, the RxTimeout interrupt will never occur
 * and the device will never go in Standby mode automatically.
 */
#define RX_SINGLE_MODE           1
#define RX_CONTINUOUS_MODE       (!RX_SINGLE_MODE)

#define EXPLICIT_HEADER_MODE     0
#define IMPLICIT_HEADER_MODE     1

#define RFO_MODE                 0
#define PA_BOOST_MODE            1

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

boolean RA02LORA_Init(void);
void RA02LORA_Task(void);
void RA02LORA_Reset(void);

boolean RA02LORA_Read(uint8_t* byte);
uint16_t RA02LORA_Available(void);

boolean RA02LORA_IsTransmitting(void);
void RA02LORA_SendData(uint8_t* data, uint8_t size);

/***************************************************/

uint8_t Lora_ReadRegister(uint8_t address);
void Lora_WriteRegister(uint8_t address, uint8_t value);

uint32_t RA02LORA_GetFrequency(void);
void RA02LORA_SetFrequency(uint32_t frequency);

uint8_t RA02LORA_GetTxPower(void);
void RA02LORA_SetTxPower(uint8_t mode, uint8_t power);

uint8_t RA02LORA_GetMode(void);
void RA02LORA_SetMode(uint8_t mode);

void RA02LORA_EnableCRC(void);
void RA02LORA_DisableCRC(void);
void RA02LORA_EnableLNABoostHF(void);
void RA02LORA_SetSyncWord(uint8_t sw);

void RA02LORA_SetLdoFlag(void);
void RA02LORA_SetCodingRate(uint8_t denominator);

uint8_t RA02LORA_GetHeaderMode(void);
void RA02LORA_SetHeaderMode(uint8_t mode);

uint32_t RA02LORA_GetSignalBandwidth(void);
void RA02LORA_SetSignalBandwidth(uint32_t sbw);

uint8_t RA02LORA_GetSpreadingFactor(void);
void RA02LORA_SetSpreadingFactor(uint8_t sf);

/******************************************************************************/

#endif

#endif /* _SOURCE_RA02_LORA_RA02_LORA_H_ */
