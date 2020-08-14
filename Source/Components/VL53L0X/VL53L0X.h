/*
 * VL53L0X.h
 *
 *  Created on: June 17, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_COMPONENTS_VL53L0X_VL53L0X_H_
#define _SOURCE_COMPONENTS_VL53L0X_VL53L0X_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h" 

#if(VL53L0X_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define IO_2V8

#define VL53L0X_ADDRESS                         0x52
#define VL53L0X_MODEL_ID                        0xEE
#define VL53L0X_REVISION_ID                     0x10

#define VL53L0X_TIMEOUT                         100

// register addresses from API vl53l0x
enum {
	SYSRANGE_START                              = 0x00,

	SYSTEM_THRESH_HIGH                          = 0x0C,
	SYSTEM_THRESH_LOW                           = 0x0E,

	SYSTEM_SEQUENCE_CONFIG                      = 0x01,
	SYSTEM_RANGE_CONFIG                         = 0x09,
	SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

	SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

	GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

	SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

	RESULT_INTERRUPT_STATUS                     = 0x13,
	RESULT_RANGE_STATUS                         = 0x14,

	RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
	RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
	RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
	RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
	RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

	ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

	I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

	MSRC_CONFIG_CONTROL                         = 0x60,

	PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
	PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
	PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
	PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

	FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
	FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
	FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
	FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

	PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
	PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

	PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

	SYSTEM_HISTOGRAM_BIN                        = 0x81,
	HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
	HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

	FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
	CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

	MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

	SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
	IDENTIFICATION_MODEL_ID                     = 0xC0,
	IDENTIFICATION_REVISION_ID                  = 0xC2,

	OSC_CALIBRATE_VAL                           = 0xF8,

	GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

	GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
	DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
	DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
	POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

	VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

	ALGO_PHASECAL_LIM                           = 0x30,
	ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

enum{
	VL53L0X_ERROR = 0,
	VL53L0X_OK,
};

typedef struct{
	uint8_t address;
	uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
	uint8_t last_status; // status of last I2C transmission
	boolean did_timeout;
	uint32_t io_timeout;
	uint32_t timeout_start_ms;
	uint32_t measurement_timing_budget_us;
} VL53L0X_t;

typedef struct{
	uint8_t tcc;
	uint8_t msrc;
	uint8_t dss;
	uint8_t pre_range;
	uint8_t final_range;
} SequenceStepEnables_t;

typedef struct{
	uint16_t pre_range_vcsel_period_pclks;
	uint16_t final_range_vcsel_period_pclks;
	uint16_t msrc_dss_tcc_mclks;
	uint16_t pre_range_mclks;
	uint16_t final_range_mclks;
	uint32_t msrc_dss_tcc_us;
	uint32_t pre_range_us;
	uint32_t final_range_us;
} SequenceStepTimeouts_t;

typedef enum{
	VcselPeriodPreRange = 0,
	VcselPeriodFinalRange,
} vcselPeriodType;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

uint8_t VL53L0X_Init(void);
void VL53L0X_StartContinuous(uint32_t period_ms);
void VL53L0X_StopContinuous(void);
uint16_t VL53L0X_ReadRangeContinuousMillimeters(void);
uint16_t VL53L0X_ReadRangeSingleMillimeters(void);
void VL53L0X_Task(void);
boolean VL53L0X_TimeoutOccurred(void);

void VL53L0X_WriteReg(uint8_t reg, uint8_t value);
void VL53L0X_WriteReg16bit(uint8_t reg, uint16_t value);
void VL53L0X_WriteReg32bit(uint8_t reg, uint16_t value);
void VL53L0X_WriteMulti(uint8_t reg, uint8_t *data, uint8_t size);
uint8_t VL53L0X_ReadReg(uint8_t reg);
uint16_t VL53L0X_ReadReg16bit(uint8_t reg);
void VL53L0X_ReadMulti(uint8_t reg, uint8_t *output, uint8_t size);

void VL53L0X_SetSignalRateLimit(float mcps);
uint32_t VL53L0X_GetMeasurementTimingBudget(void);
void VL53L0X_GetSequenceStepEnables(SequenceStepEnables_t * enables);
void VL53L0X_GetSequenceStepTimeouts(SequenceStepEnables_t const * enables, SequenceStepTimeouts_t * timeouts);
uint16_t VL53L0X_GetVcselPulsePeriod(vcselPeriodType type);
boolean VL53L0X_SetMeasurementTimingBudget(uint32_t budget_us);
boolean VL53L0X_PerformSingleRefCalibration(uint8_t vhv_init_byte);


#endif

/******************************************************************************/
#endif /* _SOURCE_COMPONENTS_VL53L0X_VL53L0X_H_ */
