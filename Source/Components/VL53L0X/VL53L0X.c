/*
 * VL53L0X.c
 *
 *  Created on: August 14, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "VL53L0X.h"

#if(VL53L0X_ENABLED)

#if(VL53L0X_DEBUG_ENABLED)
#define DEBUG_PRINT_ENABLE
#endif
#include "Utility/Debug/Debug.h"

#include "Hard/I2C/I2C.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define decodeVcselPeriod(reg_val)                (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks)           (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks)       ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

#define startTimeout()                            (VL53L0X.timeout_start_ms = 0)
#define checkTimeoutExpired()                     ( (VL53L0X.io_timeout > 0) && ((VL53L0X.timeout_start_ms++) > VL53L0X.io_timeout) )


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

VL53L0X_t VL53L0X;
I2CBase_t VL52L0X_I2C = I2C_INSTANCE(VL53L0X_I2C_USED);

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static boolean VL53L0X_Setup(void);
static uint32_t VL53L0X_TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint16_t VL53L0X_DecodeTimeout(uint16_t reg_val);
static uint32_t VL53L0X_TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static uint16_t VL53L0X_EncodeTimeout(uint16_t timeout_mclks);

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

/******************************************************************************/

/**
 * @func   VL53L0X_WriteReg
 * @brief  Write register 1 byte
 * @param  Register address and value
 * @retval None
 */

void VL53L0X_WriteReg(uint8_t reg, uint8_t value){
	uint8_t writeData[2];
	
	writeData[0] = reg;
	writeData[1] = value;
	
	I2C_WriteData(&VL52L0X_I2C, VL53L0X_ADDRESS, writeData, 2);
}

/**
 * @func   VL53L0X_WriteReg16bit
 * @brief  Write register 2 bytes
 * @param  Register address and value
 * @retval None
 */

void VL53L0X_WriteReg16bit(uint8_t reg, uint16_t value){
	uint8_t writeData[3];
	
	writeData[0] = reg;
	writeData[1] = (value >> 8) & 0xFF;
	writeData[2] = value & 0xFF;
	
	I2C_WriteData(&VL52L0X_I2C, VL53L0X_ADDRESS, writeData, 3);
}

/**
 * @func   VL53L0X_WriteReg16bit
 * @brief  Write register 4 bytes
 * @param  Register address and value
 * @retval None
 */

void VL53L0X_WriteReg32bit(uint8_t reg, uint16_t value){
	uint8_t writeData[5];
	
	writeData[0] = reg;
	writeData[1] = (value >> 24) & 0xFF;
	writeData[2] = (value >> 16) & 0xFF;
	writeData[3] = (value >> 8) & 0xFF;
	writeData[4] = value & 0xFF;
	
	I2C_WriteData(&VL52L0X_I2C, VL53L0X_ADDRESS, writeData, 5);
}

/**
 * @func   VL53L0X_WriteMulti
 * @brief  Write register n bytes
 * @param  Register address and value, size
 * @retval None
 */

void VL53L0X_WriteMulti(uint8_t reg, uint8_t *data, uint8_t size){
	uint8_t writeData[16];
	
	writeData[0] = reg;
	memcpy(&writeData[1], data, size);
	
	I2C_WriteData(&VL52L0X_I2C, VL53L0X_ADDRESS, writeData, size + 1);
}

/**
 * @func   VL53L0X_ReadReg
 * @brief  Read register 1 byte
 * @param  Register address and value
 * @retval Read value
 */

uint8_t VL53L0X_ReadReg(uint8_t reg){
	uint8_t retVal;
	
	I2C_WriteData(&VL52L0X_I2C, VL53L0X_ADDRESS, &reg, 1);
	I2C_ReadData(&VL52L0X_I2C, VL53L0X_ADDRESS, &retVal, 1);
	
	return retVal;
}

/**
 * @func   VL53L0X_ReadReg16bit
 * @brief  Read register 2 bytes
 * @param  Register address and value
 * @retval Read value
 */

uint16_t VL53L0X_ReadReg16bit(uint8_t reg){
	I2C_WriteData(&VL52L0X_I2C, VL53L0X_ADDRESS, &reg, 1);
	
	uint8_t data[2];
	I2C_ReadData(&VL52L0X_I2C, VL53L0X_ADDRESS, data, 2);
	
	return (uint16_t) ((data[0] << 8) | data[1]);
}

/**
 * @func   VL53L0X_ReadMulti
 * @brief  Read multi bytes
 * @param  Register address and output data, size
 * @retval None
 */

void VL53L0X_ReadMulti(uint8_t reg, uint8_t *output, uint8_t size){
	I2C_WriteData(&VL52L0X_I2C, VL53L0X_ADDRESS, &reg, 1);
	
	I2C_ReadData(&VL52L0X_I2C, VL53L0X_ADDRESS, output, size);
}

/******************************************************************************/

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()

static uint32_t VL53L0X_TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks){
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	return((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.

static uint16_t VL53L0X_DecodeTimeout(uint16_t reg_val){
	// format: "(LSByte * 2^MSByte) + 1"
	return(uint16_t) ((reg_val & 0x00FF) << (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()

static uint32_t VL53L0X_TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks){
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	return(((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.

static uint16_t VL53L0X_EncodeTimeout(uint16_t timeout_mclks){
	// format: "(LSByte * 2^MSByte) + 1"

	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte >>= 1;
			ms_byte++;
		}

		return(ms_byte << 8) | (ls_byte & 0xFF);
	} else {
		return 0;
	}
}

/**
 * @func   VL53L0X_SetSignalRateLimit
 * @brief  Set the return signal rate limit check value in units of MCPS (mega counts per second).
 *         "This represents the amplitude of the signal reflected from the target and detected by the device";
 *         setting this limit presumably determines the minimum measurement necessary for the sensor to report a valid reading.
 *         Setting a lower limit increases the potential range of the sensor but also seems to increase the likelihood of getting
 *         an inaccurate reading because of unwanted reflections from objects other than the intended target.
 *         Defaults to 0.25 MCPS as initialized by the ST API and this library.
 * @param  MCPS
 * @retval None
 */

void VL53L0X_SetSignalRateLimit(float mcps){
	if ( (mcps < 0) || (mcps > 511.99) ){
		return;
	}
	
	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	VL53L0X_WriteReg16bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, mcps * (1 << 7));
}

/**
 * @func   VL53L0X_GetSpadInfo
 * @brief  Get reference SPAD (single photon avalanche diode) count and type based on VL53L0X_get_info_from_device(),
 *         but only gets reference SPAD count and type
 * @param  Output: count
 *                 typeIsAperture
 * @retval False if timeout
 */

static boolean VL53L0X_GetSpadInfo(uint8_t *count, boolean *typeIsAperture){
	uint8_t temp;
	
	VL53L0X_WriteReg(0x80, 0x01);
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x00);

	VL53L0X_WriteReg(0xFF, 0x06);
	VL53L0X_WriteReg(0x83, VL53L0X_ReadReg(0x83) | 0x04);
	VL53L0X_WriteReg(0xFF, 0x07);
	VL53L0X_WriteReg(0x81, 0x01);

	VL53L0X_WriteReg(0x80, 0x01);

	VL53L0X_WriteReg(0x94, 0x6b);
	VL53L0X_WriteReg(0x83, 0x00);
	
	startTimeout();
	while(VL53L0X_ReadReg(0x83) == 0x00){
		if(checkTimeoutExpired()){ return FALSE; }
	}
	
	VL53L0X_WriteReg(0x83, 0x01);
	temp = VL53L0X_ReadReg(0x92);

	*count = temp & 0x7f;
	*typeIsAperture = (boolean) ((temp >> 7) & 0x01);

	VL53L0X_WriteReg(0x81, 0x00);
	VL53L0X_WriteReg(0xFF, 0x06);
	VL53L0X_WriteReg(0x83, VL53L0X_ReadReg(0x83) & ~0x04);
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x01);

	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x80, 0x00);
	
	return TRUE;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()

uint16_t VL53L0X_GetVcselPulsePeriod(vcselPeriodType type){
	if (type == VcselPeriodPreRange) {
		return decodeVcselPeriod(VL53L0X_ReadReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
	} else if (type == VcselPeriodFinalRange) {
		return decodeVcselPeriod(VL53L0X_ReadReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	} else {
		return 255;
	}
}

/**
 * @func   VL53L0X_GetSequenceStepEnables
 * @brief  Get sequence step enables
 *         based on VL53L0X_GetSequenceStepEnables()
 * @param  Output: enables
 * @retval None
 */

void VL53L0X_GetSequenceStepEnables(SequenceStepEnables_t * enables){
	uint8_t sequence_config = VL53L0X_ReadReg(SYSTEM_SEQUENCE_CONFIG);

	enables->tcc = (sequence_config >> 4) & 0x1;
	enables->dss = (sequence_config >> 3) & 0x1;
	enables->msrc = (sequence_config >> 2) & 0x1;
	enables->pre_range = (sequence_config >> 6) & 0x1;
	enables->final_range = (sequence_config >> 7) & 0x1;
}

/**
 * @func   VL53L0X_GetSequenceStepTimeouts
 * @brief  
 * @param  
 * @retval None
 */

void VL53L0X_GetSequenceStepTimeouts(SequenceStepEnables_t const * enables, SequenceStepTimeouts_t * timeouts){
	timeouts->pre_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(VcselPeriodPreRange);

	timeouts->msrc_dss_tcc_mclks = VL53L0X_ReadReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	timeouts->msrc_dss_tcc_us = VL53L0X_TimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

	timeouts->pre_range_mclks = VL53L0X_DecodeTimeout(VL53L0X_ReadReg16bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	timeouts->pre_range_us = VL53L0X_TimeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

	timeouts->final_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(VcselPeriodFinalRange);

	timeouts->final_range_mclks = VL53L0X_DecodeTimeout(VL53L0X_ReadReg16bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	if(enables->pre_range){
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	}

	timeouts->final_range_us = VL53L0X_TimeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}



/**
 * @func   VL53L0X_GetMeasurementTimingBudget
 * @brief  Get the measurement timing budget in microseconds
 *         based on VL53L0X_get_measurement_timing_budget_micro_seconds() in us
 * @param  None
 * @retval budget_us
 */

uint32_t VL53L0X_GetMeasurementTimingBudget(void){
	uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	
	SequenceStepEnables_t enables;
	SequenceStepTimeouts_t timeouts;
	
	// "Start and end overhead times always present"
	uint32_t budget_us = (uint32_t) (StartOverhead + EndOverhead);
	
	VL53L0X_GetSequenceStepEnables(&enables);
	VL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);
	
	if (enables.tcc)
	{
	budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
	budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
	budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
	budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
	budget_us += (timeouts.final_range_us + FinalRangeOverhead);
	}

	VL53L0X.measurement_timing_budget_us = budget_us; // store for internal reuse
	return budget_us;
}

/* Set the measurement timing budget in microseconds, which is the time allowed
 * for one measurement; the ST API and this library take care of splitting the
 * timing budget among the sub-steps in the ranging sequence. A longer timing
 * budget allows for more accurate measurements. Increasing the budget by a
 * factor of N decreases the range measurement standard deviation by a factor of
 * sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
 * based on VL53L0X_set_measurement_timing_budget_micro_seconds()
*/

boolean VL53L0X_setMeasurementTimingBudget(uint32_t budget_us){
	SequenceStepEnables_t enables;
	SequenceStepTimeouts_t timeouts;

	uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;

	uint32_t const MinTimingBudget = 20000;

	if (budget_us < MinTimingBudget) {
		return FALSE;
	}

	uint32_t used_budget_us = (uint32_t) (StartOverhead + EndOverhead);

	VL53L0X_GetSequenceStepEnables(&enables);
	VL53L0X_GetSequenceStepTimeouts(&enables, &timeouts);

	if (enables.tcc) {
		used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss) {
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	} else if (enables.msrc) {
		used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range) {
		used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range) {
		used_budget_us += FinalRangeOverhead;

		// "Note that the final range timeout is determined by the timing
		// budget and the sum of all other timeouts within the sequence.
		// If there is no room for the final range timeout, then an error
		// will be set. Otherwise the remaining time will be applied to
		// the final range."

		if (used_budget_us > budget_us) {
			// "Requested timeout too big."
			return FALSE;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;
		
		/* "For the final range timeout, the pre-range timeout
		 *  must be added. To do this both final and pre-range
		 *  timeouts must be expressed in macro periods MClks
		 *  because they have different vcsel periods."
		*/
		uint16_t final_range_timeout_mclks = (uint16_t) VL53L0X_TimeoutMicrosecondsToMclks(final_range_timeout_us, (uint8_t) timeouts.final_range_vcsel_period_pclks);

		if (enables.pre_range) {
			final_range_timeout_mclks += (uint16_t) timeouts.pre_range_mclks;
		}

		VL53L0X_WriteReg16bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_EncodeTimeout(final_range_timeout_mclks));

		// set_sequence_step_timeout() end
		VL53L0X.measurement_timing_budget_us = budget_us; // store for internal reuse
	}
	return TRUE;
}

// based on VL53L0X_perform_single_ref_calibration()

boolean VL53L0X_PerformSingleRefCalibration(uint8_t vhv_init_byte){
	VL53L0X_WriteReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
	
	startTimeout();
	while ((VL53L0X_ReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (checkTimeoutExpired()){ return FALSE; }
	}

	VL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	VL53L0X_WriteReg(SYSRANGE_START, 0x00);

	return TRUE;
}

/**
 * @func   VL53L0X_Setup
 * @brief  
 * @param  None
 * @retval None
 */

static boolean VL53L0X_Setup(void){
	uint8_t ref_spad_map[6];
	uint8_t spad_count = 0;
	boolean spad_type_is_aperture;;
	uint8_t first_spad_to_enable;
	uint8_t spads_enabled;
	
#ifdef IO_2V8
	VL53L0X_WriteReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, VL53L0X_ReadReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
#endif
	//VL53L0X_WriteReg(0x88, 0x00);  // "Set I2C standard mode"
	
	VL53L0X_WriteReg(0x80, 0x01);
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x00);
	VL53L0X.stop_variable = VL53L0X_ReadReg(0x91);
	VL53L0X_WriteReg(0x00, 0x01);
	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x80, 0x00);
	
	// Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	VL53L0X_WriteReg(MSRC_CONFIG_CONTROL, VL53L0X_ReadReg(MSRC_CONFIG_CONTROL) | 0x12);
	
	
	// Set final range signal rate limit to 0.25 MCPS (million counts per second)
	VL53L0X_SetSignalRateLimit(0.25);
	
	VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);
	
	if (!VL53L0X_GetSpadInfo(&spad_count, &spad_type_is_aperture)) {
		return FALSE;
	}
	
	/* The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	 * the API, but the same data seems to be more easily readable from
	 * GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	 */
	VL53L0X_ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
	
	// VL53L0X_set_reference_spads() begin (assume NVM values are valid)
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	VL53L0X_WriteReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
	
	first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	spads_enabled = 0;
	
	for (uint8_t i = 0; i < 48; i++) {
		if (i < first_spad_to_enable || spads_enabled == spad_count) {
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			spads_enabled++;
		}
	}
	
	VL53L0X_WriteMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
	
	/* VL53L0X_set_reference_spads() end
	 * VL53L0X_load_tuning_settings() begin
	 * DefaultTuningSettings from vl53l0x_tuning.h
	 */
	
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x00);

	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x09, 0x00);
	VL53L0X_WriteReg(0x10, 0x00);
	VL53L0X_WriteReg(0x11, 0x00);

	VL53L0X_WriteReg(0x24, 0x01);
	VL53L0X_WriteReg(0x25, 0xFF);
	VL53L0X_WriteReg(0x75, 0x00);

	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x4E, 0x2C);
	VL53L0X_WriteReg(0x48, 0x00);
	VL53L0X_WriteReg(0x30, 0x20);

	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x30, 0x09);
	VL53L0X_WriteReg(0x54, 0x00);
	VL53L0X_WriteReg(0x31, 0x04);
	VL53L0X_WriteReg(0x32, 0x03);
	VL53L0X_WriteReg(0x40, 0x83);
	VL53L0X_WriteReg(0x46, 0x25);
	VL53L0X_WriteReg(0x60, 0x00);
	VL53L0X_WriteReg(0x27, 0x00);
	VL53L0X_WriteReg(0x50, 0x06);
	VL53L0X_WriteReg(0x51, 0x00);
	VL53L0X_WriteReg(0x52, 0x96);
	VL53L0X_WriteReg(0x56, 0x08);
	VL53L0X_WriteReg(0x57, 0x30);
	VL53L0X_WriteReg(0x61, 0x00);
	VL53L0X_WriteReg(0x62, 0x00);
	VL53L0X_WriteReg(0x64, 0x00);
	VL53L0X_WriteReg(0x65, 0x00);
	VL53L0X_WriteReg(0x66, 0xA0);

	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x22, 0x32);
	VL53L0X_WriteReg(0x47, 0x14);
	VL53L0X_WriteReg(0x49, 0xFF);
	VL53L0X_WriteReg(0x4A, 0x00);

	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x7A, 0x0A);
	VL53L0X_WriteReg(0x7B, 0x00);
	VL53L0X_WriteReg(0x78, 0x21);

	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x23, 0x34);
	VL53L0X_WriteReg(0x42, 0x00);
	VL53L0X_WriteReg(0x44, 0xFF);
	VL53L0X_WriteReg(0x45, 0x26);
	VL53L0X_WriteReg(0x46, 0x05);
	VL53L0X_WriteReg(0x40, 0x40);
	VL53L0X_WriteReg(0x0E, 0x06);
	VL53L0X_WriteReg(0x20, 0x1A);
	VL53L0X_WriteReg(0x43, 0x40);

	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x34, 0x03);
	VL53L0X_WriteReg(0x35, 0x44);

	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x31, 0x04);
	VL53L0X_WriteReg(0x4B, 0x09);
	VL53L0X_WriteReg(0x4C, 0x05);
	VL53L0X_WriteReg(0x4D, 0x04);

	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x44, 0x00);
	VL53L0X_WriteReg(0x45, 0x20);
	VL53L0X_WriteReg(0x47, 0x08);
	VL53L0X_WriteReg(0x48, 0x28);
	VL53L0X_WriteReg(0x67, 0x00);
	VL53L0X_WriteReg(0x70, 0x04);
	VL53L0X_WriteReg(0x71, 0x01);
	VL53L0X_WriteReg(0x72, 0xFE);
	VL53L0X_WriteReg(0x76, 0x00);
	VL53L0X_WriteReg(0x77, 0x00);

	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x0D, 0x01);

	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x80, 0x01);
	VL53L0X_WriteReg(0x01, 0xF8);

	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x8E, 0x01);
	VL53L0X_WriteReg(0x00, 0x01);
	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x80, 0x00);
	
	/* VL53L0X_load_tuning_settings() end
	 * "Set interrupt config to new sample ready"
	 * VL53L0X_SetGpioConfig() begin
	*/
	VL53L0X_WriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	VL53L0X_WriteReg(GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t) (VL53L0X_ReadReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10)); // active low
	VL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	
	// VL53L0X_SetGpioConfig() end
	VL53L0X.measurement_timing_budget_us = VL53L0X_GetMeasurementTimingBudget();
	
	/* "Disable MSRC and TCC by default"
	 * MSRC = Minimum Signal Rate Check
	 * TCC = Target CentreCheck
	 * VL53L0X_SetSequenceStepEnable() begin
	*/
	VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
	
	// "Recalculate timing budget"
	VL53L0X_setMeasurementTimingBudget(VL53L0X.measurement_timing_budget_us);
	
	VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (!VL53L0X_PerformSingleRefCalibration(0x40)) {
		return FALSE;
	}
	
	VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!VL53L0X_PerformSingleRefCalibration(0x00)) {
		return FALSE;
	}
	
	// "restore the previous Sequence Config"
	VL53L0X_WriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
	
	return TRUE;
}

/* Start continuous ranging measurements. If period_ms (optional) is 0 or not
 * given, continuous back-to-back mode is used (the sensor takes measurements as
 * often as possible); otherwise, continuous timed mode is used, with the given
 * inter-measurement period in milliseconds determining how often the sensor
 * takes a measurement.
 * based on VL53L0X_StartMeasurement()
*/

void VL53L0X_StartContinuous(uint32_t period_ms){
	VL53L0X_WriteReg(0x80, 0x01);
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x00);
	VL53L0X_WriteReg(0x91, VL53L0X.stop_variable);
	VL53L0X_WriteReg(0x00, 0x01);
	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x80, 0x00);

	if (period_ms != 0) {
		// continuous timed mode

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

		uint16_t osc_calibrate_val = VL53L0X_ReadReg16bit(OSC_CALIBRATE_VAL);

		if (osc_calibrate_val != 0) {
			period_ms *= osc_calibrate_val;
		}

		VL53L0X_WriteReg32bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

		VL53L0X_WriteReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	} else {
		// continuous back-to-back mode
		VL53L0X_WriteReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()

void VL53L0X_StopContinuous(void){
	VL53L0X_WriteReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x00);
	VL53L0X_WriteReg(0x91, 0x00);
	VL53L0X_WriteReg(0x00, 0x01);
	VL53L0X_WriteReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)

uint16_t VL53L0X_ReadRangeContinuousMillimeters(void){
	startTimeout();
	while ((VL53L0X_ReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0){if (checkTimeoutExpired()) { VL53L0X.did_timeout = TRUE; return 0xFFFF;}}

	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled
	uint16_t range = VL53L0X_ReadReg16bit(RESULT_RANGE_STATUS + 10);
	VL53L0X_WriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	
	return range;
}

boolean VL53L0X_TimeoutOccurred(void){
	boolean retVal = VL53L0X.did_timeout;
	
	VL53L0X.did_timeout = FALSE;
	return retVal;
}


// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()

uint16_t VL53L0X_ReadRangeSingleMillimeters(void){
	VL53L0X_WriteReg(0x80, 0x01);
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x00);
	VL53L0X_WriteReg(0x91, VL53L0X.stop_variable);
	VL53L0X_WriteReg(0x00, 0x01);
	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x80, 0x00);

	VL53L0X_WriteReg(SYSRANGE_START, 0x01);

	// "Wait until start bit has been cleared"
	startTimeout();
	while (VL53L0X_ReadReg(SYSRANGE_START) & 0x01){if (checkTimeoutExpired()) { return 0xFFFF;}}
	
	return VL53L0X_ReadRangeContinuousMillimeters();
}

/**
 * @func   VL53L0X_Task
 * @brief  Run VL53L0X task
 * @param  None
 * @retval Range
 */

uint16_t VL53L0X_Task(void){
	uint16_t range = VL53L0X_ReadRangeContinuousMillimeters();
	
	if( VL53L0X_TimeoutOccurred() ){
		DEBUG_PRINTLN("VL53L0X Timeout");
		VL53L0X_Init();
		return 0xFFFF;
	}
	else{
		// Handle range
		DEBUG_PRINTLN("Range %dmm", range);
	}
	return range;
}

/**
 * @func   VL53L0X_Init
 * @brief  Initialized VL53L0X
 * @param  None
 * @retval None
 */

uint8_t VL53L0X_Init(void){
	I2C_InitAsMaster(&VL52L0X_I2C);
	
	VL53L0X.io_timeout = VL53L0X_TIMEOUT;
	if(VL53L0X_Setup()){
		VL53L0X_StartContinuous(VL53L0X_TIME_MEASURE);
		return VL53L0X_OK;
	}
	return VL53L0X_ERROR;
}

#endif
