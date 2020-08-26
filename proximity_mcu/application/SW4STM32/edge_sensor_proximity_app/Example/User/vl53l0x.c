/*
 * VL53L0X.c
 *
 * Code for communicating with VL53L0X
 *
 *  Created on: Jun 29, 2020
 *      Author: franciscobryant
 */

#include "vl53l0x.h"


/* ------- Static macro declaration ---- */

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)


// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


/* -------- Static function declaration --------- */

bool VL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte);
bool VL53L0X_getSpadInfo(uint8_t * count, bool * type_is_aperture);


/* --------- Static variable declaration -----------*/

// Address for our VL53L0X I2C
#define ADDRESS_DEFAULT		UINT8_C(0x29 << 1)		// Use 8-bit address

// Define for VcselPeriodType
#define VcselPeriodPreRange         0
#define VcselPeriodFinalRange       1

uint8_t address;
uint16_t io_timeout;
bool did_timeout;
uint16_t timeout_start_ms;
uint32_t measurement_timing_budget_us;

uint8_t stop_variable;
// read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API



// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// This function takes in a boolean parameter, if you put in true then the sensor is configured
// for 2V8 mode, if false then it's configured for 1v8 Mode

bool VL53L0X_init(bool io_2v8)
{

	if (I2C1_Read(ADDRESS_DEFAULT, IDENTIFICATION_MODEL_ID) != 0xEE) {
		return false;
	}

	// VL53L0X_DataInit() begin


	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (io_2v8)
	{
		uint8_t io_mode = I2C1_Read(ADDRESS_DEFAULT, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
		I2C1_Write(ADDRESS_DEFAULT, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, io_mode | 0x01);
		// set bit 0
	}

	// "Set I2C standard mode"
	I2C1_Write(ADDRESS_DEFAULT, 0x88, 0x00);

	I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x01);
    I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
    I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x00);
    stop_variable = I2C1_Read(ADDRESS_DEFAULT, 0x91);
    I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x01);
    I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
    I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    uint8_t signal_rate = I2C1_Read(ADDRESS_DEFAULT, MSRC_CONFIG_CONTROL);
    I2C1_Write(ADDRESS_DEFAULT, MSRC_CONFIG_CONTROL, signal_rate | 0x12);

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	VL53L0X_setSignalRateLimit(0.25);

	I2C1_Write(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG, 0xFF);

	// VL53L0X_DataInit() end


	// VL53L0X_StaticInit() begin

	uint8_t spad_count;
	bool spad_type_is_aperture;
	if (!VL53L0X_getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];

	ref_spad_map[0] = I2C1_Read(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_0);
	ref_spad_map[1] = I2C1_Read(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_1);
	ref_spad_map[2] = I2C1_Read(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_2);
	ref_spad_map[3] = I2C1_Read(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_3);
	ref_spad_map[4] = I2C1_Read(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_4);
	ref_spad_map[5] = I2C1_Read(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_5);


	// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
	if (i < first_spad_to_enable || spads_enabled == spad_count)
	{
	  // This bit is lower than the first one that should be enabled, or
	  // (reference_spad_count) bits have already been enabled, so zero this bit
	  ref_spad_map[i / 8] &= ~(1 << (i % 8));
	}
	else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
	{
	  spads_enabled++;
	}
	}

	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[0]);
	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_1, ref_spad_map[1]);
	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_2, ref_spad_map[2]);
	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_3, ref_spad_map[3]);
	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_4, ref_spad_map[4]);
	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_SPAD_ENABLES_REF_5, ref_spad_map[5]);

	//	writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// -- VL53L0X_set_reference_spads() end


	// -- VL53L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53l0x_tuning.h

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x00);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x09, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x10, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x11, 0x00);

	I2C1_Write(ADDRESS_DEFAULT, 0x24, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x25, 0xFF);
	I2C1_Write(ADDRESS_DEFAULT, 0x75, 0x00);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x4E, 0x2C);
	I2C1_Write(ADDRESS_DEFAULT, 0x48, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x30, 0x20);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x30, 0x09);
	I2C1_Write(ADDRESS_DEFAULT, 0x54, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x31, 0x04);
	I2C1_Write(ADDRESS_DEFAULT, 0x32, 0x03);
	I2C1_Write(ADDRESS_DEFAULT, 0x40, 0x83);
	I2C1_Write(ADDRESS_DEFAULT, 0x46, 0x25);
	I2C1_Write(ADDRESS_DEFAULT, 0x60, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x27, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x50, 0x06);
	I2C1_Write(ADDRESS_DEFAULT, 0x51, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x52, 0x96);
	I2C1_Write(ADDRESS_DEFAULT, 0x56, 0x08);
	I2C1_Write(ADDRESS_DEFAULT, 0x57, 0x30);
	I2C1_Write(ADDRESS_DEFAULT, 0x61, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x62, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x64, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x65, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x66, 0xA0);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x22, 0x32);
	I2C1_Write(ADDRESS_DEFAULT, 0x47, 0x14);
	I2C1_Write(ADDRESS_DEFAULT, 0x49, 0xFF);
	I2C1_Write(ADDRESS_DEFAULT, 0x4A, 0x00);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x7A, 0x0A);
	I2C1_Write(ADDRESS_DEFAULT, 0x7B, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x78, 0x21);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x23, 0x34);
	I2C1_Write(ADDRESS_DEFAULT, 0x42, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x44, 0xFF);
	I2C1_Write(ADDRESS_DEFAULT, 0x45, 0x26);
	I2C1_Write(ADDRESS_DEFAULT, 0x46, 0x05);
	I2C1_Write(ADDRESS_DEFAULT, 0x40, 0x40);
	I2C1_Write(ADDRESS_DEFAULT, 0x0E, 0x06);
	I2C1_Write(ADDRESS_DEFAULT, 0x20, 0x1A);
	I2C1_Write(ADDRESS_DEFAULT, 0x43, 0x40);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x34, 0x03);
	I2C1_Write(ADDRESS_DEFAULT, 0x35, 0x44);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x31, 0x04);
	I2C1_Write(ADDRESS_DEFAULT, 0x4B, 0x09);
	I2C1_Write(ADDRESS_DEFAULT, 0x4C, 0x05);
	I2C1_Write(ADDRESS_DEFAULT, 0x4D, 0x04);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x44, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x45, 0x20);
	I2C1_Write(ADDRESS_DEFAULT, 0x47, 0x08);
	I2C1_Write(ADDRESS_DEFAULT, 0x48, 0x28);
	I2C1_Write(ADDRESS_DEFAULT, 0x67, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x70, 0x04);
	I2C1_Write(ADDRESS_DEFAULT, 0x71, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x72, 0xFE);
	I2C1_Write(ADDRESS_DEFAULT, 0x76, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x77, 0x00);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x0D, 0x01);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x01, 0xF8);

	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x8E, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x01);
	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
	I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x00);

	// -- VL53L0X_load_tuning_settings() end


	// "Set interrupt config to new sample ready"
	// -- VL53L0X_SetGpioConfig() begin

	I2C1_Write(ADDRESS_DEFAULT, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t gpiohv_status = I2C1_Read(ADDRESS_DEFAULT, GPIO_HV_MUX_ACTIVE_HIGH);
	I2C1_Write(ADDRESS_DEFAULT, GPIO_HV_MUX_ACTIVE_HIGH, gpiohv_status & ~0x10); // active low
	I2C1_Write(ADDRESS_DEFAULT, SYSTEM_INTERRUPT_CLEAR, 0x01);

	// -- VL53L0X_SetGpioConfig() end


	measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget();


	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- VL53L0X_SetSequenceStepEnable() begin

	I2C1_Write(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// -- VL53L0X_SetSequenceStepEnable() end


	// "Recalculate timing budget"
	VL53L0X_setMeasurementTimingBudget(measurement_timing_budget_us);

	// VL53L0X_StaticInit() end


	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())


	// -- VL53L0X_perform_vhv_calibration() begin

	I2C1_Write(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (!VL53L0X_performSingleRefCalibration(0x40)) {
		return false;
	}

	// -- VL53L0X_perform_vhv_calibration() end


	// -- VL53L0X_perform_phase_calibration() begin

	I2C1_Write(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!VL53L0X_performSingleRefCalibration(0x00)) {
		return false;
	}

	// -- VL53L0X_perform_phase_calibration() end


	// "restore the previous Sequence Config"
	I2C1_Write(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// VL53L0X_PerformRefCalibration() end


	return true;

}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.

bool VL53L0X_setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  uint16_t mcps_format = limit_Mcps * (1 << 7);
  uint8_t buffer[2];
  buffer[0] = (mcps_format >> 8) & 0xFF; 		// value high byte
  buffer[1] = mcps_format       & 0xFF; 		// value low byte
//  writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));

  I2C1_WriteBuffer(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, I2C_MEMADD_SIZE_8BIT, buffer, 2);

  return true;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()

bool VL53L0X_setMeasurementTimingBudget(uint32_t budget_us)
{
  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint32_t final_range_timeout_mclks =
      VL53L0X_timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    uint16_t step_timeout = VL53L0X_encodeTimeout(final_range_timeout_mclks);

    uint8_t buffer[2];
    buffer[0] = (step_timeout >> 8) & 0xFF; 		// value high byte
    buffer[1] = step_timeout       & 0xFF; 		// value low byte

//    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
//    		step_timeout);
    I2C1_WriteBuffer(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, I2C_MEMADD_SIZE_8BIT, buffer, 2);


    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

uint32_t VL53L0X_getMeasurementTimingBudget(void)
{
  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

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

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

void VL53L0X_getSequenceStepEnables(struct SequenceStepEnables * enables)
{
  uint8_t sequence_config = I2C1_Read(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

void VL53L0X_getSequenceStepTimeouts(struct SequenceStepEnables const * enables, struct SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = I2C1_Read(ADDRESS_DEFAULT, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    VL53L0X_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);


  uint8_t recBuffer[2];
  I2C1_ReadBuffer(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, I2C_MEMADD_SIZE_8BIT, recBuffer, 2);

  uint16_t pre_range = (((recBuffer[0] & 0xff) << 8) | (recBuffer[1] & 0xff));

  // now, length is 1 since we assume they can transfer 16bit at 1 time. But if cannot, then we have to make the transfer 8 bit and specify the length = 2
  // if we can't send 16bit at one time, see at polulu's readReg16Bit to see how he does it

  timeouts->pre_range_mclks =
		  VL53L0X_decodeTimeout(pre_range);

  timeouts->pre_range_us =
		  VL53L0X_timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(VcselPeriodFinalRange);

  uint8_t recBuffer2[2];

  I2C1_ReadBuffer(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, I2C_MEMADD_SIZE_8BIT, recBuffer2, 2);

  uint16_t final_range = (((recBuffer2[0] & 0xff) << 8) | (recBuffer2[1] & 0xff));


  timeouts->final_range_mclks =
		  VL53L0X_decodeTimeout(final_range);

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
		  VL53L0X_timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}



// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()

bool VL53L0X_setVcselPulsePeriod(int vcselPeriodType, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (vcselPeriodType == 0)  // type = VcselPeriodPreRange
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
    	I2C1_Write(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
    	I2C1_Write(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
    	I2C1_Write(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
    	I2C1_Write(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    I2C1_Write(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    I2C1_Write(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
    		VL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    uint16_t new_pre_range_timeout = VL53L0X_encodeTimeout(new_pre_range_timeout_mclks);
    uint8_t buffer[2];
    buffer[0] = (new_pre_range_timeout >> 8) & 0xFF; 		// value high byte
    buffer[1] = new_pre_range_timeout       & 0xFF; 		// value low byte

    I2C1_WriteBuffer(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, I2C_MEMADD_SIZE_8BIT, buffer, 2);
//    writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
//      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
    		VL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    I2C1_Write(ADDRESS_DEFAULT, MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (vcselPeriodType == 1) 	// type = VcselPeriodFinalRange
  {
    switch (period_pclks)
    {
      case 8:
		I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
		I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
		I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
		I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
		I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_LIM, 0x30);
		I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
        break;

      case 10:
    	I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
    	I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
    	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
    	I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
    	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
    	I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_LIM, 0x20);
    	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
        break;

      case 12:
    	I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
    	I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
    	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
    	I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
    	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
    	I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_LIM, 0x20);
    	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
        break;

      case 14:
    	I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
    	I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
    	I2C1_Write(ADDRESS_DEFAULT, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
    	I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
    	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
    	I2C1_Write(ADDRESS_DEFAULT, ALGO_PHASECAL_LIM, 0x20);
    	I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    I2C1_Write(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
    		VL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    uint16_t new_final_range_timeout = VL53L0X_encodeTimeout(new_final_range_timeout_mclks);
    uint8_t buffer[2];
    buffer[0] = (new_final_range_timeout >> 8) & 0xFF; 		// value high byte
    buffer[1] = new_final_range_timeout       & 0xFF; 		// value low byte

    I2C1_WriteBuffer(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, I2C_MEMADD_SIZE_8BIT, buffer, 2);
//    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
//      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  VL53L0X_setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = I2C1_Read(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG);
  I2C1_Write(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG, 0x02);
  VL53L0X_performSingleRefCalibration(0x0);
  I2C1_Write(ADDRESS_DEFAULT, SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X_getVcselPulsePeriod(int vcselPeriodType)
{
  if (vcselPeriodType == 0) // type = VcselPeriodPreRange
  {
	uint8_t pre_range_vcsel_period = I2C1_Read(ADDRESS_DEFAULT, PRE_RANGE_CONFIG_VCSEL_PERIOD);
    return decodeVcselPeriod(pre_range_vcsel_period);
  }
  else if (vcselPeriodType == 1) // type = VcselPeriodFinalRange
  {
	uint8_t final_range_vcsel_period = I2C1_Read(ADDRESS_DEFAULT, FINAL_RANGE_CONFIG_VCSEL_PERIOD);
    return decodeVcselPeriod(final_range_vcsel_period);
  }
  else { return 255; }
}

// Start continuous ranging measurements.
// You must give a value to this function's parameter, if you give 0 then continuous
// back-to-back mode is used (the sensor takes measurements as often as possible); otherwise,
// continuous timed mode is used, with your given value is the inter-measurement period in
// milliseconds determining how often the sensor takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X_startContinuous(uint32_t period_ms)
{
  I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x00);
  I2C1_Write(ADDRESS_DEFAULT, 0x91, stop_variable);
  I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
  I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

//    uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);

	  uint8_t recBuffer[2];

      I2C1_ReadBuffer(ADDRESS_DEFAULT, OSC_CALIBRATE_VAL, I2C_MEMADD_SIZE_8BIT, recBuffer, 2);

      uint16_t osc_calibrate_val = (((recBuffer[0] & 0xff) << 8) | (recBuffer[1] & 0xff));

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

//    writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    uint8_t buffer[4];
    buffer[0] = (period_ms >> 24) & 0xFF; 		// value high byte
    buffer[1] = (period_ms >> 16) 	& 0xFF;
    buffer[2] = (period_ms >> 8) 	& 0xFF;
    buffer[1] = period_ms       & 0xFF; 		// value low byte
    I2C1_WriteBuffer(ADDRESS_DEFAULT, SYSTEM_INTERMEASUREMENT_PERIOD, I2C_MEMADD_SIZE_8BIT, buffer, 4);


    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    I2C1_Write(ADDRESS_DEFAULT, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
	  I2C1_Write(ADDRESS_DEFAULT, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void VL53L0X_stopContinuous(void)
{
  I2C1_Write(ADDRESS_DEFAULT,SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x00);
  I2C1_Write(ADDRESS_DEFAULT, 0x91, 0x00);
  I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X_readRangeContinuousMillimeters(void)
{
  VL53L0X_startTimeout();
  uint8_t interrupt_status = I2C1_Read(ADDRESS_DEFAULT, RESULT_INTERRUPT_STATUS);

  while ((interrupt_status & 0x07) == 0)
  {
    if (VL53L0X_checkTimeoutExpired())
    {
      did_timeout = true;

      return 65535;

    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
//  uint16_t range = readReg16Bit(RESULT_RANGE_STATUS + 10);
  uint8_t recBuffer[2];

  I2C1_ReadBuffer(ADDRESS_DEFAULT, RESULT_RANGE_STATUS + 10, I2C_MEMADD_SIZE_8BIT, recBuffer, 2);

  uint16_t range = (((recBuffer[0] & 0xff) << 8) | (recBuffer[1] & 0xff));;


  I2C1_Write(ADDRESS_DEFAULT, SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53L0X_timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53L0X_getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x00);

  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x06);
  uint8_t temp1 = I2C1_Read(ADDRESS_DEFAULT, 0x83);
  I2C1_Write(ADDRESS_DEFAULT, 0x83, temp1 | 0x04);
  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x07);
  I2C1_Write(ADDRESS_DEFAULT, 0x81, 0x01);

  I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x01);

  I2C1_Write(ADDRESS_DEFAULT, 0x94, 0x6b);
  I2C1_Write(ADDRESS_DEFAULT, 0x83, 0x00);
  VL53L0X_startTimeout();

  uint8_t temp = I2C1_Read(ADDRESS_DEFAULT, 0x83);

  while (temp == 0x00)
  {
    if (VL53L0X_checkTimeoutExpired()) { return false; }
  }
  I2C1_Write(ADDRESS_DEFAULT, 0x83, 0x01);
  tmp = I2C1_Read(ADDRESS_DEFAULT, 0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  I2C1_Write(ADDRESS_DEFAULT, 0x81, 0x00);
  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x06);
  uint8_t temp2 = I2C1_Read(ADDRESS_DEFAULT, 0x83);
  I2C1_Write(ADDRESS_DEFAULT, 0x83, temp2  & ~0x04);
  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x01);
  I2C1_Write(ADDRESS_DEFAULT, 0x00, 0x01);

  I2C1_Write(ADDRESS_DEFAULT, 0xFF, 0x00);
  I2C1_Write(ADDRESS_DEFAULT, 0x80, 0x00);

  return true;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
uint16_t VL53L0X_encodeTimeout(uint32_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte)
{
  I2C1_Write(ADDRESS_DEFAULT, SYSRANGE_START, 0x01 | vhv_init_byte); 	// VL53L0X_REG_SYSRANGE_MODE_START_STOP

  VL53L0X_startTimeout();

  uint8_t temp = I2C1_Read(ADDRESS_DEFAULT, RESULT_INTERRUPT_STATUS);
  while (temp == 0)
  {
    if (VL53L0X_checkTimeoutExpired()) { return false; }
  }

  I2C1_Write(ADDRESS_DEFAULT, SYSTEM_INTERRUPT_CLEAR, 0x01);

  I2C1_Write(ADDRESS_DEFAULT, SYSRANGE_START, 0x00);

  return true;
}

void VL53L0X_startTimeout(void) {
	timeout_start_ms = (uint16_t)HAL_GetTick();
}

void VL53L0X_setTimeout(uint16_t timeout) {
	io_timeout = timeout;
}

bool VL53L0X_checkTimeoutExpired(void) {
	bool status;
	if(io_timeout > 0 && (((uint16_t)HAL_GetTick() - timeout_start_ms) > io_timeout)) {
		status = true;
	} else { status = false;}
	return status;
}





