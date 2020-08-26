/**
  ******************************************************************************
  * @file    VL53L0X.h
  * @author  FranciscoBryant
  * @brief   Header for VL53L0X.c module. This file is made based on the
  * 		 VL53L0X API provided by STMicroelectronics.
  ******************************************************************************
  */

#ifndef __vl53l0x_h
#define __vl53l0x_h

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_envision_proximity.h"


/* Exported Constants ----------------------------------------------------------*/

// Register addresses from the API "vl53l0x_device.h" (ordered as listed there)

#define    SYSRANGE_START                                 0x00

#define    SYSTEM_THRESH_HIGH                             0x0C
#define    SYSTEM_THRESH_LOW                              0x0E

#define    SYSTEM_SEQUENCE_CONFIG                         0x01
#define    SYSTEM_RANGE_CONFIG                            0x09
#define    SYSTEM_INTERMEASUREMENT_PERIOD                 0x04

#define    SYSTEM_INTERRUPT_CONFIG_GPIO                   0x0A

#define    GPIO_HV_MUX_ACTIVE_HIGH                        0x84

#define    SYSTEM_INTERRUPT_CLEAR                         0x0B

#define    RESULT_INTERRUPT_STATUS                        0x13
#define    RESULT_RANGE_STATUS                            0x14

#define    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN          0xBC
#define    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN           0xC0
#define    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF          0xD0
#define    RESULT_CORE_RANGING_TOTAL_EVENTS_REF           0xD4
#define    RESULT_PEAK_SIGNAL_RATE_REF                    0xB6

#define    ALGO_PART_TO_PART_RANGE_OFFSET_MM              0x28

#define    I2C_SLAVE_DEVICE_ADDRESS                       0x8A

#define    MSRC_CONFIG_CONTROL                            0x60

#define    PRE_RANGE_CONFIG_MIN_SNR                       0x27
#define    PRE_RANGE_CONFIG_VALID_PHASE_LOW               0x56
#define    PRE_RANGE_CONFIG_VALID_PHASE_HIGH              0x57
#define    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT             0x64

#define    FINAL_RANGE_CONFIG_MIN_SNR                     0x67
#define    FINAL_RANGE_CONFIG_VALID_PHASE_LOW             0x47
#define    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH            0x48
#define    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT    0x44

#define    PRE_RANGE_CONFIG_SIGMA_THRESH_HI               0x61
#define    PRE_RANGE_CONFIG_SIGMA_THRESH_LO               0x62

#define    PRE_RANGE_CONFIG_VCSEL_PERIOD                  0x50
#define    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI             0x51
#define    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO             0x52

#define    SYSTEM_HISTOGRAM_BIN                           0x81
#define    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT          0x33
#define    HISTOGRAM_CONFIG_READOUT_CTRL                  0x55

#define    FINAL_RANGE_CONFIG_VCSEL_PERIOD                0x70
#define    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI           0x71
#define    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO           0x72
#define    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS          0x20

#define    MSRC_CONFIG_TIMEOUT_MACROP                     0x46

#define    SOFT_RESET_GO2_SOFT_RESET_N                    0xBF
#define    IDENTIFICATION_MODEL_ID                        0xC0
#define    IDENTIFICATION_REVISION_ID                     0xC2

#define    OSC_CALIBRATE_VAL                              0xF8

#define    GLOBAL_CONFIG_VCSEL_WIDTH                      0x32
#define    GLOBAL_CONFIG_SPAD_ENABLES_REF_0               0xB0
#define    GLOBAL_CONFIG_SPAD_ENABLES_REF_1               0xB1
#define    GLOBAL_CONFIG_SPAD_ENABLES_REF_2               0xB2
#define    GLOBAL_CONFIG_SPAD_ENABLES_REF_3               0xB3
#define    GLOBAL_CONFIG_SPAD_ENABLES_REF_4               0xB4
#define    GLOBAL_CONFIG_SPAD_ENABLES_REF_5               0xB5

#define    GLOBAL_CONFIG_REF_EN_START_SELECT              0xB6
#define    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD            0x4E
#define    DYNAMIC_SPAD_REF_EN_START_OFFSET               0x4F
#define    POWER_MANAGEMENT_GO1_POWER_FORCE               0x80

#define    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV              0x89

#define    ALGO_PHASECAL_LIM                              0x30
#define    ALGO_PHASECAL_CONFIG_TIMEOUT                   0x30


/* Exported Functions ------------------------------------------------------------*/

/**
  * @brief  VL53L0X sensor initialisation using sequence based on VL53L0X_DataInit(),
  *			VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration() from the API
  *
  * @param  io_2v8: if you put in true then the sensor is configured for 2V8 mode,
  * 				if false then it's configured for 1V8 mode.
  *
  * @retval True if no error occured, false otherwise
  */
bool VL53L0X_init(bool io_2v8);

/**
  * @brief  Set the return signal rate limit check value in units of MCPS (mega counts
  * 		per second)
  *
  * @param  limit_Mcps: return signal rate limit check
  *
  * @note	This represents the amplitude of the signal reflected from the
  *			target and detected by the device; setting this limit presumably determines
  *			the minimum measurement necessary for the sensor to report a valid reading.
  *			Setting a lower limit increases the potential range of the sensor but also
  *			seems to increase the likelihood of getting an inaccurate reading because of
  *			unwanted reflections from objects other than the intended target.
  *			Defaults to 0.25 MCPS as initialized by the ST API and this library.
  *
  * @retval True if no error occured, false otherwise
  */
bool VL53L0X_setSignalRateLimit(float limit_Mcps);

/**
  * @brief  Set the measurement timing budget (time allowed for 1 measurement)
  *
  * @param  budget_us: measurement timing budget value in microseconds
  *
  * @note	A longer timing budget allows for more accurate measurements.
  * 		Increasing the budget by a factor of N decreases the range measurement
  * 		standard deviation by a factor of sqrt(N). Defaults to about 33
  * 		milliseconds; the minimum is 20 ms.
  *
  * @retval True if no error occured, false otherwise
  */
bool VL53L0X_setMeasurementTimingBudget(uint32_t budget_us);

/**
  * @brief  Get the current measurement timing budget
  *
  * @param  None
  *
  * @retval Current measurement timing budget in microseconds
  */
uint32_t VL53L0X_getMeasurementTimingBudget(void);

/**
  * @brief  Set the VCSEL CVertical Cavity Surface Emitting Laser) pulse period for the
  * 		given period type (pre-range or final range) to the given value in PCLKs.
  *
  * @param  vcselPeriodType: VcselPeriodPreRange or VcselPeriodFinalRange
  * 		period_pclks: The value you would like to set for the pulse period
  *
  * @note	Longer periods increase the potential range of the sensor.
  * 	    Valid values are (even numbers only):
  *			pre:  12 to 18 (initialized default: 14)
  *			final: 8 to 14 (initialized default: 10)
  *
  * @retval True if no error occured, false otherwise
  */
bool VL53L0X_setVcselPulsePeriod(int vcselPeriodType, uint8_t period_pclks);

/**
  * @brief  Get the current VCSEL pulse period for the given period type
  *
  * @param  vcselPeriodType: VcselPeriodPreRange or VcselPeriodFinalRange
  *
  * @retval Current VCSEL pulse period in PCKLs
  */
uint8_t VL53L0X_getVcselPulsePeriod(int vcselPeriodType);

/**
  * @brief  Start continuous ranging measurements
  *
  * @param  period_ms: A time period in milliseconds determining how often the sensor
  * 				   takes a measurement.
  *
  * @note	If the argument period_ms is 0, continuous back-to-back mode is used (the
  * 		sensor take measurements as often as possible, if it's nonzero, continuous
  * 		timed is used with period_ms is the delay between measurements
  *
  * @retval None
  */
void VL53L0X_startContinuous(uint32_t period_ms);

/**
  * @brief  Stops Continuous Mode
  *
  * @param  None
  *
  * @retval None
  */
void VL53L0X_stopContinuous(void);

/**
  * @brief  Get the range reading in continuous mode
  *
  * @param  None
  *
  * @retval Distance reading in millimeters
  */
uint16_t VL53L0X_readRangeContinuousMillimeters(void);

/**
  * @brief  Check whether a timeout occured in the read function
  *
  * @param  None
  *
  * @retval True if a timeout occured, false otherwise
  */
bool VL53L0X_timeoutOccurred(void);


// Private Variables and Functions:

struct SequenceStepEnables
{
  bool tcc, msrc, dss, pre_range, final_range;
};

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection


struct SequenceStepTimeouts
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
};

// Define for VcselPeriodType

#define VcselPeriodPreRange         	0
#define VcselPeriodFinalRange      	    1

/**
  * @brief  Record the current time to check an upcoming timeout against
  *
  * @param  None
  *
  * @retval None
  */
void VL53L0X_startTimeout(void);

/**
  * @brief  Set a timeout period for the sensor
  *
  * @param  timeout: Duration value in milliseconds
  *
  * @retval None
  */
void VL53L0X_setTimeout(uint16_t timeout);

/**
  * @brief  Check if timeout is enabled (set to nonzero value) and has expired
  *
  * @param  None
  *
  * @retval True if a timeout occured, false otherwise
  */
bool VL53L0X_checkTimeoutExpired(void);

/**
  * @brief  Set up the SequenceStepEnables
  *
  * @param  enables: pointer to the structure SequenceStepEnables
  *
  * @retval None
  */
void VL53L0X_getSequenceStepEnables(struct SequenceStepEnables * enables);

/**
  * @brief  Set up the SequenceStepTimeouts
  *
  * @param  enables: pointer to the structure SequenceStepEnables
  * 		timeouts: pointer to the structure SequenceStepTImeouts
  *
  * @retval None
  */
void VL53L0X_getSequenceStepTimeouts(struct SequenceStepEnables const * enables, struct SequenceStepTimeouts * timeouts);

/**
  * @brief  Decode sequence step timeout in MCLKs from register value
  */
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val);

/**
  * @brief  Encode sequence step timeout register value from timeout in MCLKs
  */
uint16_t VL53L0X_encodeTimeout(uint32_t timeout_mclks);

/**
  * @brief Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
  */
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

/**
  * @brief Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
  */
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);




#endif
