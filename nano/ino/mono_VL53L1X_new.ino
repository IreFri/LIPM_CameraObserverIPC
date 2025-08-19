/*******************************************************************************

This sketch file is derived from an example program
(Projects\Multi\Examples\VL53L1X\SimpleRangingExamples\Src\main.c) in the
X-CUBE-53L1A1 Long Distance Ranging sensor software expansion for STM32Cube
from ST, available here:

http://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube-expansion-software/stm32-ode-sense-sw/x-cube-53l1a1.html

The rest of the files in this sketch are from the STSW-IMG007 VL53L1X API from
ST, available here:

http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img007.html

********************************************************************************

COPYRIGHT(c) 2017 STMicroelectronics
COPYRIGHT(c) 2018 Pololu Corporation

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include <Wire.h>
#include "vl53l1_api.h"

// Constants
constexpr uint32_t  MEASUREMENT_BUDGET_MS         = 10;
constexpr uint32_t  INTER_MEASUREMENT_PERIOD_MS   = 15;

constexpr bool      DISPLAY_REFLECTANCE           = false;
// Must change this value to drop the undesired value depending on the threshold
constexpr bool      USE_REFLECTANCE_THRESHOLD     = true;
constexpr float     REFLECTANCE_THRESHOLD         = 0.5;

constexpr bool      DISPLAY_FILTERED_RANGE        = false;
constexpr bool      USE_LOW_PASS_FILTER           = true;
constexpr float     LOW_PASS_FILTER_ALPHA_MOTION  = 0.75;
constexpr float     LOW_PASS_FILTER_ALPHA_STATIC  = 0.98;
constexpr uint8_t   NR_INITIAL_AVERAGE            = 10;

// Must change this value to apply the correct offset
constexpr uint16_t  OFFSET_IN_MM                  = 0;

// Global variables
float               average                       = 0.;
float               sum_average                   = 0.;
uint8_t             counter_initial_average       = 0;

VL53L1_Dev_t        dev;
VL53L1_DEV          Dev                           = &dev;


namespace
{

VL53L1_Error VL53L1X_SetROI(uint16_t dev, uint16_t X, uint16_t Y)
{
	uint8_t OpticalCenter;
	VL53L1_Error status = 0;

	status = VL53L1_RdByte(dev, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &OpticalCenter);
	if (X > 16)
		X = 16;
	if (Y > 16)
		Y = 16;
	if (X > 10 || Y > 10){
		OpticalCenter = 199;
	}
	status = VL53L1_WrByte(dev, VL53L1_ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
	status = VL53L1_WrByte(dev, VL53L1_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
		       (Y - 1) << 4 | (X - 1));
	return status;
}

String VL53L1_ErrorToString(VL53L1_Error error)
{
  switch(error)
  {
    case VL53L1_ERROR_NONE:
      return "VL53L1_ERROR_NONE";
    case VL53L1_ERROR_CALIBRATION_WARNING:
      return "VL53L1_ERROR_CALIBRATION_WARNING";
    case VL53L1_ERROR_MIN_CLIPPED:
      return "VL53L1_ERROR_MIN_CLIPPED";
    case VL53L1_ERROR_UNDEFINED:
      return "VL53L1_ERROR_UNDEFINED";
    case VL53L1_ERROR_INVALID_PARAMS:
      return "VL53L1_ERROR_INVALID_PARAMS";
    case VL53L1_ERROR_NOT_SUPPORTED:
      return "VL53L1_ERROR_NOT_SUPPORTED";
    case VL53L1_ERROR_RANGE_ERROR:
      return "VL53L1_ERROR_RANGE_ERROR";
    case VL53L1_ERROR_TIME_OUT:
      return "VL53L1_ERROR_TIME_OUT";
    case VL53L1_ERROR_MODE_NOT_SUPPORTED:
      return "VL53L1_ERROR_MODE_NOT_SUPPORTED";
    case VL53L1_ERROR_BUFFER_TOO_SMALL:
      return "VL53L1_ERROR_BUFFER_TOO_SMALL";
    case VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL:
      return "VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL";
    case VL53L1_ERROR_GPIO_NOT_EXISTING:
      return "VL53L1_ERROR_GPIO_NOT_EXISTING";
    case VL53L1_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED:
      return "VL53L1_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED";
    case VL53L1_ERROR_CONTROL_INTERFACE:
      return "VL53L1_ERROR_CONTROL_INTERFACE";
    case VL53L1_ERROR_INVALID_COMMAND:
      return "VL53L1_ERROR_INVALID_COMMAND";
    case VL53L1_ERROR_DIVISION_BY_ZERO:
      return "VL53L1_ERROR_DIVISION_BY_ZERO";
    case VL53L1_ERROR_REF_SPAD_INIT:
      return "VL53L1_ERROR_REF_SPAD_INIT";
    case VL53L1_ERROR_GPH_SYNC_CHECK_FAIL:
      return "VL53L1_ERROR_GPH_SYNC_CHECK_FAIL";
    case VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL:
      return "VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL";
    case VL53L1_ERROR_GPH_ID_CHECK_FAIL:
      return "VL53L1_ERROR_GPH_ID_CHECK_FAIL";
    case VL53L1_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL:
      return "VL53L1_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL";
    case VL53L1_ERROR_ZONE_GPH_ID_CHECK_FAIL:
      return "VL53L1_ERROR_ZONE_GPH_ID_CHECK_FAIL";
    case VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL:
      return "VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL";
    case VL53L1_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL:
      return "VL53L1_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL";
    case VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL:
      return "VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL";
    case VL53L1_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL:
      return "VL53L1_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL";
    case VL53L1_ERROR_ZONE_CAL_NO_SAMPLE_FAIL:
      return "VL53L1_ERROR_ZONE_CAL_NO_SAMPLE_FAIL";
    case VL53L1_ERROR_TUNING_PARM_KEY_MISMATCH:
      return "VL53L1_ERROR_TUNING_PARM_KEY_MISMATCH";
    case VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS:
      return "VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS";
    case VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH:
      return "VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH";
    case VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW:
      return "VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW";
    case VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES:
      return "VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES";
    case VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH:
      return "VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH";
    case VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH:
      return "VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH";
    case VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW:
      return "VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW";
    case VL53L1_WARNING_ZONE_CAL_MISSING_SAMPLES:
      return "VL53L1_WARNING_ZONE_CAL_MISSING_SAMPLES";
    case VL53L1_WARNING_ZONE_CAL_SIGMA_TOO_HIGH:
      return "VL53L1_WARNING_ZONE_CAL_SIGMA_TOO_HIGH";
    case VL53L1_WARNING_ZONE_CAL_RATE_TOO_HIGH:
      return "VL53L1_WARNING_ZONE_CAL_RATE_TOO_HIGH";
    case VL53L1_WARNING_XTALK_MISSING_SAMPLES:
      return "VL53L1_WARNING_XTALK_MISSING_SAMPLES";
    case VL53L1_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT:
      return "VL53L1_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT";
    case VL53L1_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT:
      return "VL53L1_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT";
    case VL53L1_ERROR_NOT_IMPLEMENTED:
      return "VL53L1_ERROR_NOT_IMPLEMENTED";
    case VL53L1_ERROR_PLATFORM_SPECIFIC_START:
      return "VL53L1_ERROR_PLATFORM_SPECIFIC_START";
    default:
      return "VL53L1_ERROR_NOT_DEFINED";
  }
}

void VL53L1_CheckError(const String& from, VL53L1_Error error)
{
  if(error)
  {
    Serial.println(from + ": " + VL53L1_ErrorToString(error));
    while(1);
  }
}

bool VL53L1_CheckErrorNonBlocking(const String& from, VL53L1_Error error)
{
  if(error)
  {
    Serial.println(from + ": " + VL53L1_ErrorToString(error));
  }
  return error == VL53L1_ERROR_NONE;
}

uint16_t VL53L1_LowPassFilter(uint16_t value)
{
  float newValue = (float) value;
  if(counter_initial_average < NR_INITIAL_AVERAGE)
  {
    sum_average += value;
    ++ counter_initial_average;
    return value;
  }
  else if(counter_initial_average == NR_INITIAL_AVERAGE)
  {
    average = sum_average / NR_INITIAL_AVERAGE;
    ++ counter_initial_average;
    Serial.print("InitialAverage:");
    Serial.println(average);
  }
  float alpha = abs(average - newValue) > 5. ? LOW_PASS_FILTER_ALPHA_MOTION : LOW_PASS_FILTER_ALPHA_STATIC;
  average = (alpha * average) + ((1.0 - alpha) * newValue);
  return average;
}

}


void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // This is the default 8-bit slave address (including R/W as the least
  // significant bit) as expected by the API. Note that the Arduino Wire library
  // uses a 7-bit address without the R/W bit instead (0x29 or 0b0101001).
  Dev->I2cDevAddr = 0x52;

  VL53L1_software_reset(Dev);

  VL53L1_CheckError("VL53L1_WaitDeviceBooted", VL53L1_WaitDeviceBooted(Dev));
  VL53L1_CheckError("VL53L1_DataInit", VL53L1_DataInit(Dev));
  VL53L1_CheckError("VL53L1_StaticInit", VL53L1_StaticInit(Dev));
  VL53L1_CheckError("VL53L1_SetDistanceMode", VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT));
  VL53L1_CheckError("VL53L1_SetMeasurementTimingBudgetMicroSeconds", VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, (uint32_t)MEASUREMENT_BUDGET_MS * 1000));
  VL53L1_CheckError("VL53L1_SetInterMeasurementPeriodMilliSeconds", VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS));
  VL53L1_CheckError("VL53L1_StartMeasurement", VL53L1_StartMeasurement(Dev));
  VL53L1_CheckError("VL53L1X_SetROI", VL53L1X_SetROI(Dev, 4, 4));

  // Serial.println("Starting VL53L1x is started!");
}

void loop()
{
  static uint16_t start_ms = millis();

  uint8_t is_ready;
  if(VL53L1_CheckErrorNonBlocking("VL53L1_GetMeasurementDataReady", VL53L1_GetMeasurementDataReady(Dev, &is_ready)))
  {
    if(is_ready)
    {
      VL53L1_RangingMeasurementData_t ranging_data;
      if(VL53L1_CheckErrorNonBlocking("VL53L1_GetRangingMeasurementData", VL53L1_GetRangingMeasurementData(Dev, &ranging_data)))
      {
        const float reflectance = ranging_data.SignalRateRtnMegaCps/65536.0;
        if(DISPLAY_REFLECTANCE)
        {
          Serial.print("reflectance:");
          Serial.println(reflectance);
        }

        const uint16_t range_mm = ranging_data.RangeMilliMeter + OFFSET_IN_MM;
        const uint16_t range_filtered_mm = VL53L1_LowPassFilter(range_mm);

        if(USE_REFLECTANCE_THRESHOLD)
        {
          Serial.print("top:");
          if(USE_LOW_PASS_FILTER)
          {
            Serial.println(range_filtered_mm);
          }
          else
          {
            Serial.println(range_mm);
          }
        }

        if(DISPLAY_FILTERED_RANGE)
        {
          Serial.print("filtered:");
          Serial.println(range_filtered_mm);
        }
      }
      VL53L1_CheckErrorNonBlocking("VL53L1_ClearInterruptAndStartMeasurement", VL53L1_ClearInterruptAndStartMeasurement(Dev));
      start_ms = millis();
    }
    else if((uint16_t)(millis() - start_ms) > VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS)
    {
      Serial.println("Timeout waiting for data ready.");
      VL53L1_CheckErrorNonBlocking("VL53L1_ClearInterruptAndStartMeasurement", VL53L1_ClearInterruptAndStartMeasurement(Dev));
      start_ms = millis();
    }
  }
}
