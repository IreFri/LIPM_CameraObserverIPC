#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor_top;
VL53L1X sensor_bot;

const uint8_t xshut_pin_top = 8;
const uint8_t xshut_pin_bot = 7;

bool sensor_top_is_init = false;
bool sensor_bot_is_init = false;

bool sensor_top_is_reading = false;
bool sensor_bot_is_reading = false;

unsigned long sensor_top_start_time_acquisition_ms = 0;
unsigned long sensor_bot_start_time_acquisition_ms = 0;
unsigned long delta_top_bot_ms = 12;

void setup()
{
  // while (!Serial) {}
  Serial.begin(115200);

  // Disable/reset all sensors by driving their XSHUT pins low.
  pinMode(xshut_pin_top, OUTPUT);
  digitalWrite(xshut_pin_top, LOW);

  pinMode(xshut_pin_bot, OUTPUT);
  digitalWrite(xshut_pin_bot, LOW);

  delay(500);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Enable, initialize, and start each sensor, one by one.
  // Stop driving this sensor's XSHUT low. This should allow the carrier
  // board to pull it high. (We do NOT want to drive XSHUT high since it is
  // not level shifted.) Then wait a bit for the sensor to start up.
  pinMode(xshut_pin_top, INPUT);
  delay(550);

  sensor_top.setTimeout(500);
  sensor_top_is_init = sensor_top.init();
  if (!sensor_top_is_init)
  {
    Serial.println("Failed to detect and initialize sensor top");
    // while (1);
  }
  else
  {
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensor_top.setAddress(0x2A + 6);
    sensor_top.setDistanceMode(VL53L1X::Short);
    sensor_top.setMeasurementTimingBudget(20000);
    sensor_top.stopContinuous();
    sensor_top.startContinuous(20);
    sensor_top.stopContinuous();
  }


  // // Enable, initialize, and start each sensor, one by one.
  // // Stop driving this sensor's XSHUT low. This should allow the carrier
  // // board to pull it high. (We do NOT want to drive XSHUT high since it is
  // // not level shifted.) Then wait a bit for the sensor to start up.
  pinMode(xshut_pin_bot, INPUT);
  delay(550);

  sensor_bot.setTimeout(500);
  sensor_bot_is_init = sensor_bot.init();
  if (!sensor_bot_is_init)
  {
    Serial.println("Failed to detect and initialize sensor bot");
    // while (1);
  }
  else
  {
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensor_bot.setAddress(0x2A + 0);
    sensor_bot.setDistanceMode(VL53L1X::Short);
    sensor_bot.setMeasurementTimingBudget(20000);
    sensor_bot.stopContinuous();
    sensor_bot.startContinuous(20);
    sensor_bot.stopContinuous();
  }

  if(!sensor_top_is_reading)
  {
    sensor_top.readRangeSingleMillimeters(false);
    sensor_top_is_reading = true;
    sensor_top_start_time_acquisition_ms = millis();
  }

}

void loop()
{
  if(sensor_top_is_init)
  {
    if(!sensor_top_is_reading && (sensor_bot_start_time_acquisition_ms + delta_top_bot_ms) < millis())
    {
      sensor_top_start_time_acquisition_ms = millis();
      sensor_top.readRangeSingleMillimeters(false);
      sensor_top_is_reading = true;
    }
    if(sensor_top.dataReady())
    {
      const uint16_t range = sensor_top.read(true);
      if(VL53L1X::RangeStatus::RangeValid == sensor_top.ranging_data.range_status)
      {
        Serial.print("top:");
        Serial.println(range);
      }
      else
      {
        Serial.print("top:");
        Serial.println(VL53L1X::rangeStatusToString(sensor_top.ranging_data.range_status));
      }
      sensor_top_is_reading = false;
    }
  }

  if(sensor_bot_is_init)
  {
    if(!sensor_bot_is_reading && (sensor_top_start_time_acquisition_ms + delta_top_bot_ms) < millis())
    {
      sensor_bot_start_time_acquisition_ms = millis();
      sensor_bot.readRangeSingleMillimeters(false);
      sensor_bot_is_reading = true;
    }
    if(sensor_bot.dataReady())
    {
      const uint16_t range = sensor_bot.read(true);
      if(VL53L1X::RangeStatus::RangeValid == sensor_bot.ranging_data.range_status)
      {
        Serial.print("bot:");
        Serial.println(range);
      }
      else
      {
        Serial.print("bot:");
        Serial.println(VL53L1X::rangeStatusToString(sensor_bot.ranging_data.range_status));
      }
      sensor_bot_is_reading = false;
    }
  }
}
