/*
This script represents a sensor station with three sensors attached:
  * DHT22 for humidity and temperature,
  * TSL2561 for luminosity,
  * CCS811 for eCO2 and TVOC

Instructions to read sensor data are received through MQTT topics and
reading results are also published through MQTT topics.

Run this script on a ESP8266 board.
*/

#include <Wire.h>
#include "ccs811.h"
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ArduinoJson.h>
#include <tuple>
#include <string>
using namespace std;

/*
  The CCS811 is an ultra-low power digital gas sensor solution which integrates a metal oxide (MOX) gas sensor
  to detect a wide range of Volatile Organic Compounds (VOCs) for indoor air quality monitoring with a
  microcontroller unit (MCU), which includes an Analog-to-Digital converter (ADC), and an I²C interface.

  Wiring for ESP8266 NodeMCU boards:
    * VCC to 3V3,
    * GND to GND,
    * SDA to D2,
    * SCL to D1,
    * nWAKE to GND
*/
#define CCS811_MODE CCS811_MODE_1SEC
// #define CCS811_MODE CCS811_MODE_10SEC
// #define CCS811_MODE CCS811_MODE_60SEC
// #define CCS811_MODE CCS811_MODE_IDLE

#define CCS811_SLAVE_ADDR CCS811_SLAVEADDR_0
// #define CCS811_SLAVE_ADDR CCS811_SLAVEADDR_1

// Pin number connected to nWAKE (nWAKE can also be bound to GND, then pass -1)
#define CCS811_NWAKE_PIN -1

/*
  The DHT22 is a basic, low-cost digital temperature and humidity sensor.
  It uses a capacitive humidity sensor and a thermistor to measure the surrounding air,
  and spits out a digital signal on the data pin.
  It's fairly simple to use, but requires careful timing to grab data.
  The only real downside of this sensor is you can only get new data from it once every 2 seconds,
  so when using our library, sensor readings can be up to 2 seconds old.

  Wiring for ESP8266 NodeMCU boards:
    * VCC to 3V3
    * PIN 2 to whatever your DHT22_DATA_PIN is (default: D4)
    * Connect a 10k resistor from PIN 2 to VCC
    * GND to GND
*/
#define DHT22_DATA_PIN 2

#define DHT22_TEMPERATURE_UNIT_FAHRENHEIT false // Otherwise Celsius
// #define DHT22_TEMPERATURE_UNIT_FAHRENHEIT false

#define DHT_TYPE DHT22

/*
The TSL2561 luminosity sensor is an advanced digital light sensor, ideal for use in a wide range of light situations.
Compared to low cost CdS cells, this sensor is more precise, allowing for exact lux calculations and can be configured
for different gain/timing ranges to detect light ranges from up to 0.1 - 40,000+ Lux on the fly.
The best part of this sensor is that it contains both infrared and full spectrum diodes! That means you can separately measure infrared,
full-spectrum or human-visible light.

Wiring for ESP8266 NodeMCU boards:
  * Connect VCC to 3.3V
  * Connect GND to GND
  * Connect SCL to SCL
  * Connect SDA to SDA
*/
#define TSL2561_GAIN_AUTO true
// #define TSL2561_GAIN_AUTO false

#define TSL2561_GAIN TSL2561_GAIN_1X
// #define TSL2561_GAIN TSL2561_GAIN_16X

#define TSL2561_INTEGRATIONTIME TSL2561_INTEGRATIONTIME_13MS
// #define TSL2561_INTEGRATIONTIME TSL2561_INTEGRATIONTIME_101MS
// #define TSL2561_INTEGRATIONTIME TSL2561_INTEGRATIONTIME_402MS

#define TSL2561_ADDR TSL2561_ADDR_FLOAT
// #define TSL2561_ADDR TSL2561_ADDR_LOW
// #define TSL2561_ADDR TSL2561_ADDR_HIGH

#define TSL2561_SENSOR_ID -1

CCS811 ccs(CCS811_NWAKE_PIN, CCS811_SLAVE_ADDR);
DHT dht(DHT22_DATA_PIN, DHT_TYPE);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR, TSL2561_SENSOR_ID);

void setup_ccs811()
{
  ccs.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  if (!ccs.begin())
  {
    Serial.println("CSS811 begin FAILED");
    while (1)
      ;
  }
  if (!ccs.start(CCS811_MODE))
  {
    Serial.println("CCS811 start FAILED");
    while (1)
      ;
  }
}

void setup_dht22()
{
  dht.begin();
}

void setup_tsl2561()
{
  if (!tsl.begin())
  {
    Serial.println("TSL2561 begin FAILED");
    while (1)
      ;
  }
  if (TSL2561_GAIN_AUTO)
  {
    tsl.enableAutoRange(true);
  }
  else
  {
    tsl.setGain(TSL2561_GAIN);
  }
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME);
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  setup_ccs811();
  setup_dht22();
  setup_tsl2561();
}

DynamicJsonDocument buildDocument(char *sensor, char *measure, char *unit, float value, char *error)
{
  DynamicJsonDocument doc(1024);
  doc["sensor"] = sensor;
  doc["measure"] = measure;
  doc["unit"] = unit;
  doc["value"] = value;
  doc["error"] = error;
  return doc;
}

DynamicJsonDocument readHumidity()
{
  float value = dht.readHumidity();
  char *error = (char *)"";
  if (isnan(value))
  {
    error = (char *)"Error";
  }
  return buildDocument((char *)"dht22", (char *)"humidity", (char *)"%", value, error);
}

DynamicJsonDocument readTemperature()
{
  float value = dht.readTemperature(DHT22_TEMPERATURE_UNIT_FAHRENHEIT);
  char *error = (char *)"";
  char *unit = (char *)"° Celsius";
  if (DHT22_TEMPERATURE_UNIT_FAHRENHEIT)
  {
    unit = (char *)"° Fahrenheit";
  }
  if (isnan(value))
  {
    error = (char *)"Error";
  }
  return buildDocument((char *)"dht22", (char *)"temperature", unit, value, error);
}

uint16_t humidity_to_compensation(float humidity)
{
  // Humidity is stored as an unsigned 16 bits in 1/512%RH.
  // The default value is 50% = 0x64, 0x00.
  // As an example 48.5% humidity would be 0x61, 0x00.
  // return uint16(50 * 512); // Default value
  return uint16(humidity * 512);
}

uint16_t temperature_to_compensation(float temperature)
{
  // Temperature is stored as an unsigned 16 bits integer in 1/512 degrees;
  // there is an offset: 0 maps to -25°C. The default value is 25°C = 0x64, 0x00.
  // As an example 23.5% temperature would be 0x61, 0x00.
  // return uint16(50 * 512); // Default value
  return uint16((temperature + 25) * 512);
}

DynamicJsonDocument buildECO2Document(float value, char *error)
{
  return buildDocument((char *)"ccs811", (char *)"eco2", (char *)"ppm", value, error);
}

DynamicJsonDocument buildTVOCDocument(float value, char *error)
{
  return buildDocument((char *)"ccs811", (char *)"tvoc", (char *)"ppb", value, error);
}

void compensate_for_environment()
{
  float temperature, humidity;
  temperature = dht.readTemperature(true); // Always needs to be Celsius for conversion
  humidity = dht.readHumidity();
  // Make sure temperature and humidity are set,
  // otherwise CCS sensor values will be all over the place
  if (!(isnan(temperature) | isnan(humidity)))
  {
    ccs.set_envdata(temperature_to_compensation(temperature), humidity_to_compensation(humidity));
  }
}

tuple<float, float, char *> readCCS811()
{
  compensate_for_environment();
  uint16_t eco2, tvoc, errstat, raw;
  char *error = (char *)"";
  ccs.read(&eco2, &tvoc, &errstat, &raw);
  if (errstat == CCS811_ERRSTAT_OK_NODATA)
  {
    error = (char *)"Waiting for new data";
  }
  else if (errstat & CCS811_ERRSTAT_I2CFAIL)
  {
    error = (char *)"I2C error";
  }
  else if (errstat != CCS811_ERRSTAT_OK)
  {
    error = itoa(errstat, error, 16);
  }
  return make_tuple(eco2, tvoc, error);
}

DynamicJsonDocument readECO2()
{
  tuple<float, float, char *> eco2_tvoc_error = readCCS811();
  return buildECO2Document(get<0>(eco2_tvoc_error), get<2>(eco2_tvoc_error));
}

DynamicJsonDocument readTVOC()
{
  tuple<float, float, char *> eco2_tvoc_error = readCCS811();
  return buildTVOCDocument(get<1>(eco2_tvoc_error), get<2>(eco2_tvoc_error));
}

tuple<DynamicJsonDocument, DynamicJsonDocument> readECO2AndTVOC()
{
  tuple<float, float, char *> eco2_tvoc_error = readCCS811();
  return make_tuple(
      buildECO2Document(get<0>(eco2_tvoc_error), get<2>(eco2_tvoc_error)),
      buildTVOCDocument(get<1>(eco2_tvoc_error), get<2>(eco2_tvoc_error)));
}

tuple<float, char *> readTSL2561()
{
  sensors_event_t event;
  tsl.getEvent(&event);
  char *error = (char *)"";
  if (!event.light)
  {
    error = (char *)"Sensor overload";
  }
  return make_tuple(event.light, error);
}

DynamicJsonDocument readLuminosity()
{
  tuple<float, char *> luminosity_error = readTSL2561();
  return buildDocument((char *)"tsl2561", (char *)"luminosity", (char *)"lux", get<0>(luminosity_error), get<1>(luminosity_error));
}

void loop()
{
  tuple<DynamicJsonDocument, DynamicJsonDocument> docs = readECO2AndTVOC();
  DynamicJsonDocument doc_temp = readTemperature();
  DynamicJsonDocument doc_hum = readHumidity();
  DynamicJsonDocument doc_luminosity = readLuminosity();
  char buf[256];
  serializeJson(get<0>(docs), buf);
  Serial.println(buf);
  serializeJson(get<1>(docs), buf);
  Serial.println(buf);
  serializeJson(doc_temp, buf);
  Serial.println(buf);
  serializeJson(doc_hum, buf);
  Serial.println(buf);
  serializeJson(doc_luminosity, buf);
  Serial.println(buf);
  delay(1000);
}
