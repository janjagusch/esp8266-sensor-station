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
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <tuple>
#include <WiFiUdp.h>
#include <string>
using namespace std;
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "wifi_secrets.h"
#include "mqtt_secrets.h"
#include "sensor_station.h"

const long utcOffsetInSeconds = 3600;

std::string MQTT_CLIENT_PREFIX = "sensor_station_";
std::string STATION_ID_STR = STATION_ID;
std::string MQTT_CLIENT_ID_STR = MQTT_CLIENT_PREFIX + STATION_ID_STR;
char *MQTT_CLIENT_ID = &MQTT_CLIENT_ID_STR[0];

CCS811 ccs(CCS811_NWAKE_PIN, CCS811_SLAVE_ADDR);
DHT dht(DHT22_DATA_PIN, DHT_TYPE);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR, TSL2561_SENSOR_ID);

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

void setup_wifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }
  Serial.println(WiFi.localIP());
}

void setup_mqtt()
{
  client.setServer(MQTT_BROKER, 1883);
}

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
  setup_wifi();
  setup_mqtt();
}

DynamicJsonDocument buildDocument(char *sensor, char *measure, char *unit, float value, char *error, unsigned long timestamp)
{
  DynamicJsonDocument doc(1024);
  doc["sensor"] = sensor;
  doc["measurement"] = measure;
  doc["unit"] = unit;
  doc["value"] = value;
  doc["error"] = error;
  doc["station"] = STATION_ID;
  doc["timestamp"] = timestamp;
  return doc;
}

DynamicJsonDocument readHumidity(unsigned long timestamp)
{
  float value = dht.readHumidity();
  char *error = (char *)"";
  if (isnan(value))
  {
    error = (char *)"Error";
  }
  return buildDocument((char *)"dht22", (char *)"humidity", (char *)"%", value, error, timestamp);
}

DynamicJsonDocument readTemperature(unsigned long timestamp)
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
  return buildDocument((char *)"dht22", (char *)"temperature", unit, value, error, timestamp);
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

DynamicJsonDocument buildECO2Document(float value, char *error, unsigned long timestamp)
{
  return buildDocument((char *)"ccs811", (char *)"eco2", (char *)"ppm", value, error, timestamp);
}

DynamicJsonDocument buildTVOCDocument(float value, char *error, unsigned long timestamp)
{
  return buildDocument((char *)"ccs811", (char *)"tvoc", (char *)"ppb", value, error, timestamp);
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

DynamicJsonDocument readECO2(unsigned long timestamp)
{
  tuple<float, float, char *> eco2_tvoc_error = readCCS811();
  return buildECO2Document(get<0>(eco2_tvoc_error), get<2>(eco2_tvoc_error), timestamp);
}

DynamicJsonDocument readTVOC(unsigned long timestamp)
{
  tuple<float, float, char *> eco2_tvoc_error = readCCS811();
  return buildTVOCDocument(get<1>(eco2_tvoc_error), get<2>(eco2_tvoc_error), timestamp);
}

tuple<DynamicJsonDocument, DynamicJsonDocument> readECO2AndTVOC(unsigned long timestamp)
{
  tuple<float, float, char *> eco2_tvoc_error = readCCS811();
  return make_tuple(
      buildECO2Document(get<0>(eco2_tvoc_error), get<2>(eco2_tvoc_error), timestamp),
      buildTVOCDocument(get<1>(eco2_tvoc_error), get<2>(eco2_tvoc_error), timestamp));
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

DynamicJsonDocument readLuminosity(unsigned long timestamp)
{
  tuple<float, char *> luminosity_error = readTSL2561();
  return buildDocument((char *)"tsl2561", (char *)"luminosity", (char *)"lux", get<0>(luminosity_error), get<1>(luminosity_error), timestamp);
}

void publishDocument(DynamicJsonDocument doc, string topic)
{
  // Append STATION_ID to topic
  topic += "/" + STATION_ID_STR;
  char *topic_arr = &topic[0];
  // Build payload
  char payload[256];
  serializeJson(doc, payload);
  // Publish
  client.publish(topic_arr, payload);
}

void mqtt_connect()
{
  if (!client.connected())
  {
    while (!client.connected())
    {
      client.connect(MQTT_CLIENT_ID);
      delay(100);
    }
  }
}

void loop()
{
  mqtt_connect();
  timeClient.update();
  unsigned long timestamp = timeClient.getEpochTime();
  tuple<DynamicJsonDocument, DynamicJsonDocument> docs_eco2_tvoc = readECO2AndTVOC(timestamp);
  DynamicJsonDocument doc_temperature = readTemperature(timestamp);
  DynamicJsonDocument doc_humidity = readHumidity(timestamp);
  DynamicJsonDocument doc_luminosity = readLuminosity(timestamp);
  publishDocument(doc_luminosity, (char *)"apartment/sensors/response/tsl2561/luminosity");
  publishDocument(doc_humidity, (char *)"apartment/sensors/response/dht22/humidity");
  publishDocument(doc_temperature, (char *)"apartment/sensors/response/dht22/temperature");
  publishDocument(get<0>(docs_eco2_tvoc), (char *)"apartment/sensors/response/ccs811/eco2");
  publishDocument(get<1>(docs_eco2_tvoc), (char *)"apartment/sensors/response/ccs811/tvoc");
  delay(STATION_READ_INTERVAL);
}
