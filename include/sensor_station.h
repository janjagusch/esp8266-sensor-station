#define STATION_ID "1"

#define STATION_READ_INTERVAL 60000

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
