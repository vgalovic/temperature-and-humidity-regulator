/**
 * @file dht22.h
 * @brief Interface for reading temperature and humidity from the DHT22 sensor.
 */

#ifndef DHT22_H_
#define DHT22_H_

#include <stdint.h>

/**
 * @name DHT22 Read Macros
 * @brief Convenient macros for reading specific values from the DHT22 sensor.
 */
/**@{*/
#define dht22ReadTemperature(temp) \
  dht22Read((temp), 0, 0) /**< Read only temperature */
#define dht22ReadHumidity(hum) dht22Read(0, (hum), 0) /**< Read only humidity */
#define dht22ReadRawData(raw) \
  dht22Read(0, 0, (raw)) /**< Read raw 64-bit data */
#define dht22ReadTH(t, h) \
  dht22Read((t), (h), 0) /**< Read both temperature and humidity */
#define dht22Refresh() \
  dht22Read(0, 0, 0) /**< Trigger a sensor read with no output */
/**@}*/

/**
 * @enum error
 * @brief Error codes returned by DHT22 operations.
 */
enum error {
  OK,              /**< No error */
  ERR_TIMING_80,   /**< Timing error during 80us low signal */
  ERR_TIMING_50,   /**< Timing error during 50us high signal */
  ERR_TIMING_BITS, /**< Timing error while reading bits */
  ERR_CRC          /**< CRC check failed */
};

/**
 * @brief Initializes the DHT22 sensor on the specified pin.
 *
 * @param pinData GPIO pin connected to DHT22 data line.
 */
void dht22Init(uint8_t pinData);

/**
 * @brief Reads data from the DHT22 sensor.
 *
 * @param [out] outTemp Pointer to store temperature in °C (nullable).
 * @param [out] outHum Pointer to store humidity percentage (nullable).
 * @param [out] outRaw Pointer to store raw 64-bit sensor data (nullable).
 * @return Error code (see @ref error).
 */
uint8_t dht22Read(float* outTemp, float* outHum, uint64_t* outRaw);

/**
 * @brief Returns the last error encountered during a DHT22 read.
 *
 * @return Last error code (see @ref error).
 */
uint8_t dht22getLastError(void);

/**
 * @brief Returns a string representation of the last DHT22 error.
 *
 * @return Constant string describing the last error.
 */
const char* dht22Debug(void);

#endif  // DHT22_H_
