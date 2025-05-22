/**
 * @file dht22.c
 * @brief Implementation of DHT22 temperature and humidity sensor interface.
 */

#include "dht22.h"

#include <stdint.h>
#include <util/delay.h>

#include "pin.h"
#include "strfmt.h"
#include "timebase.h"

// --- Macros ---

#define HIGH_BYTE(x) \
  (((x) >> 8) & 0xFF)            /**< Extracts high byte from 16-bit value */
#define LOW_BYTE(x) ((x) & 0xFF) /**< Extracts low byte from 16-bit value */
#define BIT_READ(value, bit) \
  (((value) >> (bit)) & 0x01) /**< Reads specific bit from value */
#define COMPUTE_CRC()                                                         \
  ((uint8_t)(HIGH_BYTE(_h16bits) + LOW_BYTE(_h16bits) + HIGH_BYTE(_t16bits) + \
             LOW_BYTE(_t16bits)) ==                                           \
   _crc8bits) /**< Verifies CRC of read data */

#define DEBUG_BUFFER_SIZE 256 /**< Size of debug string buffer */
#define T 30                  /**< Timing tolerance in microseconds */
#define cSamplingTime \
  2100 /**< Minimum interval between sensor samples in milliseconds */

// --- Static Sensor State ---

static uint8_t _pinData;
static uint32_t _timer;
static uint64_t _rawData;
static uint16_t _h16bits;
static uint16_t _t16bits;
static uint8_t _crc8bits;
static uint8_t _firstStart = 1;
static enum error _lastError;

static uint8_t _timing80L = 80;
static uint8_t _timing80H = 80;
static uint8_t _timing50 = 50;
static uint8_t _timingBit0 = 27;
static uint8_t _timingBit1 = 70;

static float _cachedTemperature = -273.0f;
static float _cachedHumidity = -1.0f;
static uint8_t _dataIsFresh = 0;

// --- Initialization ---

/**
 * @brief Initializes the DHT22 sensor with the specified GPIO pin.
 * @param pinData The GPIO pin connected to the DHT22 data line.
 */
void dht22Init(uint8_t pinData) {
  timebaseInit();
  _pinData = pinData;
}

// --- Timing Calibration ---

/**
 * @brief Resets timing values used in bit measurement.
 */
static void resetTimings() {
  _timingBit0 = 0;
  _timingBit1 = 0;
}

/**
 * @brief Measures and calibrates timing values used to interpret DHT22 signals.
 */
void measureTimings() {
  if (!_firstStart && (millis() - _timer) < cSamplingTime)
    return;

  _timer = millis();
  _firstStart = 0;
  resetTimings();

  pinMode(_pinData, OUTPUT);
  digitalWrite(_pinData, LOW);
  _delay_ms(2);
  digitalWrite(_pinData, HIGH);
  pinMode(_pinData, INPUT);

  uint32_t t = micros();
  uint32_t m = 0;

  while (digitalRead(_pinData) == 1) {
    if ((micros() - t) > 60)
      return;
  }

  t = micros();
  while (digitalRead(_pinData) == 0) {
    m = micros() - t;
    if (m > 100)
      return;
  }
  _timing80L = m;

  t = micros();
  while (digitalRead(_pinData) == 1) {
    m = micros() - t;
    if (m > 100)
      return;
  }
  _timing80H = m;

  t = micros();
  while (digitalRead(_pinData) == 0) {
    m = micros() - t;
    if (m > 60)
      return;
  }
  _timing50 = m;

  t = micros();
  while (_timingBit0 == 0 || _timingBit1 == 0) {
    while (digitalRead(_pinData) == 1) {
      m = micros() - t;
      if (m > 100)
        return;
    }

    if (m > 40)
      _timingBit1 = m;
    else
      _timingBit0 = m;

    t = micros();
    while (digitalRead(_pinData) == 0) {
      m = micros() - t;
      if (m > 100)
        return;
    }
    t = micros();
  }
}

// --- Sensor Reading ---

/**
 * @brief Reads raw 40-bit data from the DHT22 sensor.
 * @return Error code if any timing errors occurred, otherwise OK.
 */
static uint8_t readRawSensor() {
  pinMode(_pinData, OUTPUT);
  digitalWrite(_pinData, LOW);
  _delay_ms(2);
  digitalWrite(_pinData, HIGH);
  pinMode(_pinData, INPUT);

  int32_t t = pulseIn(_pinData, HIGH, 250);
  if (t == 0)
    return (_lastError = ERR_TIMING_80);

  _rawData = 0;

  for (uint8_t i = 0; i < 40; i++) {
    t = micros();
    while (digitalRead(_pinData) == LOW) {
      if ((micros() - t) > (_timing50 + T))
        return (_lastError = ERR_TIMING_50);
    }

    _delay_us(40);
    if (digitalRead(_pinData) == 1)
      _rawData++;

    if (i != 39)
      _rawData <<= 1;

    t = micros();
    while (digitalRead(_pinData) == 1) {
      if ((micros() - t) > _timingBit1)
        return (_lastError = ERR_TIMING_BITS);
    }
  }

  _delay_us(10);
  pinMode(_pinData, OUTPUT);
  digitalWrite(_pinData, HIGH);

  return OK;
}

/**
 * @brief Decodes raw 40-bit sensor data into temperature and humidity values.
 * @return OK if successful, otherwise CRC error.
 */
static uint8_t decodeRawSensor() {
  _h16bits = _rawData >> 24;
  _t16bits = _rawData >> 8;
  _crc8bits = _rawData;

  uint8_t crcOk = COMPUTE_CRC();

  if (crcOk) {
    _cachedHumidity = _h16bits / 10.0f;
    _cachedTemperature = (BIT_READ(_t16bits, 15)) ? (_t16bits & 0x7FFF) / -10.0f
                                                  : _t16bits / 10.0f;
  } else {
    _cachedHumidity = -1.0f;
    _cachedTemperature = -273.0f;
  }

  return (_lastError = crcOk ? OK : ERR_CRC);
}

// --- Sampling Logic ---
/**
 * @brief Samples sensor if required and updates cached values.
 * @return Last encountered error during sampling.
 */
static uint8_t sampleIfNeeded() {
  if (_dataIsFresh && (millis() - _timer) < cSamplingTime)
    return _lastError;

  _dataIsFresh = 0;
  _timer = millis();
  _firstStart = 0;

  uint8_t err = readRawSensor();
  if (err != OK)
    return (_lastError = err);

  err = decodeRawSensor();
  _dataIsFresh = 1;

  return (_lastError = err);
}

// --- API ---

/**
 * @brief Reads temperature, humidity, and/or raw data from the DHT22.
 * @param [out] outTemp Pointer to float to store temperature (nullable).
 * @param [out] outHum Pointer to float to store humidity (nullable).
 * @param [out] outRaw Pointer to uint64_t to store raw data (nullable).
 * @return Error code from the read operation.
 */
uint8_t dht22Read(float* outTemp, float* outHum, uint64_t* outRaw) {
  uint8_t err = sampleIfNeeded();

  if (outTemp)
    *outTemp = _cachedTemperature;
  if (outHum)
    *outHum = _cachedHumidity;
  if (outRaw)
    *outRaw = _rawData;

  return err;
}

/**
 * @brief Gets the last error code encountered by the sensor.
 * @return Error code.
 */
uint8_t dht22getLastError() {
  return _lastError;
}

/**
 * @brief Provides a formatted string with debug information about the sensor state.
 * @return Pointer to static string buffer with debug info.
 */
const char* dht22Debug() {
  static char buffer[DEBUG_BUFFER_SIZE];
  char* p = buffer;

  _dataIsFresh = 0;
  sampleIfNeeded();

  p += appendLine(p, "### BEGIN DEBUG ###", 1, 0);
  p += appendLine(p, "look at datasheet for timing specs", 1, 0);
  p += appendLine(p, "t_80L\tt_80H\tt_50\tt_Bit0\tt_Bit1", 1, 0);

  p += appendUInt(p, _timing80L, 0);
  *p++ = '\t';
  p += appendUInt(p, _timing80H, 0);
  *p++ = '\t';
  p += appendUInt(p, _timing50, 0);
  *p++ = '\t';
  p += appendUInt(p, _timingBit0, 0);
  *p++ = '\t';
  p += appendUInt(p, _timingBit1, 0);
  p += appendLine(p, "", 1, 0);

  p += appendLine(p, "error : ", 0, 0);
  p += appendUInt(p, _lastError, 0);
  p += appendLine(p, "", 1, 0);

  p += appendBinary40(p, _rawData, 0);
  p += appendLine(p, "", 1, 0);

  p += appendLine(p, "h\tt\tcrc", 1, 0);
  p += appendFloat(p, _cachedHumidity, 1, 0);
  *p++ = '\t';
  p += appendFloat(p, _cachedTemperature, 1, 0);
  *p++ = '\t';
  p += appendLine(p, _lastError != ERR_CRC ? "TRUE" : "FALSE", 0, 0);
  p += appendLine(p, "", 1, 0);

  p += appendLine(p, "### END DEBUG ###", 1, 0);

  *p = '\0';
  return buffer;
}
