/**
 * @file main.h
 * @brief Header for environmental regulator controller.
 *
 * This header defines constants, pin configurations, data types, and function
 * prototypes for the embedded system controlling temperature and humidity
 * monitoring, along with fan and buzzer control.
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

/** @defgroup KeypadPins Keypad pin definitions
 * Defines microcontroller pins connected to keypad rows and columns.
 */
/**@{*/
#define R1 9 /**< Keypad Row 1 pin */
#define R2 8 /**< Keypad Row 2 pin */
#define C1 5 /**< Keypad Column 1 pin */
#define C2 4 /**< Keypad Column 2 pin */
#define C3 3 /**< Keypad Column 3 pin */
#define C4 2 /**< Keypad Column 4 pin */
/**@}*/

/** @defgroup KeypadConfig Keypad configuration
 * Defines number of rows and columns in keypad matrix.
 */
/**@{*/
#define ROWS 2 /**< Number of keypad rows */
#define COLS 4 /**< Number of keypad columns */
/**@}*/

/** @defgroup LCDConfig LCD configuration
 * Defines LCD I2C address and dimensions.
 */
/**@{*/
#define I2C_PORT 0x27  /**< I2C address for LCD display */
#define LCD_COLUMNS 16 /**< Number of columns in LCD */
#define LCD_ROWS 2     /**< Number of rows in LCD */
/**@}*/

/** @defgroup PeripheralPins Peripheral pin configuration
 * Defines pins used for fan PWM, buzzer control, and DHT22 sensor.
 */
/**@{*/
#define PWM 10  /**< PWM pin for DC fan speed control */
#define BUZZ 11 /**< Digital pin to control buzzer */
#define DHT 12  /**< Digital pin connected to DHT22 sensor data line */
/**@}*/

/** @defgroup SystemConstants System timing and limits
 * Defines timing intervals, delays, default thresholds, and parameter limits.
 */
/**@{*/
#define DEBUG_SERIAL_BAUDRATE 9600 /**< Baud rate for USART debug output */
#define DEBOUNCE_TIME 100          /**< Debounce time for keypad in ms */
#define BUZZER_DELAY 200           /**< Duration buzzer is active in ms */
#define DELAY_IN_SETUP 1000        /**< General delay in setup routine in ms */
#define DHT_UPDATE_INTERVAL \
  2000 /**< Interval to update DHT sensor readings (ms) */
#define FAN_SPEED_UPDATE_INTERVAL 100 /**< Interval to update fan speed (ms) */

#define DEFAULT_HUMIDITY_MIN 30 /**< Default minimum humidity threshold (%) */
#define DEFAULT_HUMIDITY_MAX 80 /**< Default maximum humidity threshold (%) */
#define DEFAULT_TEMPERATURE_MIN \
  26 /**< Default minimum temperature threshold (°C) */

#define HUMIDITY_MIN_LOWER_LIMIT \
  10 /**< Lower bound for humidity min parameter */
#define HUMIDITY_MIN_UPPER_LIMIT \
  50 /**< Upper bound for humidity min parameter */
#define HUMIDITY_MAX_LOWER_LIMIT \
  60 /**< Lower bound for humidity max parameter */
#define HUMIDITY_MAX_UPPER_LIMIT \
  90 /**< Upper bound for humidity max parameter */
#define TEMPERATURE_MIN_LOWER_LIMIT \
  16 /**< Lower bound for temperature min parameter */
#define TEMPERATURE_MIN_UPPER_LIMIT \
  36 /**< Upper bound for temperature min parameter */

#define HUMIDITY_STEP 10 /**< Step size for adjusting humidity parameters (%) */
#define TEMPERATURE_STEP \
  1 /**< Step size for adjusting temperature parameter (°C) */
/**@}*/

/**
 * @brief Operational states of the system.
 *
 * Enumerates the different states for system behavior:
 * - NORMAL_DISPLAY: Regular operation showing sensor data.
 * - DEBUG_MODE: Serial debug output enabled.
 * - FAN_TEST: Fan test mode active.
 * - BUZZER_TEST: Buzzer test mode active.
 */
typedef enum {
  NORMAL_DISPLAY = 0,
  DEBUG_MODE,
  FAN_TEST,
  BUZZER_TEST
} ActionState;

/**
 * @brief Activates buzzer if humidity is out of acceptable range.
 * @param humidity Current humidity value (%).
 * @return 1 if buzzer was activated, 0 otherwise.
 */
uint8_t buzzControl(uint8_t humidity);

/**
 * @brief Controls DC fan speed based on temperature input.
 * @param temperature Current temperature value (°C).
 * @return Fan speed as a percentage (0-100%).
 */
uint8_t dcFanControl(uint8_t temperature);

/**
 * @brief Updates LCD display with current sensor readings or system info.
 */
void display(void);

/**
 * @brief Handles debug mode display and serial output.
 */
void debugMode(void);

/**
 * @brief Main action function that runs periodically based on current system state.
 */
void action(void);

/**
 * @brief Handles keypad input events and triggers corresponding actions.
 */
void keypadEvent(void);

/**
 * @brief Interactive mode for setting threshold parameters via keypad.
 */
void setParameters(void);

/**
 * @brief Initializes hardware and peripherals.
 */
void init(void);

/**
 * @brief Setup routine executed once on startup.
 */
void setup(void);

/**
 * @brief Main loop function (typically runs indefinitely).
 */
void loop(void);

#endif  // MAIN_H_
