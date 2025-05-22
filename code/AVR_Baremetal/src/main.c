/**
 * @file main.c
 * @brief Environmental regulator controller using DHT22 sensor, fan, and buzzer.
 *
 * This embedded application monitors environmental parameters such as temperature
 * and humidity using the DHT22 sensor. It controls a DC fan speed via PWM and
 * activates a buzzer based on threshold parameters. The system provides a user
 * interface through a keypad and an LCD display and supports serial debugging
 * through USART communication.
 */

#include "main.h"

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#include "dht22.h"
#include "keypad.h"
#include "lcd.h"
#include "pin.h"
#include "strfmt.h"
#include "tasks.h"
#include "timebase.h"
#include "usart.h"

// Keypad layout definition: 2 rows x 4 columns
static const char keymap[ROWS][COLS] = {
    {'1', '2', '3', '4'},
    {'5', '6', '7', '8'},
};

// GPIO pins connected to keypad rows
static const uint8_t rowPins[ROWS] = {R1, R2};

// GPIO pins connected to keypad columns
static const uint8_t colPins[COLS] = {C1, C2, C3, C4};

// Current system action state (normal display, debug, tests)
ActionState action_state = NORMAL_DISPLAY;

// Flag controlling LCD display mode (temperature/humidity or fan/buzzer info)
uint8_t display_state = 1;

// Flag enabling/disabling debug output via USART
uint8_t debug_enable = 0;

// Threshold parameters for humidity and temperature
uint8_t humidity_min = DEFAULT_HUMIDITY_MIN;       /**< Min humidity */
uint8_t humidity_max = DEFAULT_HUMIDITY_MAX;       /**< Max humidity */
uint8_t temperature_min = DEFAULT_TEMPERATURE_MIN; /**< Min temperature */

// Define periodic tasks and their intervals (in milliseconds)
DEFINE_TASKS({action, DHT_UPDATE_INTERVAL, 0}, {keypadEvent, 0, 0}, );

// Buffer for formatted output strings
static char buffer[16];

/**
 * @brief Controls the buzzer based on humidity thresholds.
 *
 * If the humidity is outside the specified range [humidity_min, humidity_max],
 * the buzzer will beep once to alert the user.
 *
 * @param humidity Current humidity value (%).
 * @return 1 if buzzer was activated, otherwise 0.
 */
uint8_t buzzControl(uint8_t humidity) {
  if (humidity <= humidity_min || humidity >= humidity_max) {
    digitalWrite(BUZZ, LOW);   // Turn buzzer ON (active low assumed)
    _delay_ms(BUZZER_DELAY);   // Keep buzzer on for delay period
    digitalWrite(BUZZ, HIGH);  // Turn buzzer OFF
    return 1;
  }
  return 0;
}

/**
 * @brief Controls DC fan speed proportionally to temperature.
 *
 * If temperature is below the minimum threshold, the fan is stopped.
 * If temperature is above minimum + 4°C, fan runs at full speed.
 * Otherwise, fan speed is scaled linearly between 0 and 100%.
 *
 * @param temperature Current temperature value (°C).
 * @return Fan speed as percentage (0 to 100).
 */
uint8_t dcFanControl(uint8_t temperature) {
  if (temperature < temperature_min) {
    analogWrite(PWM, 0);  // Fan off
    _delay_ms(FAN_SPEED_UPDATE_INTERVAL);
    return 0;
  }

  // Calculate fan speed linearly between temperature_min and temperature_min + 4
  uint8_t fanSpeed = (temperature >= (temperature_min + 4))
                         ? 100
                         : (temperature - (temperature_min - 1)) * 20;

  // Set PWM duty cycle accordingly (0-255 scale)
  analogWrite(PWM, fanSpeed * 255 / 100);

  _delay_ms(FAN_SPEED_UPDATE_INTERVAL);
  return fanSpeed;
}

/**
 * @brief Updates the LCD display with current temperature and humidity or
 * fan speed and buzzer status depending on the display state.
 *
 * If DHT22 sensor reports an error, display the error code and stop fan and buzzer.
 */
void display(void) {
  float temperature, humidity;
  dht22ReadTH(&temperature, &humidity);

  // Check for sensor error and display error message
  if (dht22getLastError() != 0) {
    analogWrite(PWM, 0);       // Turn off fan
    digitalWrite(BUZZ, HIGH);  // Turn off buzzer
    lcdClear();
    lcdHome();
    lcdPrint("DHT22:");
    lcdSetCursor(0, 1);
    lcdPrint("Sensor Error ");
    appendUInt(buffer, dht22getLastError(), 1);
    lcdPrint(buffer);
    return;
  }

  // Control buzzer and fan based on sensor readings
  uint8_t buzz = buzzControl((uint8_t)humidity);
  uint8_t fanSpeed = dcFanControl((uint8_t)temperature);

  lcdClear();
  lcdHome();

  if (display_state) {
    // Display temperature and humidity
    lcdPrint("Temp: ");
    appendFloat(buffer, temperature, 1, 1);
    lcdPrint(buffer);
    lcdPrint(" ");
    lcdPrintChar((char)223);  // Degree symbol
    lcdPrint("C");

    lcdSetCursor(0, 1);
    lcdPrint("Hum: ");
    appendFloat(buffer, humidity, 1, 1);
    lcdPrint(buffer);
    lcdPrint(" %");
  } else {
    // Display fan speed and buzzer status
    lcdPrint("Fan speed: ");
    appendUInt(buffer, fanSpeed, 1);
    lcdPrint(buffer);
    lcdPrint(" %");

    lcdSetCursor(0, 1);
    lcdPrint("Buzz: ");
    lcdPrint(buzz ? "On" : "Off");
  }
}

/**
 * @brief Displays debug information on LCD and sends detailed debug strings
 * over USART serial interface.
 *
 * This mode is activated when debug_enable flag is set.
 */
void debugMode(void) {
  lcdClear();
  lcdHome();
  lcdPrint("DHT22 Debug mode");

  lcdSetCursor(0, 1);
  lcdPrint("Serial: ");
  appendUInt(buffer, DEBUG_SERIAL_BAUDRATE, 1);
  lcdPrint(buffer);
  lcdPrint(" b/s");

  // Print sensor debug info to serial
  usartPrintln(dht22Debug());
}

/**
 * @brief Executes system action depending on the current mode.
 *
 * Supports normal display, debug mode, and hardware tests (fan/buzzer).
 * Updates peripherals accordingly.
 */
void action(void) {
  switch (action_state) {
    case NORMAL_DISPLAY:
      // Normal display or debug output depending on debug_enable flag
      if (debug_enable) {
        debugMode();
      } else {
        display();
      }
      break;

    case DEBUG_MODE:
      // Initialize or stop USART debug communication
      if (debug_enable) {
        usartBegin(DEBUG_SERIAL_BAUDRATE);
      } else {
        usartEnd();
      }
      digitalWrite(BUZZ, HIGH);       // Turn off buzzer
      analogWrite(PWM, 0);            // Stop fan
      action_state = NORMAL_DISPLAY;  // Return to normal display
      action();                       // Re-run action with updated state
      break;

    default:
      // Test modes: display hardware test info and control buzzer or fan
      lcdClear();
      lcdHome();
      lcdPrint("Testing:");
      lcdSetCursor(0, 1);
      lcdPrint(action_state == FAN_TEST ? "DC fan" : "Buzzer");

      digitalWrite(BUZZ, action_state == BUZZER_TEST ? LOW : HIGH);
      analogWrite(PWM, action_state == FAN_TEST ? 255 : 0);
      break;
  }
}

/**
 * @brief Handles keypad input events.
 *
 * Maps keypad keys to system actions such as switching display mode,
 * enabling debug mode, entering parameter adjustment, or hardware tests.
 */
void keypadEvent(void) {
  char key = keypadScan();
  if (keypadState(&key) == KEY_PRESSED) {
    switch (key) {
      case '1':
        action_state = DEBUG_MODE;
        debug_enable = 0;
        display_state = 1;
        break;
      case '2':
        action_state = DEBUG_MODE;
        debug_enable = 0;
        display_state = 0;
        break;
      case '3':
        action_state =
            (action_state == NORMAL_DISPLAY || action_state == BUZZER_TEST)
                ? FAN_TEST
                : NORMAL_DISPLAY;
        break;
      case '4':
        action_state =
            (action_state == NORMAL_DISPLAY || action_state == FAN_TEST)
                ? BUZZER_TEST
                : NORMAL_DISPLAY;
        break;
      case '5':
        debug_enable = !debug_enable;
        action_state = DEBUG_MODE;
        break;
      case '6':
        action_state = DEBUG_MODE;
        debug_enable = 0;
        setParameters();
        break;
      case '8':
        lcdClear();
        return;
      default:
        return;
    }
    action();
  }
}

/**
 * @brief Interactive parameter adjustment via keypad.
 *
 * Allows the user to modify humidity and temperature thresholds,
 * reset parameters to defaults, or exit adjustment mode.
 */
void setParameters(void) {
  analogWrite(PWM, 0);       // Turn off fan while adjusting
  digitalWrite(BUZZ, HIGH);  // Turn off buzzer

DISPLAY:
  lcdClear();

  // Display current thresholds on LCD
  lcdSetCursor(2, 0);
  appendFloat(buffer, humidity_min, 1, 1);
  lcdPrint(buffer);

  lcdSetCursor(7, 0);
  appendFloat(buffer, humidity_max, 1, 1);
  lcdPrint(buffer);

  lcdSetCursor(12, 0);
  appendFloat(buffer, temperature_min, 1, 1);
  lcdPrint(buffer);

  while (1) {
    char key_parameters = keypadScan();

    if (keypadState(&key_parameters) == KEY_PRESSED) {
      switch (key_parameters) {
        case '1':
          if (humidity_min < HUMIDITY_MIN_UPPER_LIMIT)
            humidity_min += HUMIDITY_STEP;
          break;
        case '5':
          if (humidity_min > HUMIDITY_MIN_LOWER_LIMIT)
            humidity_min -= HUMIDITY_STEP;
          break;
        case '2':
          if (humidity_max < HUMIDITY_MAX_UPPER_LIMIT)
            humidity_max += HUMIDITY_STEP;
          break;
        case '6':
          if (humidity_max > HUMIDITY_MAX_LOWER_LIMIT)
            humidity_max -= HUMIDITY_STEP;
          break;
        case '3':
          if (temperature_min < TEMPERATURE_MIN_UPPER_LIMIT)
            temperature_min += TEMPERATURE_STEP;
          break;
        case '7':
          if (temperature_min > TEMPERATURE_MIN_LOWER_LIMIT)
            temperature_min -= TEMPERATURE_STEP;
          break;
        case '4':
          // Reset to defaults
          humidity_min = DEFAULT_HUMIDITY_MIN;
          humidity_max = DEFAULT_HUMIDITY_MAX;
          temperature_min = DEFAULT_TEMPERATURE_MIN;
          break;
        case '8':
          // Exit parameter setting mode
          return;
      }

      goto DISPLAY;  // Refresh display with updated values
    }
  }
}

/**
 * @brief Initializes hardware peripherals and modules.
 *
 * Sets up the timebase timer, initializes the DHT22 sensor,
 * initializes the LCD display, and sets up the keypad input system.
 */
void init(void) {
  timebaseInit();
  dht22Init(DHT);
  lcdInit(I2C_PORT, LCD_COLUMNS, LCD_ROWS);
  keypadInit(MAPKEYPAD(keymap), rowPins, colPins, ROWS, COLS);
  setDebounceTime(100);
}

/**
 * @brief Setup routine executed once after reset.
 *
 * Configures GPIO pins for buzzer, PWM fan control, and sensor input.
 * Initializes LCD with welcome message and performs startup buzzer and fan tests.
 */
void setup(void) {
  pinMode(BUZZ, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DHT, INPUT);

  lcdHome();
  lcdPrint("Temp & Hum");
  lcdSetCursor(0, 1);
  lcdPrint("Regulator");

  // Startup buzzer and fan test
  analogWrite(PWM, 255);
  _delay_ms(BUZZER_DELAY);
  digitalWrite(BUZZ, HIGH);
  _delay_ms(BUZZER_DELAY);
  analogWrite(PWM, 0);

  _delay_ms(2000);
}

/**
 * @brief Main application entry point.
 *
 * Initializes hardware, runs startup routines, and then continuously runs
 * scheduled tasks including reading keypad inputs and updating system actions.
 */
int main(void) {
  init();
  setup();

  while (1) {
    tasksRun();  // Run scheduled tasks (action, keypad event, etc.)
  }

  return 0;
}
