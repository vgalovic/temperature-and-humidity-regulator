#include "keypad.h"

#include <avr/io.h>

#include "pin.h"
#include "timebase.h"

// Sentinel value indicating no valid row selected
#define INVALID_ROW 0xFF

// --- Static Variables ---
// Number of rows and columns in the keypad matrix
static uint8_t _rows = 0, _cols = 0;

// Pointers to user configuration data
static const char *_keymap;       // User-defined keypad layout string
static const uint8_t *_row_pins;  // GPIO pins connected to rows
static const uint8_t *_col_pins;  // GPIO pins connected to columns

// Debounce duration in milliseconds (default 50ms)
static uint16_t debounce_time = 50;

/**
 * @brief Initializes keypad hardware and software state.
 *
 * Configures row pins as outputs and sets them HIGH (inactive),
 * configures column pins as inputs with internal pull-up resistors enabled.
 *
 * @param keymap    Pointer to a character array defining key layout.
 * @param row_pins  Pointer to array of row GPIO pin numbers.
 * @param col_pins  Pointer to array of column GPIO pin numbers.
 * @param rows      Number of rows in keypad matrix.
 * @param cols      Number of columns in keypad matrix.
 */
void keypadInit(const char *keymap, const uint8_t *row_pins,
                const uint8_t *col_pins, uint8_t rows, uint8_t cols) {
  _keymap = keymap;
  _rows = rows;
  _cols = cols;
  _row_pins = row_pins;
  _col_pins = col_pins;

  timebaseInit();

  // Initialize columns as inputs with internal pull-ups enabled
  for (uint8_t i = 0; i < _cols; i++) {
    pinMode(_col_pins[i], INPUT);
    digitalWrite(_col_pins[i], HIGH);
  }

  // Initialize rows as outputs and set them HIGH (inactive)
  for (uint8_t i = 0; i < _rows; i++) {
    pinMode(_row_pins[i], OUTPUT);
    digitalWrite(_row_pins[i], HIGH);
  }
}

/**
 * @brief Sets the debounce time to avoid false key detection.
 *
 * @param time Debounce interval in milliseconds.
 */
void setDebounceTime(uint16_t time) {
  debounce_time = time;
}

/**
 * @brief Scans the keypad for a pressed key using row-column scanning.
 *
 * Activates each row sequentially, reading columns to detect key presses.
 * Implements software debounce based on elapsed time and key changes.
 *
 * @return Character of pressed key or NO_KEY if none detected.
 */
char keypadScan(void) {
  static uint8_t last_row = INVALID_ROW;   // Last row activated
  static uint32_t last_debounce_time = 0;  // Last debounce timestamp
  static char last_key = NO_KEY;           // Last key detected

  for (uint8_t row = 0; row < _rows; row++) {
    // Reset previous row (set HIGH to deactivate)
    if (last_row != INVALID_ROW)
      digitalWrite(_row_pins[last_row], HIGH);

    // Activate current row (set LOW)
    digitalWrite(_row_pins[row], LOW);
    last_row = row;

    // Check each column for LOW signal indicating key press
    for (uint8_t col = 0; col < _cols; col++) {
      if (digitalRead(_col_pins[col]) == LOW) {
        char key = _keymap[row * _cols + col];
        uint32_t now = millis();

        // Debounce: register key only if changed or debounce time elapsed
        if (key != last_key || (now - last_debounce_time) >= debounce_time) {
          last_debounce_time = now;
          last_key = key;
          return key;
        } else {
          // Key held within debounce window, ignore
          return NO_KEY;
        }
      }
    }
  }

  // No key pressed detected, reset state
  last_key = NO_KEY;
  return NO_KEY;
}

/**
 * @brief Detects and reports keypad events: press, release, or idle.
 *
 * Maintains internal state of last key to detect transitions.
 *
 * @param key Pointer to store detected key character.
 * @return Keypad event type: KEY_PRESSED, KEY_RELEASED, or KEY_IDLE.
 */
keypad_event_t keypadState(char *key) {
  static char last_key = NO_KEY;
  char current_key = NO_KEY;

  // Scan the keypad matrix row-by-row for current key press
  for (uint8_t row = 0; row < _rows; row++) {
    // Set all rows inactive (HIGH) before activating one
    for (uint8_t r = 0; r < _rows; r++) digitalWrite(_row_pins[r], HIGH);

    // Activate current row
    digitalWrite(_row_pins[row], LOW);

    // Check each column for key press
    for (uint8_t col = 0; col < _cols; col++) {
      if (digitalRead(_col_pins[col]) == LOW) {
        current_key = _keymap[row * _cols + col];
        break;
      }
    }

    // Deactivate current row
    digitalWrite(_row_pins[row], HIGH);

    if (current_key != NO_KEY)
      break;
  }

  *key = current_key;

  // Detect key press event
  if (current_key != NO_KEY && last_key == NO_KEY) {
    last_key = current_key;
    return KEY_PRESSED;
  }

  // Detect key release event
  if (current_key == NO_KEY && last_key != NO_KEY) {
    char released_key = last_key;
    last_key = NO_KEY;
    *key = released_key;
    return KEY_RELEASED;
  }

  // No change in key state
  return KEY_IDLE;
}
