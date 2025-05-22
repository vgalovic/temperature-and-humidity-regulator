#ifndef KEYPAD_H_
#define KEYPAD_H_

#include <stdint.h>

/**
 * @brief Macro to cast a keymap string literal to a const char pointer.
 * @param map The keymap string.
 */
#define MAPKEYPAD(map) ((const char *)(map))

/**
 * @brief Represents no key pressed.
 */
#define NO_KEY '\0'

/**
 * @brief Enumeration of keypad event states.
 */
typedef enum {
  KEY_IDLE = 0, /**< No key event */
  KEY_PRESSED,  /**< Key pressed event */
  KEY_RELEASED  /**< Key released event */
} keypad_event_t;

/**
 * @brief Initializes the keypad interface.
 *
 * @param userKeymap Pointer to a string representing the user-defined keymap.
 * @param row_pins Pointer to an array of GPIO pins connected to keypad rows.
 * @param col_pins Pointer to an array of GPIO pins connected to keypad columns.
 * @param rows Number of rows in the keypad.
 * @param cols Number of columns in the keypad.
 */
void keypadInit(const char *userKeymap, const uint8_t *row_pins,
                const uint8_t *col_pins, uint8_t rows, uint8_t cols);

/**
 * @brief Scans the keypad for a pressed key.
 *
 * @return The character corresponding to the pressed key, or NO_KEY if none pressed.
 */
char keypadScan();

/**
 * @brief Sets the debounce time for keypad scanning.
 *
 * @param time Debounce time in milliseconds.
 */
void setDebounceTime(uint16_t time);

/**
 * @brief Returns the current keypad event state for the given key.
 *
 * @param key Pointer to a character where the detected key will be stored.
 * @return The keypad event state (KEY_IDLE, KEY_PRESSED, or KEY_RELEASED).
 */
keypad_event_t keypadState(char *key);

#endif  // KEYPAD_H_
