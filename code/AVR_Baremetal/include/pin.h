#ifndef PIN_H_
#define PIN_H_

#include <stdint.h>

// Digital pin states
#define HIGH 1 /**< Digital HIGH state */
#define LOW 0  /**< Digital LOW state */

// Pin direction modes
#define INPUT 0  /**< Configure pin as input */
#define OUTPUT 1 /**< Configure pin as output */

/**
 * @brief Configure the specified pin as INPUT or OUTPUT.
 *
 * @param pin Pin number to configure.
 * @param direction Direction mode: INPUT or OUTPUT.
 */
void pinMode(uint8_t pin, uint8_t direction);

/**
 * @brief Write a digital value (HIGH or LOW) to a pin.
 *
 * @param pin Pin number to write to.
 * @param value Value to write: HIGH or LOW.
 */
void digitalWrite(uint8_t pin, uint8_t value);

/**
 * @brief Read the digital value from a pin.
 *
 * @param pin Pin number to read from.
 * @return uint8_t Returns HIGH or LOW.
 */
uint8_t digitalRead(uint8_t pin);

/**
 * @brief Write an analog value (PWM) to a pin.
 *
 * @param pin Pin number to write to.
 * @param value PWM value (0-255).
 */
void analogWrite(uint8_t pin, uint8_t value);

/**
 * @brief Measure the length (in microseconds) of a pulse on the pin.
 *
 * Waits for the pin to go to the given state, then measures how long it stays in that state.
 *
 * @param pin Pin number to read pulse on.
 * @param state State to measure pulse duration of: HIGH or LOW.
 * @param timeout Timeout in microseconds to wait for pulse.
 * @return uint32_t Pulse duration in microseconds, or 0 if timeout occurred.
 */
uint32_t pulseIn(uint8_t pin, uint8_t state, uint32_t timeout);

#endif  // PIN_H_
