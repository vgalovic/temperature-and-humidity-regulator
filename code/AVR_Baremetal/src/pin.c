#include "pin.h"

#include <avr/io.h>

#define F_CPU 16000000UL  // CPU frequency in Hz (adjust as needed)

#define VALID_PIN(pin) ((pin) <= 19)

/**
 * Macros to access hardware registers and bit positions for each pin.
 * These help map the Arduino pin numbers to the underlying AVR registers.
 */
#define DDR_OF(pin) \
  (*(ddr_table[(pin)]))  // Data Direction Register for the pin
#define PORT_OF(pin) (*(port_table[(pin)]))  // Data Register (write HIGH/LOW)
#define PIN_REG_OF(pin) \
  (*(pin_reg_table[(pin)]))             // Input Register (read pin state)
#define BIT_OF(pin) (bit_table[(pin)])  // Bit number within the register

/**
 * Lookup tables mapping pin numbers (0–19) to the corresponding AVR DDR, PORT, and PIN registers,
 * and the bit positions within those registers.
 * This abstracts away hardware specifics and allows generic pin manipulation.
 */
static volatile uint8_t* const ddr_table[20] = {
    &DDRD, &DDRD, &DDRD, &DDRD, &DDRD, &DDRD, &DDRD, &DDRD,  // Digital pins 0–7
    &DDRB, &DDRB, &DDRB, &DDRB, &DDRB, &DDRB,  // Digital pins 8–13
    &DDRC, &DDRC, &DDRC, &DDRC, &DDRC, &DDRC   // Analog pins A0–A5
};

static volatile uint8_t* const port_table[20] = {
    &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD,
    &PORTD, &PORTB, &PORTB, &PORTB, &PORTB, &PORTB, &PORTB,
    &PORTC, &PORTC, &PORTC, &PORTC, &PORTC, &PORTC};

static volatile uint8_t* const pin_reg_table[20] = {
    &PIND, &PIND, &PIND, &PIND, &PIND, &PIND, &PIND, &PIND, &PINB, &PINB,
    &PINB, &PINB, &PINB, &PINB, &PINC, &PINC, &PINC, &PINC, &PINC, &PINC};

static const uint8_t bit_table[20] = {
    0, 1, 2, 3, 4, 5, 6, 7,  // Pins 0–7 bits
    0, 1, 2, 3, 4, 5,        // Pins 8–13 bits
    0, 1, 2, 3, 4, 5         // Pins A0–A5 bits
};

/**
 * @brief Configure a pin as INPUT or OUTPUT.
 *
 * Sets the appropriate DDR bit to configure the pin direction.
 * Invalid pins are ignored.
 *
 * @param pin Pin number (0–19).
 * @param direction INPUT (0) or OUTPUT (1).
 */
void pinMode(uint8_t pin, uint8_t direction) {
  if (!VALID_PIN(pin))
    return;

  if (direction == OUTPUT) {
    DDR_OF(pin) |= (1 << BIT_OF(pin));
  } else {
    DDR_OF(pin) &= ~(1 << BIT_OF(pin));
  }
}

/**
 * @brief Write a digital HIGH or LOW value to a pin.
 *
 * Sets or clears the PORT bit accordingly.
 * Invalid pins are ignored.
 *
 * @param pin Pin number (0–19).
 * @param value HIGH (1) or LOW (0).
 */
void digitalWrite(uint8_t pin, uint8_t value) {
  if (!VALID_PIN(pin))
    return;

  if (value == HIGH) {
    PORT_OF(pin) |= (1 << BIT_OF(pin));
  } else {
    PORT_OF(pin) &= ~(1 << BIT_OF(pin));
  }
}

/**
 * @brief Read the digital state of a pin.
 *
 * Returns HIGH if the pin input register bit is set, LOW otherwise.
 * Invalid pins return LOW.
 *
 * @param pin Pin number (0–19).
 * @return uint8_t HIGH or LOW.
 */
uint8_t digitalRead(uint8_t pin) {
  if (!VALID_PIN(pin))
    return 0;

  return (PIN_REG_OF(pin) & (1 << BIT_OF(pin))) ? HIGH : LOW;
}

/**
 * @brief Generate a PWM signal on supported pins or fallback to digital HIGH/LOW.
 *
 * Configures timers and output compare registers for hardware PWM on pins
 * 3, 5, 6, 9, 10, and 11 on Arduino Uno.
 * For other pins, writes digital HIGH if value >= 128, else LOW.
 *
 * @param pin Pin number (0–19).
 * @param value PWM duty cycle (0–255).
 */
void analogWrite(uint8_t pin, uint8_t value) {
  switch (pin) {
    case 3:                 // OC2B - PD3
      DDRD |= (1 << DDD3);  // Set output
      TCCR2A = (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
      TCCR2B = (1 << CS21);  // Prescaler 8
      OCR2B = value;
      break;

    case 5:  // OC0B - PD5
      DDRD |= (1 << DDD5);
      TCCR0A = (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
      TCCR0B = (1 << CS01);
      OCR0B = value;
      break;

    case 6:  // OC0A - PD6
      DDRD |= (1 << DDD6);
      TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
      TCCR0B = (1 << CS01);
      OCR0A = value;
      break;

    case 9:  // OC1A - PB1
      DDRB |= (1 << DDB1);
      TCCR1A = (1 << COM1A1) | (1 << WGM10);
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
      OCR1A = value;
      break;

    case 10:  // OC1B - PB2
      DDRB |= (1 << DDB2);
      TCCR1A = (1 << COM1B1) | (1 << WGM10);
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
      OCR1B = value;
      break;

    case 11:  // OC2A - PB3
      DDRB |= (1 << DDB3);
      TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);
      TCCR2B = (1 << CS21);
      OCR2A = value;
      break;

    default:
      // Unsupported pins: simple digital HIGH or LOW based on threshold
      digitalWrite(pin, (value < 128) ? LOW : HIGH);
      break;
  }
}

/**
 * @brief Measure the length (in microseconds) of a pulse on a pin.
 *
 * Waits for the pulse to start by detecting a change to the desired state,
 * then measures how long the pin stays in that state before changing again.
 * Returns 0 on timeout or invalid pin.
 *
 * The measurement assumes the CPU frequency and estimates loop timing.
 *
 * @param pin Pin number (0–19).
 * @param state Logic state to measure pulse duration for (HIGH or LOW).
 * @param timeout Timeout in microseconds to wait for pulse.
 * @return uint32_t Duration of pulse in microseconds, or 0 if timeout.
 */
uint32_t pulseIn(uint8_t pin, uint8_t state, uint32_t timeout) {
  if (!VALID_PIN(pin))
    return 0;

  uint32_t count = 0;
  uint32_t max_count =
      timeout * (F_CPU / 1000000UL / 4);  // Approximate CPU cycles per loop

  // Wait for any previous pulse to end
  while (digitalRead(pin) == state) {
    if (count++ >= max_count)
      return 0;  // Timeout: pulse didn't end
  }

  // Wait for the pulse to start
  count = 0;
  while (digitalRead(pin) != state) {
    if (count++ >= max_count)
      return 0;  // Timeout: pulse didn't start
  }

  // Measure pulse length
  count = 0;
  while (digitalRead(pin) == state) {
    if (count++ >= max_count)
      return 0;  // Timeout: pulse too long
  }

  // Convert count to microseconds and return
  return (count * 4) / (F_CPU / 1000000UL);
}
