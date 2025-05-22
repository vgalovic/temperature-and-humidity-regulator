#include "timebase.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

// Global counter incremented every Timer0 overflow (~1.024 ms)
volatile uint32_t millis_counter = 0;

// Guard flag to ensure timer is initialized only once
static uint8_t initialized = 0;

/**
 * @brief Timer0 overflow interrupt service routine.
 *
 * This ISR triggers every 256 Timer0 ticks. With a prescaler of 64 on a
 * 16 MHz clock, each tick is 4 µs, so the overflow occurs approximately
 * every 1.024 ms.
 *
 * Increments the global millis_counter to track elapsed milliseconds.
 */
ISR(TIMER0_OVF_vect) {
  millis_counter++;
}

/**
 * @brief Initializes Timer0 as the system timebase.
 *
 * Configures Timer0 in normal mode with a prescaler of 64, resulting in
 * a timer tick every 4 µs (16 MHz / 64). Enables the Timer0 overflow
 * interrupt to increment millis_counter every ~1.024 ms.
 *
 * Enables global interrupts after setup.
 *
 * This function does nothing if called more than once.
 */
void timebaseInit(void) {
  if (initialized)
    return;

  TCCR0A = 0;                          // Normal timer mode (no PWM)
  TCCR0B = (1 << CS01) | (1 << CS00);  // Set prescaler to 64
  TIMSK0 |= (1 << TOIE0);              // Enable Timer0 overflow interrupt
  TCNT0 = 0;                           // Reset timer count

  sei();  // Enable global interrupts
  initialized = 1;
}

/**
 * @brief Returns the number of milliseconds elapsed since timer initialization.
 *
 * Uses a global counter incremented by the Timer0 overflow ISR.
 * Access to the volatile counter is protected by disabling interrupts
 * to prevent reading a partially updated value.
 *
 * @return Elapsed milliseconds as a 32-bit unsigned integer.
 */
uint32_t millis(void) {
  uint32_t ms;
  uint8_t oldSREG = SREG;  // Save interrupt flag state
  cli();                   // Disable interrupts
  ms = millis_counter;     // Copy volatile variable
  SREG = oldSREG;          // Restore interrupt flag state
  return ms;
}

/**
 * @brief Returns the number of microseconds elapsed since timer initialization.
 *
 * Combines the overflow count (millis_counter) with the current Timer0 count
 * to calculate microseconds more precisely.
 *
 * Disables interrupts to safely read the counter and overflow flag, and
 * accounts for the rare case where an overflow happens during reading.
 *
 * @return Elapsed microseconds as a 32-bit unsigned integer.
 */
uint32_t micros(void) {
  uint32_t overflows;
  uint8_t tcnt;

  uint8_t oldSREG = SREG;  // Save interrupt flag state
  cli();                   // Disable interrupts

  overflows = millis_counter;
  tcnt = TCNT0;

  // Check if overflow occurred but ISR has not yet run
  if ((TIFR0 & (1 << TOV0)) && (tcnt < 255)) {
    overflows++;
    tcnt = TCNT0;
  }

  SREG = oldSREG;  // Restore interrupt flag state

  // Each overflow represents 256 timer ticks; each tick = 4 µs
  return ((overflows << 8) + tcnt) * 4UL;
}
