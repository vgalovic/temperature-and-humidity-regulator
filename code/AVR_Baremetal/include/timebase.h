#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/**
 * @brief Initializes Timer0 as a system timebase.
 *
 * Configures Timer0 with a prescaler of 64 on a 16 MHz clock,
 * sets up the overflow interrupt to track elapsed time,
 * and enables global interrupts.
 *
 * This function is safe to call multiple times; initialization
 * will only happen once.
 */
void timebaseInit(void);

/**
 * @brief Gets the number of milliseconds elapsed since timebase initialization.
 *
 * This value is incremented approximately every 1.024 milliseconds by
 * the Timer0 overflow interrupt.
 *
 * @return Elapsed milliseconds as a 32-bit unsigned integer.
 */
uint32_t millis(void);

/**
 * @brief Gets the number of microseconds elapsed since timebase initialization.
 *
 * Combines the Timer0 overflow counter and current timer count to provide
 * microsecond precision timing.
 *
 * @return Elapsed microseconds as a 32-bit unsigned integer.
 */
uint32_t micros(void);

#endif  // TIMEBASE_H
