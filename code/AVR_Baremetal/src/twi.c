#include "twi.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

/**
 * @brief Internal variable to store the latest TWI error status.
 *
 * Holds the status code from TWI status register when an unexpected
 * status is detected.
 */
static char _twi_error_stat = TWI_ERROR_NONE;

/**
 * @brief Initialize TWI (I2C) interface.
 *
 * Configures the TWI bit rate and prescaler to achieve 100 kHz SCL clock
 * assuming a 16 MHz CPU clock. Enables TWI and clears interrupt flag.
 */
void twiInit(void) {
  TWBR = 72;    // Bit rate register for 100 kHz with prescaler = 1
  TWSR = 0x00;  // Prescaler value = 1

  // Enable TWI, clear interrupt flag
  TWCR = (1 << TWEN) | (1 << TWINT);
}

/**
 * @brief Send a START condition on the I2C bus.
 *
 * Signals the beginning of a transmission on the I2C bus.
 * Waits until the START condition is transmitted successfully.
 */
void twiStart(void) {
  // Initiate START condition
  TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));  // Wait for START to complete
}

/**
 * @brief Send a slave address with the read/write bit.
 *
 * Loads the 7-bit slave address and the R/W bit into the data register,
 * then initiates transmission and waits for completion.
 *
 * @param address     7-bit slave address
 * @param read_write  0 to write, 1 to read
 */
void twiSendSlaveAddr(uint8_t address, uint8_t read_write) {
  // Load address and R/W bit into data register
  TWDR = (address << 1) | (read_write ? 1 : 0);

  // Start transmission
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));  // Wait for completion
}

/**
 * @brief Transmit a single byte to the I2C slave.
 *
 * Loads the data byte into the data register, starts transmission,
 * and waits for it to complete.
 *
 * @param data  Byte to send
 */
void twiTransmitByte(uint8_t data) {
  TWDR = data;  // Load data register

  // Start transmission
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));  // Wait for completion
}

/**
 * @brief Receive a byte from the I2C slave.
 *
 * Reads a byte from the data register after the slave transmits.
 * Optionally sends ACK (acknowledge) or NACK (not acknowledge)
 * after reception.
 *
 * @param ack  If non-zero, send ACK after receiving; else send NACK.
 * @return     Received byte from the slave.
 */
uint8_t twiReceiveByte(uint8_t ack) {
  if (ack) {
    // Send ACK after receiving
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  } else {
    // Send NACK after receiving
    TWCR = (1 << TWINT) | (1 << TWEN);
  }

  while (!(TWCR & (1 << TWINT)));  // Wait for completion
  return TWDR;                     // Return received byte
}

/**
 * @brief Send a STOP condition on the I2C bus.
 *
 * Signals the end of the transmission to the slave device.
 * Waits until STOP condition is executed.
 */
void twiStop(void) {
  // Initiate STOP condition
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
  while (TWCR & (1 << TWSTO));  // Wait until STOP is completed
}

/**
 * @brief Check the current TWI status against an expected status.
 *
 * Reads the TWI status register (TWSR) and masks out prescaler bits.
 * If the status differs from expected, records the error status internally.
 *
 * @param expected_status  Expected status code (e.g., 0x18 for SLA+W transmitted, ACK received)
 */
void twiCheckStatus(uint8_t expected_status) {
  uint8_t status = TWSR & 0xF8;  // Mask prescaler bits

  if (status != expected_status) {
    _twi_error_stat = status;  // Store the error code for later inspection
  }
}
