#ifndef TWI_H_
#define TWI_H_

#include <stdint.h>

/// Target TWI (I2C) clock frequency in Hz
#define TWI_FREQUENCY 100000  // 100 kHz

/// TWI error status codes
#define TWI_ERROR_NONE 0      ///< No error
#define TWI_ERROR_START 1     ///< Error on START condition
#define TWI_ERROR_SLA_ACK 2   ///< SLA+R/W not acknowledged
#define TWI_ERROR_DATA_ACK 3  ///< Data byte not acknowledged

/**
 * @brief Initialize the TWI (I2C) hardware interface.
 *
 * Configures the TWI bit rate and prescaler to generate the target
 * SCL frequency. Enables the TWI module.
 */
void twiInit(void);

/**
 * @brief Send a START condition on the I2C bus.
 *
 * Signals the beginning of a communication session with a slave device.
 * Blocks until the START condition has been transmitted.
 */
void twiStart(void);

/**
 * @brief Send a slave address and R/W bit.
 *
 * Loads the 7-bit slave address and read/write flag into the data register,
 * then transmits it on the bus.
 *
 * @param address    7-bit I2C slave address
 * @param read_write 0 for write, 1 for read operation
 */
void twiSendSlaveAddr(uint8_t address, uint8_t read_write);

/**
 * @brief Transmit a single data byte to the slave device.
 *
 * Loads the byte into the data register and transmits it.
 *
 * @param data Byte to be sent
 */
void twiTransmitByte(uint8_t data);

/**
 * @brief Receive a data byte from the slave device.
 *
 * Reads a byte from the TWI data register. Sends an ACK or NACK
 * after receiving depending on the parameter.
 *
 * @param ack If non-zero, send ACK after reception; else send NACK.
 * @return    The received byte from the slave device.
 */
uint8_t twiReceiveByte(uint8_t ack);

/**
 * @brief Send a STOP condition on the I2C bus.
 *
 * Terminates the current communication session with the slave device.
 * Blocks until the STOP condition has been transmitted.
 */
void twiStop(void);

/**
 * @brief Check the current TWI status register against an expected value.
 *
 * Reads the TWI status register and compares it with the expected status code.
 * Records an error if they do not match.
 *
 * @param expected_status Expected status code to verify
 */
void twiCheckStatus(uint8_t expected_status);

#endif  // TWI_H_
