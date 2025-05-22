#ifndef USART_H_
#define USART_H_

#include <stdint.h>

// Size of the receive buffer (must be a power of two for circular buffer mask)
#define USART_BUFFER_SIZE 128

/**
 * @brief Initialize USART with the specified baud rate.
 *
 * Configures USART registers, enables RX, TX, and RX complete interrupt.
 *
 * @param baud Desired baud rate (e.g., 9600, 115200)
 */
void usartBegin(uint32_t baud);

/**
 * @brief Disable USART communication.
 *
 * Turns off USART RX and TX, disables RX interrupts, and clears RX buffer.
 */
void usartEnd(void);

/**
 * @brief Retrieve a single received character from the RX buffer.
 *
 * @return The next character from the buffer, or -1 if buffer is empty.
 */
char usartGetChar(void);

/**
 * @brief Transmit a single character via USART.
 *
 * Blocks until transmit buffer is ready.
 *
 * @param c Character to send.
 */
void usartSetChar(char c);

/**
 * @brief Transmit a null-terminated string via USART.
 *
 * @param s Pointer to the string to send.
 */
void usartPrint(const char *s);

/**
 * @brief Transmit a null-terminated string followed by CRLF ("\r\n").
 *
 * @param s Pointer to the string to send.
 */
void usartPrintln(const char *s);

/**
 * @brief Get the number of bytes currently available in the receive buffer.
 *
 * @return Number of bytes waiting to be read.
 */
uint8_t usartAvailable(void);

/**
 * @brief Read a line from the receive buffer into the provided buffer.
 *
 * Reads characters until a newline ('\n'), carriage return ('\r'), or buffer full.
 * The line is null-terminated.
 *
 * @param buf Pointer to buffer to store the line.
 */
void usartRead(char *buf);

#endif  // USART_H_
