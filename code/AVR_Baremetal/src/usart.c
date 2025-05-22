#include "usart.h"

#include <avr/interrupt.h>
#include <avr/io.h>

static char rx_buffer[USART_BUFFER_SIZE];  // Circular buffer for received data
static volatile uint8_t rx_buffer_head = 0;  // Read index in buffer
static volatile uint8_t rx_buffer_tail = 0;  // Write index in buffer
static volatile uint8_t rx_buffer_size = 0;  // Number of bytes currently stored

/**
 * @brief USART RX complete interrupt service routine.
 *
 * Automatically called when a byte is received.
 * Stores the received byte in the circular RX buffer if space is available.
 */
ISR(USART_RX_vect) {
  if (rx_buffer_size < USART_BUFFER_SIZE) {  // Check buffer space
    rx_buffer[rx_buffer_tail++] = UDR0;      // Read received byte into buffer
    rx_buffer_tail &=
        USART_BUFFER_SIZE - 1;  // Wrap tail index for circular buffer
    rx_buffer_size++;           // Increase count of stored bytes
  }
}

/**
 * @brief Initialize USART with specified baud rate.
 *
 * Configures baud rate registers, enables RX, TX and RX complete interrupt.
 * Sets frame format to 8 data bits, no parity, 1 stop bit (8N1).
 *
 * @param baud Baud rate (e.g. 9600, 115200)
 */
void usartBegin(uint32_t baud) {
  uint16_t ubrr = F_CPU / (16UL * baud) - 1;  // Calculate UBRR value

  UBRR0H = (ubrr >> 8);  // Set baud rate high byte
  UBRR0L = ubrr;         // Set baud rate low byte

  UCSR0B = (1 << RXEN0) | (1 << TXEN0) |
           (1 << RXCIE0);  // Enable RX, TX, RX interrupt

  UCSR0C =
      (1 << UCSZ01) | (1 << UCSZ00);  // Set 8 data bits, no parity, 1 stop bit

  sei();  // Enable global interrupts
}

/**
 * @brief Disable USART communication.
 *
 * Disables receiver, transmitter and RX complete interrupt.
 * Clears the RX buffer.
 */
void usartEnd(void) {
  UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) |
              (1 << RXCIE0));  // Disable USART RX, TX, RX interrupt

  rx_buffer_head = 0;  // Reset buffer indices and size
  rx_buffer_tail = 0;
  rx_buffer_size = 0;
}

/**
 * @brief Get the number of bytes available in the receive buffer.
 *
 * @return Number of bytes waiting to be read.
 */
uint8_t usartAvailable(void) {
  return rx_buffer_size;
}

/**
 * @brief Send a single character over USART.
 *
 * Waits until the transmit buffer is empty before sending.
 *
 * @param c Character to transmit.
 */
void usartSetChar(char c) {
  while (!(UCSR0A & (1 << UDRE0)));  // Wait until transmit buffer empty
  UDR0 = c;                          // Send character
}

/**
 * @brief Send a null-terminated string over USART.
 *
 * @param s Pointer to the string to transmit.
 */
void usartPrint(const char *s) {
  while (*s) {
    usartSetChar(*s++);
  }
}

/**
 * @brief Send a null-terminated string followed by CRLF over USART.
 *
 * @param s Pointer to the string to transmit.
 */
void usartPrintln(const char *s) {
  usartPrint(s);
  usartPrint("\r\n");
}

/**
 * @brief Retrieve a single character from the receive buffer.
 *
 * Returns -1 if no data is available.
 *
 * @return Received character or -1 if buffer empty.
 */
char usartGetChar(void) {
  if (rx_buffer_size == 0)
    return -1;  // Buffer empty

  char c = rx_buffer[rx_buffer_head++];     // Read character from buffer
  rx_buffer_head &= USART_BUFFER_SIZE - 1;  // Wrap head index
  rx_buffer_size--;                         // Decrement buffer size

  return c;
}

/**
 * @brief Read a line (up to newline or carriage return) from the receive buffer.
 *
 * Stores the received characters in the provided buffer, null-terminated.
 * Stops reading on newline, carriage return, or when buffer is full.
 *
 * @param buf Pointer to buffer to store the received line.
 */
void usartRead(char *buf) {
  uint8_t i = 0;
  char c;

  while (i < USART_BUFFER_SIZE - 1) {
    if (rx_buffer_size > 0) {
      c = usartGetChar();
      if (c == '\n' || c == '\r')
        break;
      buf[i++] = c;
    }
  }

  buf[i] = '\0';  // Null-terminate the string
}
