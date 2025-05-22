#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>

/**
 * @brief Initialize the LCD display.
 *
 * @param i2c_addr The I2C address of the LCD module.
 * @param cols     Number of columns in the LCD.
 * @param rows     Number of rows in the LCD.
 *
 * @return uint8_t Returns non-zero on success, zero otherwise.
 */
uint8_t lcdInit(uint8_t i2c_addr, uint8_t cols, uint8_t rows);

/**
 * @brief Clear the LCD display and set cursor position to home (0,0).
 */
void lcdClear();

/**
 * @brief Set the cursor position to home (0,0) without clearing the display.
 */
void lcdHome();

/**
 * @brief Print a null-terminated string to the LCD at the current cursor position.
 *
 * @param str Pointer to the string to be displayed.
 */
void lcdPrint(const char *str);

/**
 * @brief Print a single character to the LCD at the current cursor position.
 *
 * @param chr Character to display.
 */
void lcdPrintChar(char chr);

/**
 * @brief Send a low-level command byte directly to the LCD.
 *
 * @param cmd Command byte to send.
 */
void lcdSendCommand(uint8_t cmd);

/**
 * @brief Set the cursor position on the LCD.
 *
 * @param col Column index (0-based).
 * @param row Row index (0-based).
 */
void lcdSetCursor(uint8_t col, uint8_t row);

/**
 * @brief Toggle the LCD backlight on or off.
 */
void lcdToggleBacklight();

/**
 * @brief Toggle the visibility of the cursor.
 */
void lcdToggleCursor();

/**
 * @brief Toggle the blinking of the cursor.
 */
void lcdToggleCursorBlink();

/**
 * @brief Toggle the entire display on or off.
 */
void lcdToggleDisplay();

#endif  // LCD_H_
