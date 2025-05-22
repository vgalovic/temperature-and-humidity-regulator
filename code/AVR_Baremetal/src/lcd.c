#include "lcd.h"

#include <stdint.h>
#include <util/delay.h>

#include "twi.h"

// LCD command definitions for controlling the LCD module
#define LCD_CLEAR_DISPLAY \
  0x01  // Clear entire display and reset DDRAM address to 0
#define LCD_RETURN_HOME 0x02  // Return cursor to home position (0,0)

#define LCD_DISPLAY_ON 0x0C   // Display ON, Cursor OFF, Blink OFF
#define LCD_DISPLAY_OFF 0x08  // Display OFF

#define LCD_CURSOR_ON 0x0E   // Display ON, Cursor ON, Blink OFF
#define LCD_CURSOR_OFF 0x0C  // Display ON, Cursor OFF, Blink OFF

#define LCD_CURSOR_BLINK_ON 0x0F   // Display ON, Cursor ON, Blink ON
#define LCD_CURSOR_BLINK_OFF 0x0E  // Display ON, Cursor ON, Blink OFF

#define LCD_ENTRY_MODE 0x06  // Entry mode: cursor moves right, no display shift

#define LCD_SET_4BIT_MODE 0x20  // Set 4-bit interface mode
#define LCD_SET_8BIT_MODE 0x30  // Set 8-bit interface mode
#define LCD_SET_DDRAM_ADDR \
  0x80  // Set DDRAM address (used for cursor positioning)

#define LCD_FUNCTION_SET \
  ((lcd_rows <= 2) ? 0x28 : 0xC2)  // Function set command based on rows

#define LCD_BACKLIGHT_ON 0x08   // Backlight ON bit
#define LCD_BACKLIGHT_OFF 0x00  // Backlight OFF bit

#define LCD_START 0x0C  // Start display: Display ON, Cursor OFF, Blink OFF

// Control bits for communication with LCD
#define LCD_RS 0x01  // Register Select: 1 = Data, 0 = Command
#define LCD_EN 0x04  // Enable bit (used to latch data)
#define NIBBLE 0xF0  // Mask for upper nibble (4 bits)

/* Static variables storing current LCD configuration and states */
static uint8_t lcd_addr;  // I2C address of the LCD
static uint8_t lcd_cols;  // Number of columns in the LCD
static uint8_t lcd_rows;  // Number of rows in the LCD

static uint8_t lcd_backlight = LCD_BACKLIGHT_OFF;  // Current backlight state
static uint8_t lcd_disply = LCD_DISPLAY_ON;  // Current display ON/OFF state
static uint8_t lcd_cursor_blink = LCD_CURSOR_BLINK_OFF;  // Cursor blink state
static uint8_t lcd_cursor = LCD_CURSOR_OFF;  // Cursor visibility state

/**
 * @brief Send one 4-bit nibble to the LCD over I2C with given control bits.
 *
 * This function handles the enable pulse and data transmission for 4-bit mode.
 *
 * @param nibble  Upper nibble (4 bits) of data/command to send (bits are on upper nibble).
 * @param control Control bits (e.g., LCD_RS for data/command selection).
 */
static void lcdWriteNibble(uint8_t nibble, uint8_t control) {
  uint8_t data = (nibble & NIBBLE) | control | lcd_backlight;

  twiStart();
  twiSendSlaveAddr(lcd_addr, 0);   // Send I2C address with write bit
  twiTransmitByte(data | LCD_EN);  // Enable pulse high
  _delay_us(1);
  twiTransmitByte(data & ~LCD_EN);  // Enable pulse low
  twiStop();
  _delay_us(50);  // Wait for LCD to latch data
}

/**
 * @brief Initialize the LCD module via I2C.
 *
 * This performs the required initialization sequence for LCD in 4-bit mode.
 *
 * @param i2c_addr I2C address of the LCD.
 * @param cols     Number of columns on the LCD.
 * @param rows     Number of rows on the LCD.
 * @return uint8_t Returns 0 on success (no error handling implemented).
 */
uint8_t lcdInit(uint8_t i2c_addr, uint8_t cols, uint8_t rows) {
  lcd_addr = i2c_addr;
  lcd_cols = cols;
  lcd_rows = rows;

  twiInit();      // Initialize TWI/I2C peripheral
  _delay_ms(50);  // Wait for LCD to power up

  // Reset sequence: send 8-bit mode commands 3 times to ensure LCD resets
  lcdWriteNibble(LCD_SET_8BIT_MODE, 0);
  _delay_ms(5);
  lcdWriteNibble(LCD_SET_8BIT_MODE, 0);
  _delay_us(150);
  lcdWriteNibble(LCD_SET_8BIT_MODE, 0);
  _delay_us(150);

  // Enter 4-bit mode
  lcdWriteNibble(LCD_SET_4BIT_MODE, 0);
  _delay_us(150);

  // Function set command (4-bit, number of lines, font)
  lcdSendCommand(LCD_FUNCTION_SET);

  // Clear display
  lcdSendCommand(LCD_CLEAR_DISPLAY);
  _delay_ms(2);

  // Set entry mode (increment cursor, no shift)
  lcdSendCommand(LCD_ENTRY_MODE);

  lcd_backlight = LCD_BACKLIGHT_ON;  // Turn backlight ON
  lcdSendCommand(LCD_START);         // Turn display ON (no cursor, no blink)
  _delay_ms(2);

  lcdSendCommand(LCD_RETURN_HOME);  // Return cursor to home position
  _delay_ms(2);

  return 0;  // Initialization successful
}

/**
 * @brief Send a command byte to the LCD.
 *
 * The command is sent in two 4-bit nibbles (high nibble first).
 *
 * @param cmd Command byte to send.
 */
void lcdSendCommand(uint8_t cmd) {
  lcdWriteNibble(cmd & NIBBLE, 0);         // Send high nibble, RS = 0 (command)
  lcdWriteNibble((cmd << 4) & NIBBLE, 0);  // Send low nibble, RS = 0 (command)
}

/**
 * @brief Send a character to the LCD to be displayed.
 *
 * The character is sent in two 4-bit nibbles (high nibble first).
 *
 * @param chr Character to display.
 */
void lcdPrintChar(char chr) {
  lcdWriteNibble(chr & NIBBLE, LCD_RS);  // Send high nibble, RS = 1 (data)
  lcdWriteNibble((chr << 4) & NIBBLE,
                 LCD_RS);  // Send low nibble, RS = 1 (data)
}

/**
 * @brief Print a null-terminated string to the LCD.
 *
 * This function sends characters sequentially until the string terminator.
 *
 * @param str Pointer to the null-terminated string.
 */
void lcdPrint(const char *str) {
  while (*str) {
    lcdPrintChar(*str++);
  }
}

/**
 * @brief Set the LCD cursor position.
 *
 * Uses the DDRAM addressing scheme of the LCD controller.
 *
 * @param col Column index (0-based).
 * @param row Row index (0-based).
 */
void lcdSetCursor(uint8_t col, uint8_t row) {
  // Row offsets for common LCD DDRAM addressing
  const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

  if (row >= lcd_rows || col >= lcd_cols)
    return;  // Invalid position, ignore request

  lcdSendCommand(LCD_SET_DDRAM_ADDR | (row_offsets[row] + col));
  _delay_ms(2);
}

/**
 * @brief Toggle the LCD backlight ON or OFF.
 *
 * This toggles the backlight state and updates the LCD accordingly.
 */
void lcdToggleBacklight() {
  lcd_backlight = (lcd_backlight == LCD_BACKLIGHT_ON) ? LCD_BACKLIGHT_OFF
                                                      : LCD_BACKLIGHT_ON;
  // Send dummy command to latch new backlight state (no real command)
  lcdSendCommand(lcd_backlight);
  _delay_ms(2);
}

/**
 * @brief Toggle cursor blinking ON or OFF.
 *
 * Updates the cursor blinking display state.
 */
void lcdToggleCursorBlink() {
  lcd_cursor_blink = (lcd_cursor_blink == LCD_CURSOR_BLINK_ON)
                         ? LCD_CURSOR_BLINK_OFF
                         : LCD_CURSOR_BLINK_ON;
  lcdSendCommand(lcd_cursor_blink);
  _delay_ms(2);
}

/**
 * @brief Toggle cursor visibility ON or OFF.
 *
 * Updates whether the cursor is visible or hidden.
 */
void lcdToggleCursor() {
  lcd_cursor = (lcd_cursor == LCD_CURSOR_ON) ? LCD_CURSOR_OFF : LCD_CURSOR_ON;
  lcdSendCommand(lcd_cursor);
  _delay_ms(2);
}

/**
 * @brief Toggle the entire display ON or OFF.
 */
void lcdToggleDisplay() {
  lcd_disply =
      (lcd_disply == LCD_DISPLAY_ON) ? LCD_DISPLAY_OFF : LCD_DISPLAY_ON;
  lcdSendCommand(lcd_disply);
  _delay_ms(2);
}

/**
 * @brief Clear the LCD display and set cursor to home position.
 */
void lcdClear() {
  lcdSendCommand(LCD_CLEAR_DISPLAY);
  _delay_ms(2);
}

/**
 * @brief Return the cursor to the home position (0,0).
 */
void lcdHome() {
  lcdSendCommand(LCD_RETURN_HOME);
  _delay_ms(2);
}
