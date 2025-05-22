#include "strfmt.h"

/**
 * @brief Copies a null-terminated source string into a destination buffer.
 *
 * Copies characters from `src` to `dest`. Optionally appends CRLF (`"\r\n"`)
 * and/or a null terminator (`'\0'`) at the end.
 *
 * @param dest     Pointer to destination buffer; must have enough space.
 * @param src      Null-terminated source string to copy.
 * @param newline  If non-zero, append carriage return and newline ("\r\n") after copying.
 * @param nul_term If non-zero, append null terminator ('\0') at the end.
 *
 * @return Number of characters copied (excluding null terminator),
 *         plus 2 if newline was added.
 */
uint8_t appendLine(char* dest, const char* src, uint8_t newline,
                   uint8_t nul_term) {
  uint8_t len = 0;

  while (src[len] != '\0') {
    *dest++ = src[len++];
  }

  if (newline) {
    *dest++ = '\r';
    *dest++ = '\n';
    len += 2;
  }

  if (nul_term) {
    *dest = '\0';
  }

  return len;
}

/**
 * @brief Appends an unsigned integer (max 65535) as ASCII decimal digits.
 *
 * Converts `val` to a decimal string and appends it to `dest`.
 * Handles zero as a special case. Digits are stored in correct order.
 *
 * @param dest     Pointer to destination buffer; must have enough space.
 * @param val      Unsigned 16-bit integer to convert.
 * @param nul_term If non-zero, append null terminator ('\0') at the end.
 *
 * @return Number of characters appended including null terminator if enabled.
 */
uint8_t appendUInt(char* dest, uint16_t val, uint8_t nul_term) {
  if (val == 0) {
    *dest = '0';
    if (nul_term)
      dest[1] = '\0';
    return nul_term ? 2 : 1;
  }

  uint8_t len = 0;
  // Write digits in reverse order
  while (val) {
    dest[len++] = '0' + (val % 10);
    val /= 10;
  }

  // Reverse digits in place to get correct order
  for (uint8_t i = 0; i < len / 2; i++) {
    char tmp = dest[i];
    dest[i] = dest[len - 1 - i];
    dest[len - 1 - i] = tmp;
  }

  if (nul_term) {
    dest[len++] = '\0';
  }

  return len;
}

/**
 * @brief Appends a signed 32-bit integer as ASCII decimal digits.
 *
 * Handles negative values by prepending '-' sign.
 * Converts `val` to decimal string, stores digits in correct order.
 *
 * @param dest     Pointer to destination buffer; must have enough space.
 * @param val      Signed 32-bit integer to convert.
 * @param nul_term If non-zero, append null terminator ('\0') at the end.
 *
 * @return Number of characters appended including null terminator if enabled.
 */
uint8_t appendInt(char* dest, int32_t val, uint8_t nul_term) {
  uint8_t len = 0;

  if (val < 0) {
    dest[len++] = '-';
    val = -val;
  }

  if (val == 0) {
    dest[len++] = '0';
    if (nul_term)
      dest[len++] = '\0';
    return len;
  }

  // Write digits in reverse order starting at dest[len]
  uint8_t digits_start = len;
  while (val) {
    dest[len++] = '0' + (val % 10);
    val /= 10;
  }

  // Reverse digits only (skip sign if present)
  for (uint8_t i = digits_start; i < (digits_start + (len - digits_start) / 2);
       i++) {
    char tmp = dest[i];
    dest[i] = dest[digits_start + len - 1 - i];
    dest[digits_start + len - 1 - i] = tmp;
  }

  if (nul_term) {
    dest[len++] = '\0';
  }

  return len;
}

/**
 * @brief Appends a 40-bit binary representation of a 64-bit value to a buffer.
 *
 * Extracts the lower 40 bits of `bin` and appends bits as '0' or '1'.
 * Spaces are inserted after bits 24 and 8 for readability.
 *
 * @param dest     Pointer to destination buffer; must have enough space.
 * @param bin      64-bit integer; only lower 40 bits used.
 * @param nul_term If non-zero, append null terminator ('\0') at the end.
 *
 * @return Number of characters appended including null terminator if enabled.
 */
uint8_t appendBinary40(char* dest, uint64_t bin, uint8_t nul_term) {
  uint8_t count = 0;
  for (int8_t i = 39; i >= 0; i--) {
    dest[count++] = (bin & ((uint64_t)1 << i)) ? '1' : '0';

    if (i == 24 || i == 8) {
      dest[count++] = ' ';
    }
  }
  if (nul_term) {
    dest[count++] = '\0';
  }
  return count;
}

/**
 * @brief Appends a floating-point value as a decimal string with fixed precision.
 *
 * Converts `val` to decimal string with `precision` digits after the decimal point.
 * Handles negative values. No rounding is performed on the fractional part.
 *
 * @param dest      Pointer to destination buffer; must have enough space.
 * @param val       Floating-point value to convert.
 * @param precision Number of decimal digits after the decimal point.
 * @param nul_term  If non-zero, append null terminator ('\0') at the end.
 *
 * @return Number of characters appended including null terminator if enabled.
 */
uint8_t appendFloat(char* dest, float val, uint8_t precision,
                    uint8_t nul_term) {
  uint8_t len = 0;

  // Integer part (reuse appendInt without null termination)
  int32_t intPart = (int32_t)val;
  len += appendInt(dest, intPart, 0);

  // Move pointer forward
  dest += len;

  // Decimal point
  *dest++ = '.';
  len++;

  // Fractional part
  float frac = val - (float)intPart;
  if (frac < 0)
    frac = -frac;  // Handle negative values

  for (uint8_t i = 0; i < precision; i++) {
    frac *= 10.0f;
    int digit = (int)frac;
    *dest++ = '0' + digit;
    frac -= digit;
    len++;
  }

  if (nul_term) {
    *dest = '\0';
    len++;
  }

  return len;
}
