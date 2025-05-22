#ifndef STRFMT_H
#define STRFMT_H

#include <stdint.h>

/**
 * @brief Copies a null-terminated string to a destination buffer.
 *
 * Optionally appends CRLF ("\r\n") and/or a null terminator.
 *
 * @param dest     Destination buffer (must have enough space).
 * @param str      Source null-terminated string.
 * @param newline  If non-zero, append CRLF ("\r\n") after the string.
 * @param nul_term If non-zero, append null terminator at the end.
 *
 * @return Number of characters appended excluding null terminator,
 *         plus 2 if newline added.
 */
uint8_t appendLine(char* dest, const char* str, uint8_t newline,
                   uint8_t nul_term);

/**
 * @brief Converts an unsigned 16-bit integer to decimal string and appends it.
 *
 * @param dest     Destination buffer (must have enough space).
 * @param val      Unsigned 16-bit integer to convert.
 * @param nul_term If non-zero, append null terminator at the end.
 *
 * @return Number of characters appended including null terminator if added.
 */
uint8_t appendUInt(char* dest, uint16_t val, uint8_t nul_term);

/**
 * @brief Converts a signed 32-bit integer to decimal string and appends it.
 *
 * Handles negative values by adding a leading '-' sign.
 *
 * @param dest     Destination buffer (must have enough space).
 * @param val      Signed 32-bit integer to convert.
 * @param nul_term If non-zero, append null terminator at the end.
 *
 * @return Number of characters appended including null terminator if added.
 */
uint8_t appendInt(char* dest, int32_t val, uint8_t nul_term);

/**
 * @brief Appends the 40 least significant bits of a 64-bit integer as binary.
 *
 * Spaces are inserted after bit positions 24 and 8 for readability.
 *
 * @param dest     Destination buffer (must have enough space).
 * @param bin      64-bit integer whose lower 40 bits will be appended.
 * @param nul_term If non-zero, append null terminator at the end.
 *
 * @return Number of characters appended including null terminator if added.
 */
uint8_t appendBinary40(char* dest, uint64_t bin, uint8_t nul_term);

/**
 * @brief Converts a floating-point value to decimal string with fixed precision.
 *
 * @param dest      Destination buffer (must have enough space).
 * @param val       Floating-point value to convert.
 * @param precision Number of digits after the decimal point.
 * @param nul_term  If non-zero, append null terminator at the end.
 *
 * @return Number of characters appended including null terminator if added.
 */
uint8_t appendFloat(char* dest, float val, uint8_t precision, uint8_t nul_term);

#endif  // STRFMT_H
