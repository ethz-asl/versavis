////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  NmeaParser.h
////////////////////////////////////////////////////////////////////////////////
//
// Helper functions to parse NMEA messages.
//
////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <cstdint>
#include <cstring>

template <class T>
bool numFromWord(const char *data, const uint8_t start_idx, const uint8_t len,
                 T *result) {
  bool success = ((start_idx + len - 1) < strlen(data));

  // Create copy of data range.
  char cpy[len + 1];
  memset(cpy, '\0', len + 1);
  memcpy(cpy, data + start_idx, len);

  // Check if all digits.
  for (auto i = 0; i < len; ++i) {
    success &= isDigit(cpy[i]);
  }

  // Convert to unsigned long integer.
  auto conversion = strtoul(cpy, NULL, 10);

  // Check within data range.
  T numeric_limit = ~T(0); // Bitwise NOT of 0. WARNING: only for unsigned int
  success &= (conversion < numeric_limit); // Inside target object range.

  if (result)
    *result = conversion;

  return success;
}
