////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  NmeaParser.h
////////////////////////////////////////////////////////////////////////////////
//
// A struct representing a ZDA message.
// NMEA description https://resources.winsystems.com/software/nmea.pdf
// A GPZDA sentence: $GPZDA,173538.00,14,01,2020,,*69[...]\n
//
////////////////////////////////////////////////////////////////////////////////

#include <cstdint>

struct ZdaMessage {
public:
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint32_t hundreths = 0;
  uint8_t day = 0;
  uint8_t month = 0;
  uint16_t year = 0;
  char str[23]; // TODO(rikba): Add this member only for debugging.

  bool update(const char *data, const uint8_t field);
  inline void reset() { *this = ZdaMessage(); }

private:
  void toString();
  bool updateHundredths(const char *data);
};
