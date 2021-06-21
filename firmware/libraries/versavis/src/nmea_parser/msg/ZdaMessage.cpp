#include "nmea_parser/msg/ZdaMessage.h"
#include "nmea_parser/msg/helper.h"

bool ZdaMessage::update(const char *data, const uint8_t field) {
  bool success = true;

  switch (field) {
  case 0:
    success &= numFromWord<uint8_t>(data, 0, 2, &hour);
    success &= numFromWord<uint8_t>(data, 2, 2, &minute);
    success &= numFromWord<uint8_t>(data, 4, 2, &second);
    success &= updateHundredths(data);
    break;
  case 1:
    success &= numFromWord<uint8_t>(data, 0, 2, &day);
    break;
  case 2:
    success &= numFromWord<uint8_t>(data, 0, 2, &month);
    break;
  case 3:
    success &= numFromWord<uint16_t>(data, 0, 4, &year);
    break;
  case 4:
    success &= true; // Ignore time zone field.
    break;
  case 5:
    success &= true; // Ignore time zone offset field.
    break;
  default:
    success &= false; // This field is not handled.
    break;
  }

  if (!success) {
    reset();
  } else {
    toString();
  }

  return success;
}

bool ZdaMessage::updateHundredths(const char *data) {
  hundreths = 0;

  auto data_len = strlen(data);
  if (data_len < 7)
    return true; // No decimal seconds.
  else if (*(data + 6) != '.')
    return false; // Missing decimal point.
  else if (data_len < 8)
    return true; // No digits.

  uint8_t len = data_len - 7; // Get tail length.
  return numFromWord<uint32_t>(data, 7, len, &hundreths);
}

void ZdaMessage::toString() {
  sprintf(str, "%02d:%02d:%04d:%02d:%02d:%02d.%02d", day, month, year, hour,
          minute, second, hundreths);
}
