#include "helper.h"

void error(const char *string, const int number) {
#ifdef ILLUMINATION_MODULE
  digitalWrite(ILLUMINATION_PIN, LOW); // Disable illumination if its activated.
#endif
  DEBUG_PRINTLN(string);
  while (true) {
    delay(1000);
    DEBUG_PRINT("ERROR: ");
    DEBUG_PRINTDECLN(number);
  }
}
