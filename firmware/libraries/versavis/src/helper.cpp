#include "helper.h"

void error(const char *string, const int number) {
#ifdef ILLUMINATION_MODULE
  digitalWrite(ILLUMINATION_PIN, LOW); // Disable illumination if its activated.
#endif
  DEBUG_PRINTLN(string);
  noInterrupts();
  DEBUG_PRINT("ERROR: " + String(number));
  while (true) {
    ; // TODO(floriantschopp) Find a better way to stop execution but still be
      // able to interrupt.
  }
}
