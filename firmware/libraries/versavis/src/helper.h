#ifndef helper_h
#define helper_h

#include "Arduino.h"
#include "versavis_configuration.h"

#ifdef DEBUG
#define DEBUG_INIT(x) SerialUSB.begin(x)
#define DEBUG_PRINT(x) SerialUSB.print(x)
#define DEBUG_PRINTDEC(x) SerialUSB.print(x, DEC)
#define DEBUG_PRINTHEX(x) SerialUSB.print(x, HEX)
#define DEBUG_PRINTLN(x) SerialUSB.println(x)
#define DEBUG_PRINTDECLN(x) SerialUSB.println(x, DEC)
#else
#define DEBUG_INIT(x)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTDECLN(x)
#define DEBUG_PRINTLNT(x)
#endif

extern void error(const char *string, const int number);

#endif
