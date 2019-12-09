////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Timer.h
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides interfacing functions for timers for ARM processors
//  (tested Ardnuino Zero). Refer to the parent package versavis for
//  license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Timer_h
#define Timer_h

#include "Arduino.h"

// timers Class Definition
class Timer {
public:
  // Constructor for TCC timer type.
  Timer(Tcc *tcc_timer);

  // Constructor for TcCount16 timer type.
  Timer(TcCount16 *tc_timer);

  // Initialize
  void initialize(const uint16_t prescaler, const long &compare,
                  const String &topic);

  // Check whether an overflow triggered the interrupt.
  bool checkOverflow() const;

  // Reset the overflow flag.
  void resetOverflow();

  // Set a new compare register for the interupt.
  void setCompare(const long &compare);

private:
  // Pointers to the actual timer. Depends on the type.
  Tcc *tcc_timer_;
  TcCount16 *tc_timer_;

  // Flag whether the timer is tcc or TcCount16
  bool is_tcc_;
};

#endif
