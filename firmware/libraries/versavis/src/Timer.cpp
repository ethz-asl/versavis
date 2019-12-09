#include "Timer.h"
#include "helper.h"
#include "versavis_configuration.h"

// Constructor for TCC timer type.
Timer::Timer(Tcc *tcc_timer) : tcc_timer_(tcc_timer), is_tcc_(true) {
  if (tcc_timer == nullptr) {
    error("NO_TOPIC (Timer.cpp): TCC timer does not exist.", 201);
  }
}

// Constructor for TcCount16 timer type.
Timer::Timer(TcCount16 *tc_timer) : tc_timer_(tc_timer), is_tcc_(false) {
  if (tc_timer == nullptr) {
    error("NO_TOPIC (Timer.cpp): TC timer does not exist.", 201);
  }
}

bool Timer::checkOverflow() const {
  if (is_tcc_) {
    return tcc_timer_->INTFLAG.bit.OVF == 1;
  } else {
    return tc_timer_->INTFLAG.bit.OVF == 1;
  }
}

void Timer::resetOverflow() {
  // Writing a one clears the ovf flag of the timer.
  if (is_tcc_) {
    tcc_timer_->INTFLAG.bit.OVF = 1;
  } else {
    tc_timer_->INTFLAG.bit.OVF = 1;
  }
}

void Timer::initialize(const uint16_t prescaler, const long &compare,
                       const String &topic) {
  DEBUG_PRINTLN((topic + " (Timer.cpp): Initialize.").c_str());
  if (is_tcc_) {
    DEBUG_PRINTLN((topic + " (Timer.cpp): is_tcc.").c_str());
    // -------------- For TCC timers -----------------
    tcc_timer_->CTRLA.reg &= ~TCC_CTRLA_ENABLE; // Disable the timers
    while (tcc_timer_->SYNCBUSY.bit.ENABLE == 1) {
      ; // wait for sync.
    }

    // Set bits for prescaler (32, 128 and 512 doesn’t work)
    switch (prescaler) {
    case 1:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;
      break;
    case 2:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV2;
      break;
    case 4:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV4;
      break;
    case 8:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV8;
      break;
    case 16:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV16;
      break;
    case 64:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV64;
      break;
    case 256:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV256;
      break;
    case 1024:
      tcc_timer_->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1024;
      break;
    default:
      error((topic + " (Timer.cpp): Error 200: Prescaler is not available.")
                .c_str(),
            200);
      break;
    }

    tcc_timer_->WAVE.reg |=
        TCC_WAVE_WAVEGEN_MFRQ; // Set wave form on match frequency configuration
                               // -> CC0 match.
    while (tcc_timer_->SYNCBUSY.bit.WAVE == 1) {
      ; // wait for sync.
    }

    tcc_timer_->CC[0].reg =
        compare; // define period: rate_hz, also control the prescaler!
    while (tcc_timer_->SYNCBUSY.bit.CC0 == 1) {
      ; // wait for sync.
    }

    tcc_timer_->INTENSET.reg = 0;     // disable all interrupts.
    tcc_timer_->INTENSET.bit.OVF = 1; // enable overfollow.

    tcc_timer_->CTRLA.reg |= TCC_CTRLA_ENABLE; // enable timer.
    while (tcc_timer_->SYNCBUSY.bit.ENABLE == 1) {
      ;
    }
  } else {
    DEBUG_PRINTLN((topic + " (Timer.cpp): is_tc.").c_str());
    // -------------- For TC timers -----------------
    tc_timer_->CTRLA.reg &= ~TC_CTRLA_ENABLE; // Disable the timers
    while (tc_timer_->STATUS.bit.SYNCBUSY == 1) {
      ; // wait for sync
    }
    // Set bits for prescaler (32, 128 and 512 doesn’t work)
    switch (prescaler) {
    case 1:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;
      break;
    case 2:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;
      break;
    case 4:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4;
      break;
    case 8:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;
      break;
    case 16:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;
      break;
    case 64:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64;
      break;
    case 256:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;
      break;
    case 1024:
      tc_timer_->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
      break;
    default:
      error((topic + " (Timer.cpp): Error 200: Prescaler is not available.")
                .c_str(),
            200);
      break;
    }

    tc_timer_->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    while (tc_timer_->STATUS.bit.SYNCBUSY == 1) {
      ; // wait for sync.
    }
    tc_timer_->CTRLA.reg |=
        TC_CTRLA_WAVEGEN_MFRQ; // Set wave form on match frequency
                               // configuration -> CC0 match.
    while (tc_timer_->STATUS.bit.SYNCBUSY == 1) {
      ; // wait for sync.
    }

    tc_timer_->CC[0].reg =
        compare; // define period: rate_hz, also control the prescaler!
    while (tc_timer_->STATUS.bit.SYNCBUSY == 1) {
      ; // wait for sync.
    }

    tc_timer_->INTENSET.reg = 0;     // disable all interrupts.
    tc_timer_->INTENSET.bit.OVF = 1; // enable overfollow.

    tc_timer_->CTRLA.reg |= TC_CTRLA_ENABLE; // enable timer.
    while (tc_timer_->STATUS.bit.SYNCBUSY == 1) {
      ; // wait for sync.
    }
  }
}

void Timer::setCompare(const long &compare) {
  if (is_tcc_) {
    tcc_timer_->CC[0].reg = compare;
    while (tcc_timer_->SYNCBUSY.bit.CC0 == 1) {
      ; // wait for sync.
    }
  } else {
    tc_timer_->CC[0].reg = compare;
    while (tc_timer_->STATUS.bit.SYNCBUSY == 1) {
      ; // wait for sync.
    }
  }
}
