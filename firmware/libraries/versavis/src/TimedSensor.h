////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  TimedSensor.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for generic sensors in the versavis framework that is
//  controlled by a timer.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TimedSensor_h
#define TimedSensor_h

#include "Arduino.h"
#include "Sensor.h"
#include "Timer.h"
#include <ros.h>
#include <versavis/ImuMicro.h>
#include <versavis/TimeNumbered.h>

class TimedSensor : public Sensor {
public:
  TimedSensor(ros::NodeHandle *nh, const String &topic, const int rate_hz,
              Timer &timer, versavis::TimeNumbered &img_time_msg,
              const trigger_type type = trigger_type::NON_INVERTED);
  TimedSensor(ros::NodeHandle *nh, const String &topic, const int rate_hz,
              Timer &timer, versavis::ImuMicro &imu_msg,
              const trigger_type type = trigger_type::NON_INVERTED);

  void setupTimer();

  //------------Timer members-----------
protected:
  // Frequency of the timer in Hz.
  uint16_t rate_hz_;
  // The timer object (can be TCC or TcCount16).
  Timer timer_;

  // Compare register to use for timer overlow / interrupt.
  unsigned long compare_;
  const unsigned long max_compare_ = pow(2, 16);
  // Prescaler of the timer, which is set automatically.
  uint16_t prescaler_;
  double cpu_freq_prescaler_;

private:
  // Function to determine prescaler based on timer size and frequency.
  uint16_t calculatePrescaler(const uint16_t rate_hz) const;
};

#endif
