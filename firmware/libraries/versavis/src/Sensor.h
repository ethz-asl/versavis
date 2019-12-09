////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Sensor.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for generic sensors in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"
#include "Timer.h"
#include <ros.h>
#include <versavis/ImuMicro.h>
#include <versavis/TimeNumbered.h>

enum trigger_type { INVERTED, NON_INVERTED };

class Sensor {
public:
  Sensor(ros::NodeHandle *nh, const String &topic, const int rate_hz,
         Timer &timer, versavis::TimeNumbered &img_time_msg,
         const trigger_type type = trigger_type::NON_INVERTED);
  Sensor(ros::NodeHandle *nh, const String &topic, const int rate_hz,
         Timer &timer, versavis::ImuMicro &imu_msg,
         const trigger_type type = trigger_type::NON_INVERTED);
  inline virtual void setup(){/* do nothing */};
  inline virtual void begin(){/* do nothing */};
  inline virtual void triggerMeasurement() = 0;
  inline virtual void publish() = 0;
  inline virtual void setupPublisher() = 0;

  void setTimestampNow();
  ros::Time getTimestamp() const;
  bool isNewMeasurementAvailable() const;
  void newMeasurementIsAvailable();
  void newMeasurementIsNotAvailable();

  void setupTimer();

  void trigger(const int pin, const int pulse_time_us,
               const trigger_type &type);

  // ------------ROS members-----------
protected:
  ros::NodeHandle *nh_;
  String topic_;
  ros::Publisher publisher_;

private:
  ros::Time timestamp_;
  bool new_measurement_available_;

  //------------Timer members-----------
protected:
  // Frequency of the timer in Hz.
  uint16_t rate_hz_;
  // The timer object (can be TCC or TcCount16).
  Timer timer_;
  // Trigger type. Non-inverted: The logic level is at 0 and goes to 1 for
  // triggering. Inverted: The logic level is at 1 and goes to 0 for triggering.
  const trigger_type type_;

  // Compare register to use for timer overlow / interrupt.
  unsigned long compare_;
  const unsigned long max_compare_;
  // Prescaler of the timer, which is set automatically.
  uint16_t prescaler_;
  double cpu_freq_prescaler_;

private:
  // Function to determine prescaler based on timer size and frequency.
  uint16_t calculatePrescaler(const uint16_t rate_hz) const;
};

#endif
