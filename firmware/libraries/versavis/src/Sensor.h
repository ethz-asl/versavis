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
  Sensor(ros::NodeHandle *nh, const String &topic,
         versavis::TimeNumbered &img_time_msg,
         const trigger_type type = trigger_type::NON_INVERTED);
  Sensor(ros::NodeHandle *nh, const String &topic, versavis::ImuMicro &imu_msg,
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

  void trigger(const int pin, const int pulse_time_us,
               const trigger_type &type);

protected:
  // Trigger type. Non-inverted: The logic level is at 0 and goes to 1 for
  // triggering. Inverted: The logic level is at 1 and goes to 0 for triggering.
  const trigger_type type_;
  
  // ------------ROS members-----------
  ros::NodeHandle *nh_;
  String topic_;
  ros::Publisher publisher_;

private:
  ros::Time timestamp_;
  bool new_measurement_available_ = false;
};

#endif
