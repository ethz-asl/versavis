////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ExternalEvent.h
////////////////////////////////////////////////////////////////////////////////
//
//  An external event is triggered on external interrupts and publishes its
//  timestamp.
//  Tested pins: 2 (PA14)
//
////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <std_msgs/Time.h>

#include "Sensor.h"

class ExternalEvent : public Sensor {
public:
  ExternalEvent(ros::NodeHandle *nh, const String &topic,
                const uint8_t event_pin = 2,
                const trigger_type type = trigger_type::NON_INVERTED);

  void setup() override;
  void triggerMeasurement() override;
  void publish() override;
  void setupPublisher() override;

  inline uint8_t eventPin() const { return event_pin_; }

private:
  std_msgs::Time time_msg_;
  const uint8_t event_pin_;
};
