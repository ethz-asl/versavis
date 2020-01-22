#include "Arduino.h"
#include "Sensor.h"
#include "helper.h"
#include "versavis_configuration.h"

// Image time message version.
Sensor::Sensor(ros::NodeHandle *nh, const String &topic,
               versavis::TimeNumbered &img_time_msg,
               const trigger_type type /* = trigger_type::NON_INVERTED */)
    : nh_(nh), topic_(topic),
      publisher_((topic + "img_time").c_str(), &img_time_msg), type_(type) {
  if (nh == nullptr) {
    error((topic_ + " (Sensor.cpp): The node handle is not available.").c_str(),
          49);
  }
  if (topic.length() == 0) {
    error((topic_ + " (Sensor.cpp): Time message topic is empty.").c_str(), 51);
  }
}

// IMU message version.
Sensor::Sensor(ros::NodeHandle *nh, const String &topic,
               versavis::ImuMicro &imu_msg,
               const trigger_type type /* = trigger_type::NON_INVERTED */)
    : nh_(nh), topic_(topic), publisher_(topic.c_str(), &imu_msg), type_(type) {
  if (nh == nullptr) {
    error((topic_ + " (Sensor.cpp): The node handle is not available.").c_str(),
          49);
  }
  if (topic.length() == 0) {
    error((topic_ + " (Sensor.cpp): IMU topic is empty.").c_str(), 51);
  }
}

void Sensor::setTimestampNow() { timestamp_ = nh_->now(); }

ros::Time Sensor::getTimestamp() const { return timestamp_; }

bool Sensor::isNewMeasurementAvailable() const {
  return new_measurement_available_;
}

void Sensor::newMeasurementIsAvailable() { new_measurement_available_ = true; }

void Sensor::newMeasurementIsNotAvailable() {
  new_measurement_available_ = false;
}

void Sensor::trigger(const int pin, const int pulse_time_us,
                     const trigger_type &type) {
  // Perform the trigger pulse.
  if (type == NON_INVERTED) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulse_time_us);
    digitalWrite(pin, LOW);
  } else if (type == INVERTED) {
    digitalWrite(pin, LOW);
    delayMicroseconds(pulse_time_us);
    digitalWrite(pin, HIGH);
  } else {
    error((topic_ + " (Sensor.cpp): Trigger type does not exist.").c_str(),
          1000);
  }
}
