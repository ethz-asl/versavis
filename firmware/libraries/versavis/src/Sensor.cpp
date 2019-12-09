#include "Arduino.h"
#include "Sensor.h"
#include "helper.h"
#include "versavis_configuration.h"

// Image time message version.
Sensor::Sensor(ros::NodeHandle *nh, const String &topic, const int rate_hz,
               Timer &timer, versavis::TimeNumbered &img_time_msg,
               const trigger_type type /* = trigger_type::NON_INVERTED */)
    : nh_(nh), topic_(topic),
      publisher_((topic + "img_time").c_str(), &img_time_msg),
      new_measurement_available_(false), rate_hz_(rate_hz), timer_(timer),
      type_(type), max_compare_(pow(2, 16)) {
  if (nh == nullptr) {
    error((topic_ + " (Sensor.cpp): The node handle is not available.").c_str(),
          49);
  }
  if (rate_hz <= 0) {
    error((topic_ + " (Sensor.cpp): The rate of a sensor needs to be positive.")
              .c_str(),
          50);
  }
}

// IMU message version.
Sensor::Sensor(ros::NodeHandle *nh, const String &topic, const int rate_hz,
               Timer &timer, versavis::ImuMicro &imu_msg,
               const trigger_type type /* = trigger_type::NON_INVERTED */)
    : nh_(nh), topic_(topic), publisher_(topic.c_str(), &imu_msg),
      new_measurement_available_(false), rate_hz_(rate_hz), timer_(timer),
      type_(type), max_compare_(pow(2, 16)) {
  if (nh == nullptr) {
    error((topic_ + " (Sensor.cpp): The node handle is not available.").c_str(),
          49);
  }
  if (rate_hz <= 0) {
    error((topic_ + " (Sensor.cpp): The rate of a sensor needs to be positive.")
              .c_str(),
          50);
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

uint16_t Sensor::calculatePrescaler(const uint16_t rate_hz) const {
  // All timers on Arduino Zero are assumed to be set to 16 bit.
  const uint32_t kMaxCount = 65536;

  // Amount of clock ticks required to count to the specified period time.
  const uint32_t kRequiredTicks = CPU_FREQ_HZ / static_cast<double>(rate_hz);
  DEBUG_PRINT((topic_ + " (Sensor.cpp): required ticks "));
  DEBUG_PRINTDECLN(kRequiredTicks);
  // Available prescalers on Arduino Zero are restricted to those values.
  const uint16_t kAvailablePrescalers[8] = {1, 2, 4, 8, 16, 64, 256, 1024};

  for (uint8_t i = 0; i < 8; ++i) {
    if (kMaxCount > kRequiredTicks / kAvailablePrescalers[i] &&
        kRequiredTicks % kAvailablePrescalers[i] == 0) {
      return kAvailablePrescalers[i];
    }
  }
  // If this part is reached, no available prescaler fits with the goal
  // framerate on this MCU.
  error((topic_ + " (Sensor.cpp): No prescaler found.").c_str(), 50);
  return 0;
}

void Sensor::setupTimer() {
  prescaler_ = calculatePrescaler(rate_hz_);
  if (prescaler_ == 0) {
    error((topic_ + " (Sensor.cpp): prescaler_ is zero.").c_str(), 99);
  }
  cpu_freq_prescaler_ = CPU_FREQ_HZ / prescaler_;
  DEBUG_PRINT((topic_ + " (Sensor.cpp): Setup timer with prescaler ").c_str());
  DEBUG_PRINTDECLN(prescaler_);
  // The compare value defines at which tick the timer is going to overflow
  // and triggering the corresponding interrupt.
  compare_ = 1.0 / rate_hz_ * cpu_freq_prescaler_ - 1;
  if (compare_ > max_compare_) {
    // The compare value can never be reached by a 16bit timer. Choose a
    // higher prescaler or another timer with 32bit.
    error((topic_ + " (Sensor.cpp): compare_ > pow(2,16) failed.").c_str(),
          100);
  }

  // Actually setting up the timer registers.
  timer_.initialize(prescaler_, compare_, topic_);
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
