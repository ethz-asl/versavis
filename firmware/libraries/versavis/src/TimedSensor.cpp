#include "Arduino.h"
#include "TimedSensor.h"
#include "helper.h"
#include "versavis_configuration.h"

// Image time message version.
TimedSensor::TimedSensor(
    ros::NodeHandle *nh, const String &topic, const int rate_hz, Timer &timer,
    versavis::TimeNumbered &img_time_msg,
    const trigger_type type /* = trigger_type::NON_INVERTED */)
    : Sensor(nh, topic, img_time_msg, type), rate_hz_(rate_hz), timer_(timer) {
  if (rate_hz <= 0) {
    error((topic_ +
           " (TimedSensor.cpp): The rate of a sensor needs to be positive.")
              .c_str(),
          50);
  }
}

// IMU message version.
TimedSensor::TimedSensor(
    ros::NodeHandle *nh, const String &topic, const int rate_hz, Timer &timer,
    versavis::ImuMicro &imu_msg,
    const trigger_type type /* = trigger_type::NON_INVERTED */)
    : Sensor(nh, topic, imu_msg, type), rate_hz_(rate_hz), timer_(timer) {
  if (rate_hz <= 0) {
    error((topic_ +
           " (TimedSensor.cpp): The rate of a sensor needs to be positive.")
              .c_str(),
          50);
  }
}

uint16_t TimedSensor::calculatePrescaler(const uint16_t rate_hz) const {
  // All timers on Arduino Zero are assumed to be set to 16 bit.
  const uint32_t kMaxCount = 65536;

  // Amount of clock ticks required to count to the specified period time.
  const uint32_t kRequiredTicks = CPU_FREQ_HZ / static_cast<double>(rate_hz);
  DEBUG_PRINT((topic_ + " (TimedSensor.cpp): required ticks "));
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
  error((topic_ + " (TimedSensor.cpp): No prescaler found.").c_str(), 50);
  return 0;
}

void TimedSensor::setupTimer() {
  prescaler_ = calculatePrescaler(rate_hz_);
  if (prescaler_ == 0) {
    error((topic_ + " (TimedSensor.cpp): prescaler_ is zero.").c_str(), 99);
  }
  cpu_freq_prescaler_ = CPU_FREQ_HZ / prescaler_;
  DEBUG_PRINT(
      (topic_ + " (TimedSensor.cpp): Setup timer with prescaler ").c_str());
  DEBUG_PRINTDECLN(prescaler_);
  // The compare value defines at which tick the timer is going to overflow
  // and triggering the corresponding interrupt.
  compare_ = 1.0 / rate_hz_ * cpu_freq_prescaler_ - 1;
  if (compare_ > max_compare_) {
    // The compare value can never be reached by a 16bit timer. Choose a
    // higher prescaler or another timer with 32bit.
    error((topic_ + " (TimedSensor.cpp): compare_ > pow(2,16) failed.").c_str(),
          100);
  }

  // Actually setting up the timer registers.
  timer_.initialize(prescaler_, compare_, topic_);
}
