#include "LidarLite.h"

#include "helper.h"
#include "versavis_configuration.h"

LidarLite::LidarLite(ros::NodeHandle *nh, const String &topic,
                     const int rate_hz, Timer &timer)
    : Sensor(nh, topic, rate_hz, timer, range_msg_) {}

void LidarLite::setup() {
  // The LidarEnhanced driver will configure this GPIO to start/stop the
  // LidarLite. We map it on an unused pins.
  const int kLidarEnablePin = 19; // Not connected.
  lidar_.begin(kLidarEnablePin);
  const bool kFastI2C = true;
  lidar_controller_.begin(kFastI2C);
  delay(100);
  lidar_controller_.add(&lidar_, kId);
  setupPublisher();
}

void LidarLite::begin() {
  DEBUG_PRINTLN((topic_ + " (LidarLite.cpp): Begin.").c_str());
  Sensor::setupTimer();
}

void LidarLite::triggerMeasurement() {
  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (LidarLite.cpp): Timer interrupt but not overflown.")
            .c_str());
    return;
  }

  DEBUG_PRINTLN((topic_ + " (LidarLite.cpp): Trigger.").c_str());
  Sensor::newMeasurementIsAvailable();
  // Reset the timer.
  timer_.resetOverflow();
}

void LidarLite::publish() {
  if (Sensor::isNewMeasurementAvailable()) {
    // Get the most recent time stamp.
    range_msg_.time.data = Sensor::getTimestamp();

    // Get signal strength.
    uint8_t nack =
        lidar_controller_.signalStrength(kId, &range_msg_.signal_strength);
    if (nack) {
      Sensor::newMeasurementIsNotAvailable();
      return;
    }

    // Get range and trigger new measurement.
    nack = lidar_controller_.distanceAndAsync(kId, &range_msg_.range);
    // New measurement triggered. Set the next time stamp.
    Sensor::setTimestampNow();
    if (nack) {
      Sensor::newMeasurementIsNotAvailable();
      return;
    }

    if (range_msg_.time.data.sec != 0 && range_msg_.time.data.nsec != 0) {
#ifndef DEBUG
      publisher_.publish(&range_msg_);
#endif
    }

    Sensor::newMeasurementIsNotAvailable();
  }
}

void LidarLite::setupPublisher() {
  publisher_ = ros::Publisher(topic_.c_str(), &range_msg_);
  DEBUG_PRINT(
      (topic_ + " (LidarLite.cpp): Setup publisher with topic ").c_str());
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}
