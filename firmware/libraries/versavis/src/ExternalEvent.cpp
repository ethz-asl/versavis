#include "ExternalEvent.h"

#include "helper.h"
#include "versavis_configuration.h"

ExternalEvent::ExternalEvent(
    ros::NodeHandle *nh, const String &topic, const uint8_t event_pin /* = 2 */,
    const trigger_type type /*= trigger_type::NON_INVERTED */
    )
    : Sensor(nh, topic, time_msg_, type), event_pin_(event_pin) {}

void ExternalEvent::setup() { setupPublisher(); }

void ExternalEvent::triggerMeasurement() {
  DEBUG_PRINTLN((topic_ + " (ExternalEvent.cpp): Received event.").c_str());
  Sensor::setTimestampNow();
  Sensor::newMeasurementIsAvailable();
}

void ExternalEvent::publish() {
  if (Sensor::isNewMeasurementAvailable()) {
    DEBUG_PRINTLN((topic_ + " (ExternalEvent.cpp): Publish."));
    time_msg_.data = Sensor::getTimestamp();
#ifndef DEBUG
    publisher_.publish(&time_msg_);
#endif
    Sensor::newMeasurementIsNotAvailable();
  }
}

void ExternalEvent::setupPublisher() {
  // TODO(rikba): I don't know why this needs to be redeclared here. It was
  // already decleared in Sensor::Sensor constructor. If I don't declare
  // publisher_ again here, it gets a random topic (/versavis/cam0)
  publisher_ = ros::Publisher(topic_.c_str(), &time_msg_);
DEBUG_PRINT(
    (topic_ + " (ExternalEvent.cpp): Setup publisher with topic ").c_str());
DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
nh_->advertise(publisher_);
#endif
}
