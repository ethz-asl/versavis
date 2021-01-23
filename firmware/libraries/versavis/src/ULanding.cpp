#include "ULanding.h"

#include "helper.h"
#include "versavis_configuration.h"

ULanding::ULanding(ros::NodeHandle *nh, const String &topic, const int rate_hz,
                   Timer &timer, Uart *uart)
    : Sensor(nh, topic, rate_hz, timer, range_msg_), uart_(uart) {}

void ULanding::setup() {
  if (uart_)
    uart_->begin(115200);

  setupPublisher();
}

void ULanding::begin() {
  DEBUG_PRINTLN((topic_ + " (ULanding.cpp): Begin.").c_str());
  Sensor::setupTimer();
}

void ULanding::triggerMeasurement() {
  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (ULanding.cpp): Timer interrupt but not overflown.")
            .c_str());
    return;
  }

  DEBUG_PRINTLN((topic_ + " (ULanding.cpp): Trigger.").c_str());
  Sensor::newMeasurementIsAvailable();
  // Reset the timer.
  timer_.resetOverflow();
}

void ULanding::publish() {
  // https://aerotenna.readme.io/docs/receiving-data-new#section-example-arduino-code-for-receivingshowing-data-on-pc

  if (uart_ && Sensor::isNewMeasurementAvailable()) {

    const uint8_t kMessageLength = 6;
    const uint8_t kMinBufferSize = 2 * kMessageLength;
    const uint8_t kBufferSizeAvailable = uart_->available();
    // At least have one full message.
    if (kBufferSizeAvailable >= kMinBufferSize) {

      // Read all available bytes from serial buffer.
      uint8_t data[kBufferSizeAvailable];
      for (uint8_t i = 0; i < kBufferSizeAvailable; i++) {
        data[i] = uart_->read();
      }

      // Get the previously set timestamp.
      range_msg_.time.data = Sensor::getTimestamp();

      // On Arduino the Serial buffer automatically fills up with incoming data
      // until the buffer is full. The best timestamp we can set is now, to
      // stamp the next arriving sample after the buffer has been emptied.
      Sensor::setTimestampNow();

      // Sync to beginning of message.
      const uint8_t kHeader = 0xFE;
      const uint8_t kVersionId = 0x01;
      uint8_t offset = 0;
      bool offset_found = false;
      for (int i = 0; i < kMessageLength; i++) {
        if (data[i] == kHeader && data[i + 1] == kVersionId) {
          offset = i;
          offset_found = true;
          break;
        }
      }

      // Use this message.
      if (offset_found) {
        range_msg_.range = data[offset + 2] + (data[offset + 3] << 8);
        range_msg_.signal_strength = data[offset + 4];
        uint8_t checksum = (data[offset + 1] + data[offset + 2] +
                            data[offset + 3] + data[offset + 4]) &
                           0xff;

        if (range_msg_.time.data.sec != 0 && range_msg_.time.data.nsec != 0 &&
            checksum == data[offset + 5]) {
#ifndef DEBUG
          publisher_.publish(&range_msg_);
#endif
        }
      }
    }

    Sensor::newMeasurementIsNotAvailable();
  }
}

void ULanding::setupPublisher() {
  publisher_ = ros::Publisher(topic_.c_str(), &range_msg_);
  DEBUG_PRINT(
      (topic_ + " (ULanding.cpp): Setup publisher with topic ").c_str());
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}
