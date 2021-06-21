////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Camera.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding cameras in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////

#include "Camera.h"
#include "helper.h"
#include "versavis_configuration.h"

Camera::Camera(ros::NodeHandle *nh, const String &topic, const int rate_hz,
               Timer &timer, const trigger_type &type,
               const uint8_t trigger_pin /*= 0 */,
               const uint8_t exposure_pin /*= 0 */,
               const bool exposure_compensation /*= true*/)
    : TimedSensor(nh, topic, rate_hz, timer, image_time_msg_, type),
      trigger_pin_(trigger_pin), exposure_pin_(exposure_pin),
      exposure_compensation_(exposure_compensation), is_configured_(true),
      compensating_(false), exposing_(false), image_number_(0),
      init_subscriber_((topic + "init").c_str(), &Camera::initCallback, this),
      initialized_(false) {
  // Check the input.
  if (trigger_pin == 0) {
    error((topic_ + " (Camera.cpp): Trigger pin is set to 0.").c_str(), 10);
  }
  if (exposure_pin == 0) {
    error((topic_ + " (Camera.cpp): Exposure pin is set to 0.").c_str(), 10);
  }
  Sensor::newMeasurementIsNotAvailable();
}

void Camera::setup() {
  if (topic_.length() == 0) {
    // Cameras without a topic are considered as disconnected.
    DEBUG_PRINTLN(
        F("NO_TOPIC (Camera.cpp): Skip camera setup for disconnected camera."));
    is_configured_ = false;
    initialized_ = true;
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Camera.cpp): Setup.").c_str());

  setupInitSubscriber();
  setupPublisher();

  pinMode(trigger_pin_, OUTPUT);
  digitalWrite(trigger_pin_, LOW);
  pinMode(exposure_pin_, INPUT);
}

void Camera::initialize() {
  if (!is_configured_ || initialized_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Camera.cpp): Initialize.").c_str());
  Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US * 10, type_);
  Sensor::setTimestampNow();
  ++image_number_;
  image_time_msg_.number = image_number_;
  Sensor::newMeasurementIsAvailable();
  publish();
#ifdef DEBUG
  initialized_ = true;
#endif
}

void Camera::begin() {
  if (!is_configured_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Camera.cpp): Begin.").c_str());
  // Maximal exposure time to still be able to keep up with the frequency
  // considering a security factor of 0.99, in us.
  max_exposure_time_us_ = 0.99 * 1e6 / rate_hz_;

  // Setup timer to periodically trigger the camera.
  TimedSensor::setupTimer();
}

void Camera::setupPublisher() {
  pub_topic_ = topic_ + "image_time";
  publisher_ = ros::Publisher(pub_topic_.c_str(), &image_time_msg_);
  DEBUG_PRINT((topic_ + " (Camera.cpp): Setup publisher with topic ").c_str());
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}

void Camera::setupInitSubscriber() {
  init_sub_topic_ = topic_ + "init";
  init_subscriber_ = ros::Subscriber<std_msgs::Bool, Camera>(
      init_sub_topic_.c_str(), &Camera::initCallback, this);
  DEBUG_PRINT(
      (topic_ + " (Camera.cpp): Setup init subscriber with topic ").c_str());
  DEBUG_PRINTLN(init_subscriber_.topic_);
#ifndef DEBUG
  nh_->subscribe(init_subscriber_);
#endif
}

void Camera::initCallback(const std_msgs::Bool &msg) {
  initialized_ = msg.data;
}

void Camera::triggerMeasurement() {
  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (Camera.cpp): Timer interrupt but not overflown.").c_str());
    return;
  }
  if (!is_configured_ || !initialized_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Camera.cpp): Timer overflow.").c_str());

  if (exposure_compensation_ && compensating_) {
    DEBUG_PRINTLN((topic_ + " (Camera.cpp): Compensating.").c_str());
    // Exposure-time compensating mode (Nikolic 2014). During exposure, the
    // timer will interrupt in the middle of the exposure time. At the end of
    // the exposure, the external interrupt will trigger exposureEnd() and
    // reset the timer to trigger the camera at the appropriate time.
    if (!exposing_) {
      DEBUG_PRINTLN(
          (topic_ + " (Camera.cpp): Not exposing. Trigger camera.").c_str());
      // The camera is currently not exposing meaning that the interrupt
      // triggers at the beginning of the next image.
      exposing_ = true;

#ifdef ILLUMINATION_MODULE
      // If the illumination module is active, the LEDs should turn on just
      // before the camera is exposing.
      digitalWrite(ILLUMINATION_PIN, HIGH);
      // Here, a warm-up delay for the LEDs can be added (needs checking).
      // delayMicroseconds(10);
#endif
      // Trigger the actual pulse.
      Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

      // Save the current time to estimate the exposure time in the pin
      // interrupt.
      exposure_start_us_ = micros();

      // Increament the image number as the camera is triggered now.
      ++image_number_;

      // Set the timer to the mid exposure point, e.g. half the exposure time.
      timer_.setCompare(exposure_delay_ticks_ > 0 ? exposure_delay_ticks_ - 1
                                                  : compare_);
    } else {
      DEBUG_PRINTLN(
          (topic_ + " (Camera.cpp): Exposing right now, get timestamp.")
              .c_str());
      // The camera is currently exposing. In this case, the interrupt is
      // triggered in the middle of the exposure time, where the timestamp
      // should be taken.
      Sensor::setTimestampNow();
      Sensor::newMeasurementIsAvailable();
#ifdef ADD_TRIGGERS
      trigger(ADDITIONAL_TEST_PIN, TRIGGER_PULSE_US,
              Sensor::trigger_type::NON_INVERTED);
#endif

      // Even though we are still in the compensating mode, deactivating here
      // ensures that we detect if a exposure signal is dropped and we switch
      // to non-compensating mode.
      compensating_ = false;

      // Set the timer to the standard period as we dont know the current
      // exposure time yet.
      timer_.setCompare(compare_);
    }
  } else {
    // "Standard" mode where the camera is triggered purely periodic.
    DEBUG_PRINTLN(
        (topic_ +
         " (Camera.cpp): Not compensating. Trigger camera and take timestamp.")
            .c_str());
    exposing_ = true;

#ifdef ILLUMINATION_MODULE
    // Deactivate the LEDs as we are not sure yet whether we get an exposure
    // signal.
    digitalWrite(ILLUMINATION_PIN, LOW);
#endif
    // Trigger the actual pulse.
    Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

    Sensor::setTimestampNow();
    Sensor::newMeasurementIsAvailable();

    // Save the current time to estimate the exposure time in the pin
    // interrupt.
    exposure_start_us_ = micros();

    // Increament the image number as the camera is triggered now.
    image_number_++;

    // Set the timer to make sure that the camera is triggered in a periodic
    // mode.
    timer_.setCompare(compare_);
  }
  // Reset the timer.
  timer_.resetOverflow();
}

void Camera::exposureEnd() {
  DEBUG_PRINTLN((topic_ + " (Camera.cpp): Exposure end.").c_str());
  if (exposure_compensation_) {
    unsigned long last_exposure_time_us = micros() - exposure_start_us_;
    DEBUG_PRINT((topic_ + " (Camera.cpp): exposure time [us] ").c_str());
    DEBUG_PRINTDECLN(last_exposure_time_us);
    calculateDelayTicksAndCompensate(last_exposure_time_us);
    exposing_ = false;
  }
}

void Camera::publish() {
  if (Sensor::isNewMeasurementAvailable()) {
    DEBUG_PRINTLN((topic_ + " (Camera.cpp): Publish.").c_str());
    image_time_msg_.time = Sensor::getTimestamp();
    image_time_msg_.number = image_number_;
#ifndef DEBUG
    publisher_.publish(&image_time_msg_);
#endif
    Sensor::newMeasurementIsNotAvailable();
  }
}

void Camera::calculateDelayTicksAndCompensate(
    const unsigned long &last_exposure_time_us) {
  // The goal here is to shift the time of the next camera trigger half the
  // exposure time before the mid-exposure time.

  // The next frame should be triggered by this time before the mid-exposure
  // time (in CPU ticks).
  if (last_exposure_time_us == 0 ||
      last_exposure_time_us >= max_exposure_time_us_) {
    // In this case, something with the measurement went wrong or the camera
    // is dropping triggers due to a too high exposure time (constrain the
    // maximal exposure time of your camera to be within the period time of
    // your triggering). Switch to non-compensating mode.
    exposure_delay_ticks_ = 0;
    compensating_ = false;
  } else {
    exposure_delay_ticks_ = static_cast<double>(last_exposure_time_us) / 2.0 /
                            1000000.0 * cpu_freq_prescaler_;
    compensating_ = true;
  }

  // Reset the compare register of the timer.
  timer_.setCompare(compare_ - exposure_delay_ticks_);
}
