////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Camera.h
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding cameras in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Camera_h
#define Camera_h

#include "Arduino.h"
#include "Sensor.h"
#include <ros.h>
#include <std_msgs/Bool.h>
#include <versavis/TimeNumbered.h>

// timers Class Definition
class Camera : public Sensor {
public:
  Camera(ros::NodeHandle *nh, const String &topic, const int rate_hz,
         Timer &timer, const trigger_type &type, const uint8_t trigger_pin = 0,
         const uint8_t exposure_pin = 0,
         const bool exposure_compensation = true);

  void begin();
  int exposurePin() const { return exposure_pin_; };
  bool isExposing() const { return exposing_; };
  bool isInitialized() const { return initialized_; };

  void triggerMeasurement();
  void exposureEnd();

  void publish();
  void setup();      // Setup publishers and pins.
  void initialize(); // Perform initialization procedure.
  void setupPublisher();
  void setupInitSubscriber();
  void
  initCallback(const std_msgs::Bool &msg); // Gets called as soon as the host
                                           // reports the camera as initialized.

  void
  calculateDelayTicksAndCompensate(const unsigned long &last_exposure_time_us);

private:
  // Message to save sequence number and timestamp.
  versavis::TimeNumbered image_time_msg_;

  // Hardware pins for trigger and exposure signal.
  const uint8_t trigger_pin_;
  const uint8_t exposure_pin_;

  // Flag whether the camera should perform exposure compensation and is
  // currently in compensating mode (see Nikolic 2014).
  const bool exposure_compensation_;
  bool is_configured_;
  bool compensating_;

  // Flag whether the camera is currently exposing or not.
  bool exposing_;

  // Maximum exposure time to check whether measurement is valid.
  uint32_t max_exposure_time_us_;

  // Time when exposure started.
  uint64_t exposure_start_us_;

  // The image number is a strictly incrasing number used for data association
  // of time stamp and image.
  uint32_t image_number_;

  // Time the next frame should be triggered before the mid-exposure time (in
  // CPU ticks).
  unsigned long exposure_delay_ticks_;

  // ROS related.
  String pub_topic_;
  String init_sub_topic_;
  ros::Subscriber<std_msgs::Bool, Camera> init_subscriber_;
  bool initialized_;
};

#endif
