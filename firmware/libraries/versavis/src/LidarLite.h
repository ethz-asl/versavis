////////////////////////////////////////////////////////////////////////////////
//  December 2019
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  LidarLite.h
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation for Garmin Lidar Lite in the versavis framework. Refer to the
//  parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef LidarLite_h
#define LidarLite_h

#include <LidarController.h>
#include <LidarObject.h>
#include <Sensor.h>
#include <versavis/RangeMicro.h>

class LidarLite : public Sensor {
public:
  LidarLite(ros::NodeHandle *nh, const String &topic, const int rate_hz,
            Timer &timer);
  void setup() override;
  void begin() override;
  void triggerMeasurement() override;
  void publish() override;
  void setupPublisher() override;

private:
  // Disable copy / assignment constructors.
  LidarLite(const LidarLite &) = delete;
  LidarLite &operator=(const LidarLite &) = delete;

  const uint8_t kId = 0;

  versavis::RangeMicro range_msg_;

  LidarController lidar_controller_;
  LidarObject lidar_;
};

#endif
