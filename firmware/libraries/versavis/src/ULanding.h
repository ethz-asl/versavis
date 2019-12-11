////////////////////////////////////////////////////////////////////////////////
//  December 2019
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ULanding.h
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation for Aerotenna uLanding in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ULanding_h
#define ULanding_h

#include <Sensor.h>
#include <versavis/RangeMicro.h>
#include <Uart.h>

class ULanding : public Sensor {
public:
  ULanding(ros::NodeHandle *nh, const String &topic, const int rate_hz,
           Timer &timer, Uart* uart);
  void setup() override;
  void begin() override;
  void triggerMeasurement() override;
  void publish() override;
  void setupPublisher() override;

private:
  // Disable copy / assignment constructors.
  ULanding(const ULanding &) = delete;
  ULanding &operator=(const ULanding &) = delete;

  Uart* uart_;

  versavis::RangeMicro range_msg_;
};

#endif
