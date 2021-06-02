////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Imu.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for IMUs in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Imu_h
#define Imu_h

#include "Arduino.h"
#include "Sensor.h"
#include <ros.h>
#include <versavis/ImuMicro.h>

enum ImuReading {
  STAT = 0,
  GX = 1,
  GY = 2,
  GZ = 3,
  AX = 4,
  AY = 5,
  AZ = 6,
  // The reminder of the entries is not identical for the different ADIS IMUs.
  // Check burst read function of the specific IMU.
};

class Imu : public Sensor {
public:
  Imu(ros::NodeHandle *nh, const String &topic, const int rate_hz,
      Timer &timer);
  virtual void setup() = 0;
  void begin();
  void triggerMeasurement();
  void publish();
  void setupPublisher();

  // Update data internally with recursion.
  virtual bool updateDataIterative() = 0;

  // Update data internally without recursion.
  virtual bool updateData() = 0;

protected:
  int16_t *sensor_data_;
  const unsigned int kMaxRecursiveUpdateDepth;
  const uint64_t kImuSyncTimeoutUs;

private:
  versavis::ImuMicro imu_msg_;
};

#endif
