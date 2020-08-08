////////////////////////////////////////////////////////////////////////////////
//  May 2015
//  Author: light Dynamics and Control Lab
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in VersaVIS
////////////////////////////////////////////////////////////////////////////////
//  VN100.h
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018 Flight Dynamics and Control Lab

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// NOTE: Before running this code, you must configure the IMU as
// defined in the README file.
////////////////////////////////////////////////////////////////////////////////

#ifndef VN100_h
#define VN100_h
#include "Arduino.h"
#include "Imu.h"
#include <SPI.h>
#include <ros.h>

// ADIS16448BMLZ Class Definition
class VN100 : public Imu {
public:
  // ADIS16448BMLZ Constructor (ChipSelect, DataReady output pin, HardwareReset)
  VN100(ros::NodeHandle *nh, const String &topic, const int rate_hz,
        Timer &timer);
  ~VN100();
  void setup();

  // Update data internally with validity checks.
  bool updateDataIterative();

  // Update data withoput recursion.
  bool updateData();

private:
  // Read the IMU bytes
  int16_t *readImuData();

  // Calculate the 16-bit CRC for the given ASCII or binary message.
  unsigned short checksum(byte data[], size_t length, bool ignore_first_byte);

  // Clear Serial buffer for any remaining data.
  void clearBuffer();

  // Union functions for byte to float conversions
  // IMU sends data as bytes, the union functions are used to convert
  // these data into other data types

  // Angular rates
  union {
    float f;
    byte b[4];
  } W_x_;
  union {
    float f;
    byte b[4];
  } W_y_;
  union {
    float f;
    byte b[4];
  } W_z_;

  // Acceleration
  union {
    float f;
    byte b[4];
  } a_x_;
  union {
    float f;
    byte b[4];
  } a_y_;
  union {
    float f;
    byte b[4];
  } a_z_;

  // Checksum
  union {
    unsigned short s;
    byte b[2];
  } checksum_;

  // Parameters
  //byte in_[30]; // array to save data send from the IMU
  const size_t kMessageLength;
  byte* in_; // array to save data send from the IMU
  float imu_accelerator_sensitivity_;
  float imu_gyro_sensitivity_;
};

#endif
