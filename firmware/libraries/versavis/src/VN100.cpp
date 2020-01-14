////////////////////////////////////////////////////////////////////////////////
//  May 2015
//  Author: light Dynamics and Control Lab
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in VersaVIS
////////////////////////////////////////////////////////////////////////////////
//  VN100.cpp
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

#include "VN100.h"
#include "helper.h"
#include "versavis_configuration.h"

VN100::VN100(ros::NodeHandle *nh, const String &topic, const int rate_hz,
             Timer &timer)
    : Imu(nh, topic, rate_hz, timer) {
  imu_accelerator_sensitivity_ = 1.0 / (0.00025 * 9.81);
  imu_gyro_sensitivity_ = 1.0 / (0.05 * M_PI / 180);
}

void VN100::setup() {
  DEBUG_PRINTLN((topic_ + " (VN100.cpp): Setup.").c_str());
  // Start Serial1 for IMU communication
  Serial1.begin(921600, SERIAL_8N1);
  delay(20);
  Serial.setTimeout(2);

  // Check whether IMU is setup correctly. Configuration is done on VectorNav
  // Control Center software.
  Serial1.print("$VNRRG,00*XX\r\n");
  delay(20);
  byte response[100];
  Serial1.readBytes(response, Serial1.available());

  // Check that "User Tag" is set to 7665727361766973 (HEX for versavis).
  if (strcmp((char *)response, "$VNRRG,00,7665727361766973*51\r\n") != 0) {
    error((topic_ + " (VN100.cpp): IMU is not setup correctly.").c_str(), 11);
  }
  Imu::setupPublisher();
}

////////////////////////////////////////////////////////////////////////////
// Read the IMU bytes.
////////////////////////////////////////////////////////////////////////////
int16_t *VN100::readImuData() {
  clearBuffer();
  Serial1.print("$VNBOM,1*XX\r\n");
  const uint64_t local_tic = micros();
  while (Serial1.available() < 30) {
    if (micros() - local_tic < kImuSyncTimeoutUs / 2) {
      delayMicroseconds(1); // Wait until full message is received.
    } else {
      DEBUG_PRINTLN(topic_ +
                    " (VN100.cpp): Serial data never available (timeout?).");
      return nullptr;
    }
  }
  size_t bytes = Serial1.readBytes(in_, 30);
  if (bytes != 30) {
    DEBUG_PRINTLN(topic_ + " (VN100.cpp): Not all data read (timeout?).");
    return nullptr;
  }
  if (in_[0] != 0xFA) {
    DEBUG_PRINTLN(topic_ + " (VN100.cpp): Start byte not detected.");
    return nullptr;
  }
  checksum_.b[0] = in_[29];
  checksum_.b[1] = in_[28];
  for (size_t i = 0; i < 4; ++i) {
    a_x_.b[i] = in_[4 + i];
    a_y_.b[i] = in_[8 + i];
    a_z_.b[i] = in_[12 + i];
    W_x_.b[i] = in_[16 + i];
    W_y_.b[i] = in_[20 + i];
    W_z_.b[i] = in_[24 + i];
  }
  static int16_t scaled_sensor_data[7];
  scaled_sensor_data[1] = W_x_.f * imu_gyro_sensitivity_;
  scaled_sensor_data[2] = W_y_.f * imu_gyro_sensitivity_;
  scaled_sensor_data[3] = W_z_.f * imu_gyro_sensitivity_;
  scaled_sensor_data[4] = a_x_.f * imu_accelerator_sensitivity_;
  scaled_sensor_data[5] = a_y_.f * imu_accelerator_sensitivity_;
  scaled_sensor_data[6] = a_z_.f * imu_accelerator_sensitivity_;
  return (scaled_sensor_data); // Return pointer with data
}

////////////////////////////////////////////////////////////////////////////
// Calculates checksum based on binary data.
// Returns the calculated checksum.
//
// From the documentation:
// The CRC consists of a 16-bit CRC of the packet. The CRC is calculated over
// the packet starting just after the sync byte in the header (not including the
// sync byte) and ending at the end of the payload. More information about the
// CRC algorithm and example code for how to perform the calculation is shown in
// the Checksum/CRC section of the Software Architecture chapter. The CRC is
// selected such that if you compute the 16-bit CRC starting with the group byte
// and include the CRC itself, a valid packet will result in 0x0000 computed by
// the running CRC calculation over the entire packet. This provides a simple
// way of detecting packet corruption by simply checking to see if the CRC
// calculation of the entire packet (not including the sync byte) results in
// zero.
////////////////////////////////////////////////////////////////////////////
unsigned short VN100::checksum(byte data[], size_t length,
                               bool ignore_first_byte) {
  unsigned short crc = 0;
  const size_t start_idx = ignore_first_byte ? 1 : 0;
  for (size_t i = start_idx; i < length; ++i) {
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}

////////////////////////////////////////////////////////////////////////////
// Clear Serial buffer for any remaining data.
////////////////////////////////////////////////////////////////////////////
void VN100::clearBuffer() {
  const uint64_t local_tic = micros();
  while (Serial1.available()) {
    if (micros() - local_tic < kImuSyncTimeoutUs / 3) {
      Serial1.readBytes(in_, Serial1.available());
    } else {
      return;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the sensor data without any validity checks (may result in
// spikes).
///////////////////////////////////////////////////////////////////////////////////////////////
bool VN100::updateData() {
  sensor_data_ = readImuData();
  return sensor_data_ != nullptr;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the internally stored sensor data recusivelly by checking
// the validity.
///////////////////////////////////////////////////////////////////////////////////////////////
bool VN100::updateDataIterative() {
  uint64_t tic = micros();
  bool success = false;
  for (size_t depth = 0; depth < kMaxRecursiveUpdateDepth; ++depth) {
    Sensor::setTimestampNow();
    sensor_data_ = readImuData();
    if (sensor_data_ == nullptr || checksum_.s != checksum(in_, 28, true)) {
      if (micros() - tic > kImuSyncTimeoutUs) {
        return false;
      }
      DEBUG_PRINTLN(topic_ +
                    " (VN100.cpp): Failed IMU update detected, trying again " +
                    (String)(kMaxRecursiveUpdateDepth - depth) + " times.");
    } else {
      return true;
    }
  }
  return false;
}
