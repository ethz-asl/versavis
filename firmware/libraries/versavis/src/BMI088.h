#pragma once

////////////////////////////////////////////////////////////////////////////////
//  September 2020
//  Author: Pascal Auf der Maur <pascalau@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  BMI088.h
////////////////////////////////////////////////////////////////////////////////
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions, the list of authors and contributors and the following
//   disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions, the list of authors and contributors and the following
//   disclaimer in the documentation and/or other materials provided with the
//   distribution.
// * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
//   its contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
////////////////////////////////////////////////////////////////////////////////

#define BMI088_h
#include "Arduino.h"
#include "Imu.h"
#include <Wire.h>
#include <ros.h>

#define ACC_CHIP_ID 0x00
#define ACC_ERR_REG 0x02
#define ACC_STATUS 0x03
#define ACC_STATUS 0x03
#define ACC_X_LSB 0x12
#define ACC_X_MSB 0x13
#define ACC_Y_LSB 0x14
#define ACC_Y_MSB 0x15
#define ACC_Z_LSB 0x16
#define ACC_Z_MSB 0x17
#define ACC_SENSORTIME_0 0x18
#define ACC_SENSORTIME_1 0x19
#define ACC_SENSORTIME_2 0x1A
#define ACC_INT_STAT_1 0x1D
#define ACC_TEMP_MSB 0x22
#define ACC_TEMP_LSB 0x23
#define ACC_FIFO_LENGTH_0 0x24
#define ACC_FIFO_LENGTH_1 0x25
#define ACC_FIFO_DATA 0x26
#define ACC_CONF 0x40
#define ACC_RANGE 0x41
#define ACC_FIFO_DOWNS 0x45
#define ACC_FIFO_WTM_0 0x46
#define ACC_FIFO_WTM_1 0x47
#define ACC_FIFO_WTM_1 0x47
#define ACC_FIFO_CONFIG_0 0x48
#define ACC_FIFO_CONFIG_1 0x49
#define ACC_INT1_IO_CTRL 0x53
#define ACC_INT2_IO_CTRL 0x54
#define ACC_INT_MAP_DATA 0x58
#define ACC_SELF_TEST 0x6D
#define ACC_PWR_CONF 0x7C
#define ACC_PWR_CTRL 0x7D
#define ACC_SOFTRESET 0x7E

#define GYRO_CHIP_ID 0x00
#define RATE_X_LSB 0x02
#define RATE_X_MSB 0x03
#define RATE_Y_LSB 0x04
#define RATE_Y_MSB 0x05
#define RATE_Z_LSB 0x06
#define RATE_Z_MSB 0x07
#define GYRO_INT_STAT_1 0x0A
#define GYRO_FIFO_STATUS 0x0E
#define GYRO_RANGE 0x0F
#define GYRO_BANDWIDTH 0x10
#define GYRO_LPM1 0x11
#define GYRO_SOFTRESET 0x14
#define GYRO_INT_CTRL 0x15
#define GYRO_INT3_INT4_IO_CONF 0x16
#define GYRO_INT3_INT4_IO_MAP 0x18
#define GYRO_FIFO_WM_EN 0x1E
#define GYRO_FIFO_EXT_INT_S 0x34
#define GYRO_SELF_TEST 0x3C
#define GYRO_FIFO_CONFIG_0 0x3D
#define GYRO_FIFO_CONFIG_1 0x3E
#define GYRO_FIFO_DATA 0x3F


class BMI088 : public Imu {

private:
  int acc_addr_;
  int gyr_addr_;

public:
  // Constructor with configurable CS, data ready, and HW reset pins
  BMI088(ros::NodeHandle *nh, const String &topic, const int rate_hz,
            Timer &timer, int acc_addr, int gyr_addr);

  // Destructor
  ~BMI088();

  // Configure IMU
  void setup();

  // Performs softreset
  int resetDUT(uint8_t ms);

  //Configures I2C
  int configI2C();

  // Read two consecutive registers on the sensor
  int16_t regRead(uint8_t devAddr, uint8_t regAddr);

  // Write to register
  int regWrite(uint8_t devAddr, uint8_t regAddr, int8_t regData);

  // Read sensor data using a burst read
  int16_t *burstRead(void);

  // Scale accelerator data
  float accelScale(int16_t sensorData);

  // Scale gyro data
  float gyroScale(int16_t sensorData);

  // Scale temperature data
  float tempScale(int16_t sensorData);

  // Update data internally with recursion.
  // Since there are no checksum registers this
  // function behaves identical as updateData()
  bool updateDataIterative();

  // Update data without recursion.
  bool updateData();
};
