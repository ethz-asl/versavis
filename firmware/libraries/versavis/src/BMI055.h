#pragma once

////////////////////////////////////////////////////////////////////////////////
//  September 2020
//  Author: Pascal Auf der Maur <pascalau@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  BMI055.h
////////////////////////////////////////////////////////////////////////////////
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#define BMI055_h
#include "Arduino.h"
#include "Imu.h"
#include <Wire.h>
#include <ros.h>

// User Register Memory Map from Table 6
#define ACC_BGW_CHIPID 0x00
#define ACC_ACCD_X_LSB 0x02
#define ACC_ACCD_X_MSB 0x03
#define ACC_ACCD_Y_LSB 0x04
#define ACC_ACCD_Y_MSB 0x05
#define ACC_ACCD_Z_LSB 0x06
#define ACC_ACCD_Z_MSB 0x07
#define ACC_ACCD_TEMP 0x08
#define ACC_INT_STATUS_0 0x09
#define ACC_INT_STATUS_1 0x0A
#define ACC_INT_STATUS_2 0x0B
#define ACC_INT_STATUS_3 0x0C
#define ACC_FIFO_STATUS 0x0E
#define ACC_PMU_RANGE 0x0F
#define ACC_PMU_BW 0x10
#define ACC_PMU_LPW 0x11
#define ACC_PMU_LOW_POWER 0x12
#define ACC_HBW 0x13
#define ACC_ACCD_HBW 0x13
#define ACC_BGW_SOFTRESET 0x14
#define ACC_INT_EN_0 0x16
#define ACC_INT_EN_1 0x17
#define ACC_INT_EN_2 0x18
#define ACC_INT_MAP_0 0x19
#define ACC_INT_MAP_1 0x1A
#define ACC_INT_MAP_2 0x1B
#define ACC_INT_SRC 0x1E
#define ACC_INT_OUT_CTRL 0x20
#define ACC_INT_RST_LATCH 0x21
#define ACC_INT_0 0x22
#define ACC_INT_1 0x23
#define ACC_INT_2 0x24
#define ACC_INT_3 0x25
#define ACC_INT_4 0x26
#define ACC_INT_5 0x27
#define ACC_INT_6 0x28
#define ACC_INT_7 0x29
#define ACC_INT_8 0x2A
#define ACC_INT_9 0x2B
#define ACC_INT_A 0x2C
#define ACC_INT_B 0x2D
#define ACC_INT_C 0x2E
#define ACC_INT_D 0x2F
#define ACC_FIFO_CONFIG_0 0x30
#define ACC_PMU_SELF_TEST 0x32
#define ACC_PMU_TRIM_NVM_CTRL 0x33
#define ACC_BGW_SPI3_WDT 0x34
#define ACC_OFC_CTRL 0x36
#define ACC_OFC_SETTING 0x37
#define ACC_OFC_OFFSET_X 0x38
#define ACC_OFC_OFFSET_Y 0x39
#define ACC_OFC_OFFSET_Z 0x3A
#define ACC_TRIM_GP0 0x3B
#define ACC_TRIM_GP1 0x3C
#define ACC_FIFO_CONFIG_1 0x3E
#define ACC_FIFO_DATA 0x3F

#define GYR_CHIP_ID 0x00
#define GYR_RATE_X_LSB 0x02
#define GYR_RATE_X_MSB 0x03
#define GYR_RATE_Y_LSB 0x04
#define GYR_RATE_Y_MSB 0x05
#define GYR_RATE_Z_LSB 0x06
#define GYR_RATE_Z_MSB 0x07
#define GYR_INT_STATUS_0 0x09
#define GYR_INT_STATUS_1 0x0A
#define GYR_INT_STATUS_2 0x0B
#define GYR_INT_STATUS_3 0x0C
#define GYR_FIFO_STATUS 0x0E
#define GYR_RANGE 0x0F
#define GYR_BW 0x10
#define GYR_LPM1 0x11
#define GYR_LPM2 0x12
#define GYR_RATE_HBW 0x13
#define GYR_BGW_SOFTRESET 0x14
#define GYR_INT_EN_0 0x15
#define GYR_INT_EN_1 0x16
#define GYR_INT_MAP_0 0x17
#define GYR_INT_MAP_1 0x18
#define GYR_INT_MAP_2 0x19
#define GYR_INT_0 0x1A
#define GYR_INT_1 0x1B
#define GYR_INT_2 0x1C
#define GYR_INT_4 0x1E
#define GYR_INT_RST_LATCH 0x21
#define GYR_HIGH_TH_X 0x22
#define GYR_HIGH_DUR_X 0x23
#define GYR_HIGH_TH_Y 0x24
#define GYR_HIGH_DUR_Y 0x25
#define GYR_HIGH_TH_Z 0x26
#define GYR_HIGH_DUR_Z 0x27
#define GYR_SOC 0x31
#define GYR_A_FOC 0x32
#define GYR_TRIM_NVM_CTRL 0x33
#define GYR_BGW_SPI3_WDT 0x34
#define GYR_OFC1 0x36
#define GYR_OFC2 0x37
#define GYR_OFC3 0x38
#define GYR_OFC4 0x39
#define GYR_TRIM_GP0 0x3A
#define GYR_TRIM_GP1 0x3B
#define GYR_BIST 0x3C
#define GYR_FIFO_CONFIG_0 0x3D
#define GYR_FIFO_CONFIG_1 0x3E
#define GYR_FIFO_DATA 0x3F


class BMI055 : public Imu {

private:
  int acc_addr_;
  int gyr_addr_;

public:
  // Constructor with configurable CS, data ready, and HW reset pins
  BMI055(ros::NodeHandle *nh, const String &topic, const int rate_hz,
            Timer &timer, int acc_addr, int gyr_addr);

  // Destructor
  ~BMI055();

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
