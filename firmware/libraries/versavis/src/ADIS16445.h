////////////////////////////////////////////////////////////////////////////////
//  May 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
//  Adapted for ADIS16445 by Florian Tschopp <ftschopp@ethz.ch>
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in versavis
////////////////////////////////////////////////////////////////////////////////
//  ADIS16445.h
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides all the functions necessary to interface the ADIS16445
//  IMU with an 8-Bit Atmel-based Arduino development board. Functions for SPI
//  configuration, reads and writes, and scaling are included. This library may
//  be used for the entire ADIS164XX family of devices with some modification.
//
//  This example is free software. You can redistribute it and/or modify it
//  under the terms of the GNU Lesser Public License as published by the Free
//  Software Foundation, either version 3 of the License, or any later version.
//
//  This example is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser Public License for
//  more details.
//
//  You should have received a copy of the GNU Lesser Public License along with
//  this example.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ADIS16445_h
#define ADIS16445_h
#include "Arduino.h"
#include "Imu.h"
#include <SPI.h>
#include <ros.h>

// User Register Memory Map from Table 6
#define FLASH_CNT 0x00  // Flash memory write count
#define XGYRO_OUT 0x04  // X-axis gyroscope output
#define YGYRO_OUT 0x06  // Y-axis gyroscope output
#define ZGYRO_OUT 0x08  // Z-axis gyroscope output
#define XACCL_OUT 0x0A  // X-axis accelerometer output
#define YACCL_OUT 0x0C  // Y-axis accelerometer output
#define ZACCL_OUT 0x0E  // Z-axis accelerometer output
#define TEMP_OUT 0x18   // Temperature output
#define XGYRO_OFF 0x1A  // X-axis gyroscope bias offset factor
#define YGYRO_OFF 0x1C  // Y-axis gyroscope bias offset factor
#define ZGYRO_OFF 0x1E  // Z-axis gyroscope bias offset factor
#define XACCL_OFF 0x20  // X-axis acceleration bias offset factor
#define YACCL_OFF 0x22  // Y-axis acceleration bias offset factor
#define GPIO_CTRL 0x32  // GPIO control
#define MSC_CTRL 0x34   // MISC control
#define SMPL_PRD 0x36   // Sample clock/Decimation filter control
#define SENS_AVG 0x38   // Digital filter control
#define DIAG_STAT 0x3C  // System status
#define GLOB_CMD 0x3E   // System command
#define ALM_MAG1 0x40   // Alarm 1 amplitude threshold
#define ALM_MAG2 0x42   // Alarm 2 amplitude threshold
#define ALM_SMPL1 0x44  // Alarm 1 sample size
#define ALM_SMPL2 0x46  // Alarm 2 sample size
#define ALM_CTRL 0x48   // Alarm control
#define LOT_ID1 0x52    // Lot identification number
#define LOT_ID2 0x54    // Lot identification number
#define PROD_ID 0x56    // Product identifier
#define SERIAL_NUM 0x58 // Lot-specific serial number

class ADIS16445 : public Imu {
public:
  // ADIS16445 Constructor (ChipSelect, DataReady, Reset Pins)
  ADIS16445(ros::NodeHandle *nh, const String &topic, const int rate_hz,
            Timer &timer, int CS, int DR, int RST);

  // Destructor
  ~ADIS16445();

  // Initialize
  void setup();

  // Performs hardware reset. Delay in miliseconds. Returns 1 when complete.
  int resetDUT(uint8_t ms);

  // Sets SPI bit order, clock divider, and data mode. Returns 1 when complete.
  int configSPI();

  // Read register (two bytes) Returns signed 16 bit data.
  int16_t regRead(uint8_t regAddr);

  int16_t *sensorRead();

  int16_t *sensorReadAll();
  // Write register (two bytes). Returns 1 when complete.
  int regWrite(uint8_t regAddr, int16_t regData);

  // Scale accelerometer data. Returns scaled data as float.
  float accelScale(int16_t sensorData);

  // Scale gyro data. Returns scaled data as float.
  float gyroScale(int16_t sensorData);

  // Scale temperature data. Returns scaled data as float.
  float tempScale(int16_t sensorData);

  // Update data internally with validity checks.
  bool updateDataIterative();

  // Update data withoput recursion.
  bool updateData();

private:
  // Variables to store hardware pin assignments.
  int CS_;
  int RST_;
};

#endif
