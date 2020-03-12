////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in versavis
////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.h
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides all the functions necessary to interface the ADIS16460
//  IMU with a PJRC 32-Bit Teensy 3.2 Development Board. Functions for SPI
//  configuration, reads and writes, and scaling are included. This library may
//  be used for the entire ADIS1646X family of devices with some modification.
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

#define ADIS16460_h
#include "Arduino.h"
#include "Imu.h"
#include <SPI.h>
#include <ros.h>

// User Register Memory Map from Table 6
#define FLASH_CNT 0x00  // Flash memory write count
#define DIAG_STAT 0x02  // Diagnostic and operational status
#define X_GYRO_LOW 0x04 // X-axis gyroscope output, lower word
#define X_GYRO_OUT 0x06 // X-axis gyroscope output, upper word
#define Y_GYRO_LOW 0x08 // Y-axis gyroscope output, lower word
#define Y_GYRO_OUT 0x0A // Y-axis gyroscope output, upper word
#define Z_GYRO_LOW 0x0C // Z-axis gyroscope output, lower word
#define Z_GYRO_OUT 0x0E // Z-axis gyroscope output, upper word
#define X_ACCL_LOW 0x10 // X-axis accelerometer output, lower word
#define X_ACCL_OUT 0x12 // X-axis accelerometer output, upper word
#define Y_ACCL_LOW 0x14 // Y-axis accelerometer output, lower word
#define Y_ACCL_OUT 0x16 // Y-axis accelerometer output, upper word
#define Z_ACCL_LOW 0x18 // Z-axis accelerometer output, lower word
#define Z_ACCL_OUT 0x1A // Z-axis accelerometer output, upper word
#define SMPL_CNTR 0x1C  // Sample Counter, MSC_CTRL[3:2]=11
#define TEMP_OUT 0x1E   // Temperature output (internal, not calibrated)
#define X_DELT_ANG 0x24 // X-axis delta angle output
#define Y_DELT_ANG 0x26 // Y-axis delta angle output
#define Z_DELT_ANG 0x28 // Z-axis delta angle output
#define X_DELT_VEL 0x2A // X-axis delta velocity output
#define Y_DELT_VEL 0x2C // Y-axis delta velocity output
#define Z_DELT_VEL 0x2E // Z-axis delta velocity output
#define MSC_CTRL 0x32   // Miscellaneous control
#define SYNC_SCAL 0x34  // Sync input scale control
#define DEC_RATE 0x36   // Decimation rate control
#define FLTR_CTRL 0x38  // Filter control, auto-null record time
#define GLOB_CMD 0x3E   // Global commands
#define XGYRO_OFF 0x40  // X-axis gyroscope bias offset error
#define YGYRO_OFF 0x42  // Y-axis gyroscope bias offset error
#define ZGYRO_OFF 0x44  // Z-axis gyroscope bias offset factor
#define XACCL_OFF 0x46  // X-axis acceleration bias offset factor
#define YACCL_OFF 0x48  // Y-axis acceleration bias offset factor
#define ZACCL_OFF 0x4A  // Z-axis acceleration bias offset factor
#define LOT_ID1 0x52    // Lot identification number
#define LOT_ID2 0x54    // Lot identification number
#define PROD_ID 0x56    // Product identifier
#define SERIAL_NUM 0x58 // Lot-specific serial number
#define CAL_SGNTR 0x60  // Calibration memory signature value
#define CAL_CRC 0x62    // Calibration memory CRC values
#define CODE_SGNTR 0x64 // Code memory signature value
#define CODE_CRC 0x66   // Code memory CRC values

// ADIS16460 class definition
class ADIS16460 : public Imu {

public:
  // Constructor with configurable CS, data ready, and HW reset pins
  ADIS16460(ros::NodeHandle *nh, const String &topic, const int rate_hz,
            Timer &timer, int CS, int DR, int RST);

  // Destructor
  ~ADIS16460();

  // Configure IMU
  void setup();

  // Performs hardware reset by sending pin 8 low on the DUT for 2 seconds
  int resetDUT(uint8_t ms);

  // Sets SPI bit order, clock divider, and data mode
  int configSPI();

  // Read single register from sensor
  int16_t regRead(uint8_t regAddr);

  // Write register
  int regWrite(uint8_t regAddr, int16_t regData);

  // Read sensor data using a burst read
  int16_t *burstRead(void);

  // Verify checksum
  int16_t checksum(int16_t *burstArray);

  // Scale accelerator data
  float accelScale(int16_t sensorData);

  // Scale gyro data
  float gyroScale(int16_t sensorData);

  // Scale temperature data
  float tempScale(int16_t sensorData);

  // Scale delta angle data
  float deltaAngleScale(int16_t sensorData);

  // Scale delta velocity
  float deltaVelocityScale(int16_t sensorData);

  // Update data internally with recursion.
  bool updateDataIterative();

  // Update data withoput recursion.
  bool updateData();

private:
  // Variables to store hardware pin assignments
  int CS_;
  int RST_;
};
