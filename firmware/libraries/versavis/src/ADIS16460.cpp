////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
//  Updated by Andreas Pfrunder <andreas.pfrunder@sevensense.ch> for ADIS16460
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in versavis
////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.cpp
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

#include "ADIS16460.h"
#include "helper.h"
#include "versavis_configuration.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, and RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16460::ADIS16460(ros::NodeHandle *nh, const String &topic,
                     const int rate_hz, Timer &timer, int CS, int DR, int RST)
    : Imu(nh, topic, rate_hz, timer), CS_(CS), RST_(RST) {
  if (DR >= 0) {
    pinMode(DR, INPUT); // Set DR pin to be an input
  }

  if (RST_ >= 0) {
    pinMode(RST_, OUTPUT);    // Set RST pin to be an output
    digitalWrite(RST_, HIGH); // Initialize RST pin to be high
  }
}

void ADIS16460::setup() {
  DEBUG_PRINTLN((topic_ + " (ADIS16460.cpp): Setup."));

  SPI.begin(); // Initialize SPI bus
  configSPI(); // Configure SPI

  // Reset IMU.
  regWrite(GLOB_CMD, 0xBE80); // Perform an IMU reset.
  delay(300);
  regWrite(GLOB_CMD, 0xBE00); // Take IMU out of reset state.
  delay(100);
  configSPI(); // Configure SPI controller
  delay(100);

  // Set default pin states
  pinMode(CS_, OUTPUT);     // Set CS pin to be an output
  pinMode(RST_, OUTPUT);    // Set RST pin to be an output
  digitalWrite(CS_, HIGH);  // Initialize CS pin to be high
  digitalWrite(RST_, HIGH); // Initialize RST pin to be high

  /* ---------------- Perform IMU configuration  --------------------------- */
  regWrite(MSC_CTRL, 0x00C1); // Enable Data Ready on IMU.
  delay(20);
  regWrite(FLTR_CTRL, 0x0500); // Set Digital Filter on IMU.
  delay(20);
  regWrite(DEC_RATE, 0x0000); // Set Decimation on IMU.
  delay(20);

  Imu::setupPublisher();
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16460::~ADIS16460() {
  // Close SPI bus
  SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting RST_ pin low for delay (in ms).
////////////////////////////////////////////////////////////////////////////
int ADIS16460::resetDUT(uint8_t ms) {
  digitalWrite(RST_, LOW);
  delay(100);
  digitalWrite(RST_, HIGH);
  delay(ms);
  return (1);
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16460::configSPI() {
  SPISettings IMUSettings(1000000, MSBFIRST, SPI_MODE3);
  SPI.beginTransaction(IMUSettings);
  return (1);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16460::regRead(uint8_t regAddr) {
  // Read registers using SPI

  // Write register address to be read
  digitalWrite(CS_, LOW); // Set CS low to enable device
  SPI.transfer(regAddr);  // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction
                      // requirement
  digitalWrite(CS_, HIGH); // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (16 us)

  // Read data from requested register
  digitalWrite(CS_, LOW); // Set CS low to enable device
  uint8_t _msbData =
      SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData =
      SPI.transfer(0x00);  // Send (0x00) and place lower byte into variable
  digitalWrite(CS_, HIGH); // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (16 us)

  int16_t _dataOut =
      (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

  return (_dataOut);
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16460::regWrite(uint8_t regAddr, int16_t regData) {

  // Write register address and data
  uint16_t addr =
      (((regAddr & 0x7F) | 0x80)
       << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord =
      (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord =
      ((addr | 0x100) |
       ((regData >> 8) &
        0xFF)); // OR Register address with data and increment address

  // Split words into chars
  uint8_t highBytehighWord = (highWord >> 8);
  uint8_t lowBytehighWord = (highWord & 0xFF);
  uint8_t highBytelowWord = (lowWord >> 8);
  uint8_t lowBytelowWord = (lowWord & 0xFF);

  // Write highWord to SPI bus
  digitalWrite(CS_, LOW);        // Set CS low to enable device
  SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
  SPI.transfer(lowBytelowWord);  // Write low byte from low word to SPI bus
  digitalWrite(CS_, HIGH);       // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (16 us)

  // Write lowWord to SPI bus
  digitalWrite(CS_, LOW);         // Set CS low to enable device
  SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
  SPI.transfer(lowBytehighWord);  // Write low byte from high word to SPI bus
  digitalWrite(CS_, HIGH);        // Set CS high to disable device

  return (1);
}

////////////////////////////////////////////////////////////////////////////
// Intiates a burst read from the sensor.
// Returns a pointer to an array of sensor data.
////////////////////////////////////////////////////////////////////////////
// No inputs required.
////////////////////////////////////////////////////////////////////////////
int16_t *ADIS16460::burstRead(void) {
  uint8_t burstdata[20];
  static int16_t burstwords[10];
  // Trigger Burst Read
  digitalWrite(CS_, LOW);
  SPI.transfer(0x3E);
  SPI.transfer(0x00);
  // Read Burst Data
  burstdata[0] = SPI.transfer(0x00); // DIAG_STAT
  burstdata[1] = SPI.transfer(0x00);
  burstdata[2] = SPI.transfer(0x00); // XGYRO_OUT
  burstdata[3] = SPI.transfer(0x00);
  burstdata[4] = SPI.transfer(0x00); // YGYRO_OUT
  burstdata[5] = SPI.transfer(0x00);
  burstdata[6] = SPI.transfer(0x00); // ZGYRO_OUT
  burstdata[7] = SPI.transfer(0x00);
  burstdata[8] = SPI.transfer(0x00); // XACCEL_OUT
  burstdata[9] = SPI.transfer(0x00);
  burstdata[10] = SPI.transfer(0x00); // YACCEL_OUT
  burstdata[11] = SPI.transfer(0x00);
  burstdata[12] = SPI.transfer(0x00); // ZACCEL_OUT
  burstdata[13] = SPI.transfer(0x00);
  burstdata[14] = SPI.transfer(0x00); // TEMP_OUT
  burstdata[15] = SPI.transfer(0x00);
  burstdata[16] = SPI.transfer(0x00); // SMPL_CNTR
  burstdata[17] = SPI.transfer(0x00);
  burstdata[18] = SPI.transfer(0x00); // CHECKSUM
  burstdata[19] = SPI.transfer(0x00);
  digitalWrite(CS_, HIGH);

  // Join bytes into words
  burstwords[0] = ((burstdata[0] << 8) | (burstdata[1] & 0xFF));   // DIAG_STAT
  burstwords[1] = ((burstdata[2] << 8) | (burstdata[3] & 0xFF));   // XGYRO
  burstwords[2] = ((burstdata[4] << 8) | (burstdata[5] & 0xFF));   // YGYRO
  burstwords[3] = ((burstdata[6] << 8) | (burstdata[7] & 0xFF));   // ZGYRO
  burstwords[4] = ((burstdata[8] << 8) | (burstdata[9] & 0xFF));   // XACCEL
  burstwords[5] = ((burstdata[10] << 8) | (burstdata[11] & 0xFF)); // YACCEL
  burstwords[6] = ((burstdata[12] << 8) | (burstdata[13] & 0xFF)); // ZACCEL
  burstwords[7] = ((burstdata[14] << 8) | (burstdata[15] & 0xFF)); // TEMP_OUT
  burstwords[8] = ((burstdata[16] << 8) | (burstdata[17] & 0xFF)); // SMPL_CNTR
  burstwords[9] = ((burstdata[18] << 8) | (burstdata[19] & 0xFF)); // CHECKSUM

  return burstwords;
}

////////////////////////////////////////////////////////////////////////////
// Calculates checksum based on burst data.
// Returns the calculated checksum.
////////////////////////////////////////////////////////////////////////////
// *burstArray - array of burst data
// return - (int16_t) signed calculated checksum
////////////////////////////////////////////////////////////////////////////
int16_t ADIS16460::checksum(int16_t *burstArray) {
  int16_t s = 0;
  for (int i = 0; i < 9; i++) // Checksum value is not part of the sum!!
  {
    s += (burstArray[i] & 0xFF);        // Count lower byte
    s += ((burstArray[i] >> 8) & 0xFF); // Count upper byte
  }

  return s;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function and returns
// acceleration in mg's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled accelerometer in g's
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::accelScale(int16_t sensorData) {
  float finalData =
      sensorData * 0.00025 * 9.81; // Multiply by accel sensitivity (25 mg/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the regRead() function and returns gyro rate
// in deg/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::gyroScale(int16_t sensorData) {
  float finalData = sensorData * 0.005 * M_PI / 180;
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function and returns
// temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::tempScale(int16_t sensorData) {
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData =
      (signedData * 0.05) +
      25; // Multiply by temperature scale and add 25 to equal 0x0000
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated angle data output from the regRead() function and returns
// delta angle in degrees
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled delta angle in degrees
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::deltaAngleScale(int16_t sensorData) {
  float finalData =
      sensorData * 0.005; // Multiply by delta angle scale (0.005 degrees/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated velocity data output from the regRead() function and
// returns delta velocity in mm/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled delta velocity in mm/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::deltaVelocityScale(int16_t sensorData) {
  float finalData =
      sensorData * 2.5; // Multiply by velocity scale (2.5 mm/sec/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the sensor data without any validity checks (may result in
// spikes).
///////////////////////////////////////////////////////////////////////////////////////////////
bool ADIS16460::updateData() {
  sensor_data_ = burstRead();
  return sensor_data_ != nullptr;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the internally stored sensor data recusivelly by checking
// the validity.
///////////////////////////////////////////////////////////////////////////////////////////////
bool ADIS16460::updateDataIterative() {
  uint64_t tic = micros();
  bool success = false;
  for (size_t depth = 0; depth < kMaxRecursiveUpdateDepth; ++depth) {
    Sensor::setTimestampNow();
    sensor_data_ = burstRead();
    if (sensor_data_ == nullptr || sensor_data_[9] != checksum(sensor_data_)) {
      if (micros() - tic > kImuSyncTimeoutUs) {
        return false;
      }
      DEBUG_PRINTLN(
          topic_ +
          " (ADIS16460.cpp): Failed IMU update detected, trying again " +
          (String)(kMaxRecursiveUpdateDepth - depth) + " times.");
    } else {
      return true;
    }
  }
  return false;
}
