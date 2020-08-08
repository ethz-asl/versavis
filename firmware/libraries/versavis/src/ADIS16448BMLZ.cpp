////////////////////////////////////////////////////////////////////////////////
//  July 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
//  Updated by Inkyu Sa <enddl22@gmail.com> for ADIS16448BMLZ
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in versavis
////////////////////////////////////////////////////////////////////////////////
//  ADIS16448BMLZ.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides all the functions necessary to interface the
//  ADIS16448BMLZ IMU with an 8-Bit Atmel-based Arduino development board.
//  Functions for SPI configuration, reads and writes, and scaling are included.
//  This library may be used for the entire ADIS164XX family of devices with
//  some modification.
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

#include "ADIS16448BMLZ.h"
#include "helper.h"
#include "versavis_configuration.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16448BMLZ::ADIS16448BMLZ(ros::NodeHandle *nh, const String &topic,
                             const int rate_hz, Timer &timer, int CS, int DR,
                             int RST)
    : Imu(nh, topic, rate_hz, timer), CS_(CS), RST_(RST) {
  if (DR >= 0) {
    pinMode(DR, INPUT); // Set DR pin to be an input
  }

  if (RST_ >= 0) {
    pinMode(RST_, OUTPUT);    // Set RST pin to be an output
    digitalWrite(RST_, HIGH); // Initialize RST pin to be high
  }
}

void ADIS16448BMLZ::setup() {
  DEBUG_PRINTLN((topic_ + " (ADIS16448BMLZ.cpp): Setup.").c_str());
  SPI.begin(); // Initialize SPI bus
  configSPI(); // Configure SPI

  // Reset IMU.
  regWrite(GLOB_CMD, 0xBE80); // Perform an IMU reset.
  delay(300);
  regWrite(GLOB_CMD, 0xBE00); // Take IMU out of reset state.
  delay(100);
  configSPI(); // Configure SPI
  delay(100);

  // Set default pin states
  pinMode(CS_, OUTPUT);    // Set CS pin to be an output
  digitalWrite(CS_, HIGH); // Initialize CS pin to be high

  /* ---------------- Perform IMU configuration  --------------------------- */
  //===========================================================================
  //          MSC_CTRL (16bits Default=0x0006, B0000 0000 0000 0110)
  // Bits    : Description
  // 15-12   : Not used
  // 11      : Checksum mem test(0/1=off/on)
  // 10      : Internal self test, (cleared (low) upon completion)
  // 9-8     : Do not use
  // 7       : Not used
  // 6       : Point of percussion (0/1=off/on)
  // 5       : Not used
  // 4       : CRC-16 code for burst mode (0/1=off/on)
  // 3       : Not used
  // 2       : Data ready enable (0/1=off/on)
  // 1       : Data ready polarity (0/1=active low/high when data is valid_
  // 0       : Data ready line select (0=DIO1, 1=DIO2)
  //===========================================================================

  // Disable data ready. Enable CRC (CRC is only valid for 16448BMLZ not for
  // AMLZ..)
  int16_t kMscCtrlRegister = 0x0010; // B0000 0000 0001 0000

  regWrite(MSC_CTRL, kMscCtrlRegister);
  delay(20);

  //===========================================================================
  //          SENS_AVG (16bits, default= 0x0402, B0000 0100 0000 0010)
  // Bits    : Description
  // 15-11   : Not-used
  // 10-8    : Measurement range selection,
  //          100=±1000°/sec default) 010=±500°/sec 001=±250°/sec
  // 7-3     : Not used
  // 2-0     : Filter Size variable B, Number of taps in each stage; N_B=2^B
  //===========================================================================
  // 0x0100: ±250°/sec, note that the lower dynamic range settings limit the
  // minimum filter tap sizes to maintain resolution.

  int16_t kSensAvgRegister = 0x0404; // B0000 0100 0000 0000;
  regWrite(SENS_AVG, kSensAvgRegister);
  delay(20);

  //===========================================================================
  //          SMPL_PRD (16bits, default=0x0001
  //      15-13    |         12-8           |        7-1    |        0
  //      Not-used |  D, decimation rate    | Not used      | Clock, 0= ext,
  //      1=internal
  //===========================================================================
  // internal clock,
  int16_t kSmplPrdRegister = 0x0001; // B0000 0000 0000 0001;

  regWrite(SMPL_PRD, kSmplPrdRegister);
  delay(20);

  Imu::setupPublisher();
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16448BMLZ::~ADIS16448BMLZ() {
  // Close SPI bus
  SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting RST_ pin low for delay (in ms).
////////////////////////////////////////////////////////////////////////////
int ADIS16448BMLZ::resetDUT(uint8_t ms) {
  if (RST_ == -1) {
    return -1;
  } else {
    digitalWrite(RST_, LOW);
    delay(100);
    digitalWrite(RST_, HIGH);
    delay(ms);
    return (1);
  }
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16448BMLZ::configSPI() {
  SPI.setBitOrder(MSBFIRST);            // Per the datasheet
  SPI.setClockDivider(SPI_CLOCK_DIV16); // Config for 1MHz (ADIS16448BMLZ max
                                        // 2MHz, burst read max 1MHz)
  SPI.setDataMode(SPI_MODE3); // Clock base at one, sampled on falling edge
  return (1);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16448BMLZ::regRead(uint8_t regAddr) {
  // Read registers using SPI

  // Write register address to be read
  digitalWrite(CS_, LOW); // Set CS low to enable device
  SPI.transfer(regAddr);  // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction
                      // requirement
  digitalWrite(CS_, HIGH); // Set CS high to disable device

  delayMicroseconds(25); // Delay to not violate read rate (40us)

  // Read data from requested register
  digitalWrite(CS_, LOW); // Set CS low to enable device
  uint8_t _msbData =
      SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData =
      SPI.transfer(0x00);  // Send (0x00) and place lower byte into variable
  digitalWrite(CS_, HIGH); // Set CS high to disable device

  delayMicroseconds(25); // Delay to not violate read rate (40us)

  int16_t _dataOut =
      (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

  return (_dataOut);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads all gyro, accel, magn, baro, and tmp registers in one instance (faster)
// including
// CRC-16 Checksum.
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (pointer) array of signed 16 bit 2's complement numbers
////////////////////////////////////////////////////////////////////////////////////////////
int16_t *ADIS16448BMLZ::sensorReadAllCRC() {
  // Read registers using SPI
  // Initialize sensor data arrays.
  uint8_t sensorData[26];
  // Write each requested register address and read back it's data
  digitalWrite(CS_, LOW); // Set CS low to enable communication with the device
  SPI.transfer(GLOB_CMD); // Initial SPI read. Returned data for this transfer
                          // is invalid
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction
                      // requirement

  // DIAG_STAT
  sensorData[0] =
      SPI.transfer(0x00); // Write next address to device and read upper byte
  sensorData[1] = SPI.transfer(0x00); // Read lower byte
  // XGYRO_OUT
  sensorData[2] = SPI.transfer(0x00);
  sensorData[3] = SPI.transfer(0x00);
  // YGYRO_OUT
  sensorData[4] = SPI.transfer(0x00);
  sensorData[5] = SPI.transfer(0x00);
  // ZGYRO_OUT
  sensorData[6] = SPI.transfer(0x00);
  sensorData[7] = SPI.transfer(0x00);
  // XACCL_OUT
  sensorData[8] = SPI.transfer(0x00);
  sensorData[9] = SPI.transfer(0x00);
  // YACCL_OUT
  sensorData[10] = SPI.transfer(0x00);
  sensorData[11] = SPI.transfer(0x00);
  // ZACCL_OUT
  sensorData[12] = SPI.transfer(0x00);
  sensorData[13] = SPI.transfer(0x00);
  // XMAGN_OUT
  sensorData[14] = SPI.transfer(0x00);
  sensorData[15] = SPI.transfer(0x00);
  // YMAGN_OUT
  sensorData[16] = SPI.transfer(0x00);
  sensorData[17] = SPI.transfer(0x00);
  // ZMAGN_OUT
  sensorData[18] = SPI.transfer(0x00);
  sensorData[19] = SPI.transfer(0x00);
  // BARO_OUT
  sensorData[20] = SPI.transfer(0x00);
  sensorData[21] = SPI.transfer(0x00);
  // TEMP_OUT
  sensorData[22] = SPI.transfer(0x00);
  sensorData[23] = SPI.transfer(0x00);
  // CRC-16
  sensorData[24] =
      SPI.transfer(0x00); // Final transfer. Data after this invalid
  sensorData[25] = SPI.transfer(0x00);

  digitalWrite(CS_, HIGH); // Disable communication with device.

  // Concatenate two bytes into word
  static int16_t joinedData[13];
  joinedData[0] = (sensorData[0] << 8) | (sensorData[1] & 0xFF);    // DIAG_STAT
  joinedData[1] = (sensorData[2] << 8) | (sensorData[3] & 0xFF);    // XGYRO_OUT
  joinedData[2] = (sensorData[4] << 8) | (sensorData[5] & 0xFF);    // YGYRO_OUT
  joinedData[3] = (sensorData[6] << 8) | (sensorData[7] & 0xFF);    // ZGYRO_OUT
  joinedData[4] = (sensorData[8] << 8) | (sensorData[9] & 0xFF);    // XACCL_OUT
  joinedData[5] = (sensorData[10] << 8) | (sensorData[11] & 0xFF);  // YACCL_OUT
  joinedData[6] = (sensorData[12] << 8) | (sensorData[13] & 0xFF);  // ZACCL_OUT
  joinedData[7] = (sensorData[14] << 8) | (sensorData[15] & 0xFF);  // XMAGN_OUT
  joinedData[8] = (sensorData[16] << 8) | (sensorData[17] & 0xFF);  // YMAGN_OUT
  joinedData[9] = (sensorData[18] << 8) | (sensorData[19] & 0xFF);  // ZMAGN_OUT
  joinedData[10] = (sensorData[20] << 8) | (sensorData[21] & 0xFF); // BARO_OUT
  joinedData[11] = (sensorData[22] << 8) | (sensorData[23] & 0xFF); // TEMP_OUT
  joinedData[12] = (sensorData[24] << 8) | (sensorData[25] & 0xFF); // CRC-16

  return (joinedData); // Return pointer with data
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16448BMLZ::regWrite(uint8_t regAddr, int16_t regData) {
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
  digitalWrite(CS_, LOW);         // Set CS low to enable device
  SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
  SPI.transfer(lowBytehighWord);  // Write low byte from high word to SPI bus
  digitalWrite(CS_, HIGH);        // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (40us)

  // Write lowWord to SPI bus
  digitalWrite(CS_, LOW);        // Set CS low to enable device
  SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
  SPI.transfer(lowBytelowWord);  // Write low byte from low word to SPI bus
  digitalWrite(CS_, HIGH);       // Set CS high to disable device

  return (1);
}

////////////////////////////////////////////////////////////////////////////
// Calculates checksum based on burst data.
// Returns the calculated checksum.
////////////////////////////////////////////////////////////////////////////
// *burstArray - array of burst data
// return - (int16_t) signed calculated checksum
////////////////////////////////////////////////////////////////////////////
int16_t ADIS16448BMLZ::checksum(int16_t *burstArray) {
  unsigned char i;        // Tracks each burstArray word
  unsigned int data;      // Holds the lower/Upper byte for CRC computation
  unsigned int crc;       // Holds the CRC value
  unsigned int lowerByte; // Lower Byte of burstArray word
  unsigned int upperByte; // Upper Byte of burstArray word
  unsigned int POLY;      // Divisor used during CRC computation
  POLY = 0x1021;          // Define divisor
  crc = 0xFFFF;           // Set CRC to ‐1 prior to beginning CRC computation
  // Compute CRC on burst data starting from XGYRO_OUT and ending with TEMP_OUT.
  // Start with the lower byte and then the upper byte of each word.
  // i.e. Compute XGYRO_OUT_LSB CRC first and then compute XGYRO_OUT_MSB CRC.
  for (i = 1; i < 12; i++) {
    upperByte = (burstArray[i] >> 8) & 0xFF;
    lowerByte = (burstArray[i] & 0xFF);
    updateCRC(&crc, &lowerByte, POLY);
    updateCRC(&crc, &upperByte, POLY);
  }
  crc = ~crc; // Compute complement of CRC
  data = crc;
  crc = (crc << 8) |
        (data >> 8 & 0xFF); // Perform byte swap prior to returning CRC
  return crc;
}

void ADIS16448BMLZ::updateCRC(unsigned int *crc, unsigned int *data,
                              const unsigned int &POLY) {
  unsigned char ii; // Counter for each bit of the current burstArray word
  for (ii = 0; ii < 8; ii++, *data >>= 1) {
    if ((*crc & 0x0001) ^ (*data & 0x0001))
      *crc = (*crc >> 1) ^ POLY;
    else
      *crc >>= 1;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function and returns
// acceleration in g's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled accelerometer in g's
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448BMLZ::accelScale(int16_t sensorData) {
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData; // Else return the raw number
  float finalData =
      signedData * 0.000833; // Multiply by accel sensitivity (0.833 mg/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the regRead() function and returns gyro rate
// in deg/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448BMLZ::gyroScale(int16_t sensorData) {
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData =
      signedData * 0.04; // Multiply by gyro sensitivity (0.04 dps/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function and returns
// temperature
// in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448BMLZ::tempScale(int16_t sensorData) {
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData =
      (signedData * 0.07386) +
      31; // Multiply by temperature scale and add 31 to equal 0x0000
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts barometer data output from regRead() function and returns pressure
// in bar
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled pressure in mBar
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448BMLZ::pressureScale(int16_t sensorData) {
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData =
      (signedData * 0.02); // Multiply by barometer sensitivity (0.02 mBar/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts magnetometer output from regRead() function and returns magnetic
// field
// reading in Gauss
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled magnetometer data in mgauss
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448BMLZ::magnetometerScale(int16_t sensorData) {
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData =
      (signedData * 0.0001429); // Multiply by sensor resolution (142.9 uGa/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the sensor data without any validity checks (may result in
// spikes).
///////////////////////////////////////////////////////////////////////////////////////////////
bool ADIS16448BMLZ::updateData() {
  sensor_data_ = sensorReadAllCRC();
  return sensor_data_ != nullptr;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the internally stored sensor data recusivelly by checking
// the validity.
///////////////////////////////////////////////////////////////////////////////////////////////
bool ADIS16448BMLZ::updateDataIterative() {
  uint64_t tic = micros();
  bool success = false;
  for (size_t depth = 0; depth < kMaxRecursiveUpdateDepth; ++depth) {
    Sensor::setTimestampNow();
    sensor_data_ = sensorReadAllCRC();
    if (sensor_data_ == nullptr || sensor_data_[12] != checksum(sensor_data_)) {
      if (micros() - tic > kImuSyncTimeoutUs) {
        return false;
      }
      DEBUG_PRINTLN(
          topic_ +
          " (ADIS16448BMLZ.cpp): Failed IMU update detected, trying again " +
          (String)(kMaxRecursiveUpdateDepth - depth) + " times.");
    } else {
      return true;
    }
  }
  return false;
}
