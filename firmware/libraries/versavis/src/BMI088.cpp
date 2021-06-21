////////////////////////////////////////////////////////////////////////////////
//  September 2020
//  Author: Pascal Auf der Maur <pascalau@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  BMI088.cpp
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

#include "BMI088.h"
#include "helper.h"
#include "versavis_configuration.h"

////////////////////////////////////////////////////////////////////////////
// Constructor for usage on I2C bus.
////////////////////////////////////////////////////////////////////////////
// acc_addr - Address of accelerometer on bus.
// gyr_addr - Address of gyroscope on bus.
////////////////////////////////////////////////////////////////////////////
BMI088::BMI088(ros::NodeHandle *nh, const String &topic, const int rate_hz,
              Timer &timer, int acc_addr, int gyr_addr): Imu(nh, topic, rate_hz, timer) {
    acc_addr_ = acc_addr;
    gyr_addr_ = gyr_addr;
}


////////////////////////////////////////////////////////////////////////////
// Setup function
////////////////////////////////////////////////////////////////////////////
void BMI088::setup(){
    DEBUG_PRINTLN((topic_ + " (BMI088.cpp): Setup."));

    Wire.begin();

    configI2C();

    regWrite(acc_addr_, ACC_SOFTRESET, 0xB6);
    delay(300);
    regWrite(gyr_addr_, GYRO_SOFTRESET, 0xB6);
    delay(300);

    // Set powermode of IMU
    regWrite(acc_addr_, ACC_PWR_CTRL, 0x04);
    delay(5);
    regWrite(acc_addr_, ACC_PWR_CONF, 0x00);
    delay(5);

    /* Configures the bandwidth of the internal data filter of the 
     * accelerometer.
     * More details on page 29 of the datasheet. */
    regWrite(acc_addr_, ACC_CONF, 0xAB);
    delay(20);

    /* Configures the range of the accelerometer
     *
     * Reg entry | Range | imu_accelerator_sensitivity
     * ----------+-------+------------------------------
     *    0x00   | +-3g  | 0.0000916
     * ----------+-------+------------------------------
     *    0x01   | +-6g  | 0.000183
     * ----------+-------+------------------------------
     *    0x02   | +-12g | 0.000366
     * ----------+-------+------------------------------
     *    0x03   | +-24g | 0.000732
     */
    regWrite(acc_addr_, ACC_RANGE, 0x08);
    delay(20);

    /* Configures the bandwidth of the internal data filter of the 
     * gyroscope.
     * Reg entry |  BW   
     * ----------+-------
     *    0x80   | 532Hz
     * ----------+-------
     *    0x81   | 230Hz
     * ----------+-------
     *    0x82   | 116Hz
     * ----------+-------
     *    0x83   |  47Hz
     * ----------+-------
     *    0x84   |  23Hz
     * ----------+-------
     *    0x85   |  12Hz
     * ----------+-------
     *    0x86   |  64Hz
     * ----------+-------
     *    0x87   |  32Hz
     * More infos on page 39 of the datasheet. */
    regWrite(gyr_addr_, GYRO_BANDWIDTH, 0x81);
    delay(20);

    /* Configures the range of the gyroscope
     *
     * Reg entry |    Range   | imu_gyro_sensitivity
     * ----------+------------+-------------------------
     *    0x00   | +-2000°/s  | 0.061
     * ----------+------------+-------------------------
     *    0x01   | +-1000°/s  | 0.0305
     * ----------+------------+-------------------------
     *    0x02   | +-500°/s   | 0.0153
     * ----------+------------+-------------------------
     *    0x03   | +-250°/s   | 0.0076
     * ----------+------------+-------------------------
     *    0x04   | +-125°/s   | 0.0038
     */
    regWrite(gyr_addr_, GYRO_RANGE, 0x02);
    delay(20);

    Imu::setupPublisher();

}


////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
BMI088::~BMI088(){
    Wire.end();

}

////////////////////////////////////////////////////////////////////////////
// Softresets the IMU. 
////////////////////////////////////////////////////////////////////////////
int BMI088::resetDUT(uint8_t ms){
    regWrite(acc_addr_, ACC_SOFTRESET, 0xB6);
    delay(300);
    regWrite(gyr_addr_, GYRO_SOFTRESET, 0xB6);
    delay(300);
}

////////////////////////////////////////////////////////////////////////////
// Configures the I2C Bus to run at fastest possible speed for IMU.
////////////////////////////////////////////////////////////////////////////
int BMI088::configI2C(){
    Wire.setClock(400000);
    return 1;
}

////////////////////////////////////////////////////////////////////////////
// Reads out two consecutive registers and returns their values.
////////////////////////////////////////////////////////////////////////////
// devAddr - Device address on bus.
// regAddr - Register address on device.
////////////////////////////////////////////////////////////////////////////
int16_t BMI088::regRead(uint8_t devAddr, uint8_t regAddr){
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.endTransmission();
    Wire.requestFrom(devAddr, 1);

    uint8_t dataOut;
    
    while(Wire.available()){
        dataOut = Wire.read();
    }
    return dataOut;
}

////////////////////////////////////////////////////////////////////////////
// Writes the provided data to the designated register.
////////////////////////////////////////////////////////////////////////////
// devAddr - Device address on bus.
// regAddr - Register address on device.
// regData - Data to be written to the register.
////////////////////////////////////////////////////////////////////////////
int BMI088::regWrite(uint8_t devAddr, uint8_t regAddr, int8_t regData){
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.write(regData);
    Wire.endTransmission();
    delayMicroseconds(5);
}

////////////////////////////////////////////////////////////////////////////
// Performs a burst read from both sensors and returns
// a pointer to the data.
////////////////////////////////////////////////////////////////////////////
int16_t *BMI088::burstRead(void){
    uint8_t burstdata[12];
    static int16_t burstwords[7];

    Wire.beginTransmission(gyr_addr_);
    Wire.write(RATE_X_LSB);
    Wire.endTransmission();

    Wire.requestFrom(gyr_addr_, 6);
    burstdata[0] = Wire.read(); //RATE_X_LSB
    burstdata[1] = Wire.read(); //RATE_X_MSB
    burstdata[2] = Wire.read(); //RATE_Y_LSB
    burstdata[3] = Wire.read(); //RATE_Y_MSB
    burstdata[4] = Wire.read(); //RATE_Z_LSB
    burstdata[5] = Wire.read(); //RATE_Z_MSB
    
    Wire.beginTransmission(acc_addr_);
    Wire.write(ACC_X_LSB);
    Wire.endTransmission();

    Wire.requestFrom(acc_addr_, 6);
    burstdata[6] = Wire.read(); //ACC_X_LSB
    burstdata[7] = Wire.read(); //ACC_X_MSB
    burstdata[8] = Wire.read(); //ACC_Y_LSB
    burstdata[9] = Wire.read(); //ACC_Y_MSB
    burstdata[10] = Wire.read(); //ACC_Z_LSB
    burstdata[11] = Wire.read(); //ACC_Z_MSB

    
    burstwords[1] = (burstdata[1] << 8) | (burstdata[0]);
    burstwords[2] = (burstdata[3] << 8) | (burstdata[2]);
    burstwords[3] = (burstdata[5] << 8) | (burstdata[4]);
    burstwords[4] = (burstdata[7] << 8) | (burstdata[6]);
    burstwords[5] = (burstdata[9] << 8) | (burstdata[8]);
    burstwords[6] = (burstdata[11] << 8) | (burstdata[10]);

    burstwords[1] = -(burstwords[1] & 32768) + (burstwords[1] & ~32768);
    burstwords[2] = -(burstwords[2] & 32768) + (burstwords[2] & ~32768);
    burstwords[3] = -(burstwords[3] & 32768) + (burstwords[3] & ~32768);
    burstwords[4] = -(burstwords[4] & 32768) + (burstwords[4] & ~32768);
    burstwords[5] = -(burstwords[5] & 32768) + (burstwords[5] & ~32768);
    burstwords[6] = -(burstwords[6] & 32768) + (burstwords[6] & ~32768);

    return burstwords;
}

////////////////////////////////////////////////////////////////////////////
// Scales the read out 12 bit measurement of the accelerometer to m/s^2
////////////////////////////////////////////////////////////////////////////
float BMI088::accelScale(int16_t sensorData){
    return (sensorData/ 2048.0) * 8.0 * 9.807;
}

////////////////////////////////////////////////////////////////////////////
// Scales the read out 16 bit measurement of the gyroscope to rad/s
////////////////////////////////////////////////////////////////////////////
float BMI088::gyroScale(int16_t sensorData){
    return (sensorData/ 32768.0) * 500.0 * M_PI / 180.0;
}

////////////////////////////////////////////////////////////////////////////
// Scales the read out 8 bit measurement of the temperature to °C
////////////////////////////////////////////////////////////////////////////
float BMI088::tempScale(int16_t sensorData){
    float finalData = sensorData * 0.5 + 23.0;

    return finalData;
}

////////////////////////////////////////////////////////////////////////////
// Updates the sensor data without any validity checks.
////////////////////////////////////////////////////////////////////////////
bool BMI088::updateData(){
    sensor_data_ = burstRead();
    return sensor_data_ != nullptr;
}

////////////////////////////////////////////////////////////////////////////
// Updates the sensor data without any validity checks.
////////////////////////////////////////////////////////////////////////////
bool BMI088::updateDataIterative(){
    sensor_data_ = burstRead();
    return sensor_data_ != nullptr;
}
