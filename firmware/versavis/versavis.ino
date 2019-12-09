// Import all settings for the chosen sensor configuration.
#include "versavis_configuration.h"

#include <math.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

#include "Arduino.h"

#ifdef USE_ADIS16445
#include <ADIS16445.h>
#elif defined(USE_ADIS16448AMLZ)
#include <ADIS16448AMLZ.h>
#elif defined(USE_ADIS16448BMLZ)
#include <ADIS16448BMLZ.h>
#elif defined(USE_ADIS16460)
#include <ADIS16460.h>
#endif
#include <Camera.h>
#include <Timer.h>
#include <helper.h>

static void resetCb(const std_msgs::Bool & /*msg*/) { NVIC_SystemReset(); }

#ifdef ILLUMINATION_MODULE
static void pwmCb(const std_msgs::UInt8 &msg) {
  analogWrite(ILLUMINATION_PWM_PIN, msg.data);
}
#endif

/* ----- ROS ----- */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> reset_sub("/versavis/reset", &resetCb);
#ifdef ILLUMINATION_MODULE
ros::Subscriber<std_msgs::UInt8> pwm_sub("/versavis/illumination_pwm", &pwmCb);
#endif

/* ----- Timers ----- */
// In the current setup: TC5 -> IMU, TCC0 -> cam0, TCC1 -> cam1, TC3 -> cam2
// (TCC2 is used for pwm on pin 11).
// Be careful, there is NO bookkeeping whether the timer is already used or
// not. Only use a timer once, otherwise there will be unexpected behavior.
Timer timer_cam0 = Timer((Tcc *)TCC0);
Timer timer_cam1 = Timer((Tcc *)TCC1);
Timer timer_cam2 = Timer((TcCount16 *)TC3);
Timer timer_imu = Timer((TcCount16 *)TC5);

/* ----- IMU ----- */
#ifdef USE_ADIS16445
ADIS16445 imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#elif defined(USE_ADIS16448AMLZ)
ADIS16448AMLZ imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#elif defined(USE_ADIS16448BMLZ)
ADIS16448BMLZ imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#elif defined(USE_ADIS16460)
ADIS16460 imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#endif

/* ----- Cameras ----- */
Camera cam0(&nh, CAM0_TOPIC, CAM0_RATE, timer_cam0, CAM0_TYPE, CAM0_TRIGGER_PIN,
            CAM0_EXPOSURE_PIN, true);
Camera cam1(&nh, CAM1_TOPIC, CAM1_RATE, timer_cam1, CAM1_TYPE, CAM1_TRIGGER_PIN,
            CAM1_EXPOSURE_PIN, true);
Camera cam2(&nh, CAM2_TOPIC, CAM2_RATE, timer_cam2, CAM2_TYPE, CAM2_TRIGGER_PIN,
            CAM2_EXPOSURE_PIN, true);

void setup() {
  DEBUG_INIT(115200);

/* ----- Define pins and initialize. ----- */
#ifdef ADD_TRIGGERS
  pinMode(ADDITIONAL_TEST_PIN, OUTPUT);
  digitalWrite(ADDITIONAL_TEST_PIN, LOW);
#endif

#ifdef ILLUMINATION_MODULE
  pinMode(ILLUMINATION_PWM_PIN, OUTPUT);
  pinMode(ILLUMINATION_PIN, OUTPUT);
  analogWrite(ILLUMINATION_PWM_PIN, 0);
  digitalWrite(ILLUMINATION_PIN, LOW);
#endif

  delay(1000);

/* ----- ROS ----- */
#ifndef DEBUG
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(reset_sub);
#ifdef ILLUMINATION_MODULE
  nh.subscribe(pwm_sub);
#endif
#else
  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  DEBUG_PRINTLN(F("Main: Start setup."));

  imu.setup();
  cam0.setup();
  cam1.setup();
  cam2.setup();

  /* ----- Initialize all connected cameras. ----- */
  while (!cam0.isInitialized() || !cam1.isInitialized() ||
         !cam2.isInitialized()) {
    DEBUG_PRINTLN(F("Main: Initializing."));
    cam0.initialize();
    cam1.initialize();
    cam2.initialize();

#ifndef DEBUG
    nh.spinOnce();
#endif
    delay(1000);
  }

  /* -----  Declare timers ----- */
  // Enable TCC1 and TCC2 timers.
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1);
  while (GCLK->STATUS.bit.SYNCBUSY == 1) {
    ; // wait for sync
  }
  // Enable TCC2 (not used) and TC3 timers.
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
  while (GCLK->STATUS.bit.SYNCBUSY == 1) {
    ; // wait for sync
  }

  // Enable TC4 (not used) and TC5 timers.
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5);
  while (GCLK->STATUS.bit.SYNCBUSY == 1) {
    ; // wait for sync
  }

  // enable InterruptVector.
  NVIC_EnableIRQ(TCC0_IRQn);
  NVIC_EnableIRQ(TCC1_IRQn);
  NVIC_EnableIRQ(TC3_IRQn);
  NVIC_EnableIRQ(TC5_IRQn);

  imu.begin();
  cam0.begin();
  cam1.begin();
  cam2.begin();

  /* ----- Interrupt for measuring the exposure time. ----- */
  noInterrupts(); // Disable interrupts to configure them --> delay()'s
  // currently not working!

  DEBUG_PRINTLN(F("Main: Attach interrupts."));
  attachInterrupt(digitalPinToInterrupt(cam0.exposurePin()), exposureEnd0,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(cam1.exposurePin()), exposureEnd1,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(cam2.exposurePin()), exposureEnd2,
                  FALLING);
  interrupts();

  DEBUG_PRINTLN(F("Main: Setup done."));
}

void loop() {
  cam0.publish();
  cam1.publish();
  cam2.publish();
  imu.publish();

#ifndef DEBUG
  nh.spinOnce();
#endif
}

void TCC0_Handler() { // Called by cam0_timer for camera 0 trigger.
  cam0.triggerMeasurement();
}

void TCC1_Handler() { // Called by cam1_timer for camera 1 trigger.
  cam1.triggerMeasurement();
}


void TC3_Handler() {    // Called by cam2_timer for camera 2 trigger.
 // cam2.triggerMeasurement();

  if (timer_cam2.checkOverflow()) {
    digitalWrite(4, HIGH);
    delayMicroseconds(1);
    digitalWrite(4, LOW);
    timer_cam2.resetOverflow();
  }
}

void TC5_Handler() { // Called by imu_timer for imu trigger.
  imu.triggerMeasurement();
}

void exposureEnd0() {
  cam0.exposureEnd();
#ifdef ILLUMINATION_MODULE
  // Deactivate the LEDs with the last camera.
  if (!cam1.isExposing() && !cam2.isExposing()) {
    digitalWrite(ILLUMINATION_PIN, LOW);
  }
#endif
}

void exposureEnd1() {
  cam1.exposureEnd();
#ifdef ILLUMINATION_MODULE
  // Deactivate the LEDs with the last camera.
  if (!cam0.isExposing() && !cam2.isExposing()) {
    digitalWrite(ILLUMINATION_PIN, LOW);
  }
#endif
}

void exposureEnd2() {
  cam2.exposureEnd();
#ifdef ILLUMINATION_MODULE
  // Deactivate the LEDs with the last camera.
  if (!cam0.isExposing() && !cam1.isExposing()) {
    digitalWrite(ILLUMINATION_PIN, LOW);
  }
#endif
}
