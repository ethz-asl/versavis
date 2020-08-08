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
#elif defined(USE_VN100)
#include <VN100.h>
#endif
#include <Camera.h>
#ifdef USE_LIDAR_LITE
#include <LidarLite.h>
#endif
#include <Timer.h>
#ifdef USE_U_LANDING
#include <ULanding.h>
#endif
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
// The SAMD21G has 3 TC and 3 TCC timers.
// In the current setup: TC5 -> IMU, TCC0 -> cam0, TCC1 -> cam1, TC3 -> cam2
// (TCC2 is used for pwm on pin 11).
// Be careful, there is NO bookkeeping whether the timer is already used or
// not. Only use a timer once, otherwise there will be unexpected behavior.
Timer timer_cam0 = Timer((Tcc *)TCC0);
Timer timer_cam1 = Timer((Tcc *)TCC1);
#ifdef USE_CAM2
Timer timer_cam2 = Timer((TcCount16 *)TC3);
#endif
#ifdef USE_U_LANDING // Need to deactivate cam2!
Timer timer_u_landing = Timer((TcCount16 *)TC3);
#endif
#ifdef USE_LIDAR_LITE
Timer timer_lidar_lite = Timer((TcCount16 *)TC4);
#endif
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
#elif defined(USE_VN100)
VN100 imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu);
#endif

/* ----- Cameras ----- */
Camera cam0(&nh, CAM0_TOPIC, CAM0_RATE, timer_cam0, CAM0_TYPE, CAM0_TRIGGER_PIN,
            CAM0_EXPOSURE_PIN, true);
Camera cam1(&nh, CAM1_TOPIC, CAM1_RATE, timer_cam1, CAM1_TYPE, CAM1_TRIGGER_PIN,
            CAM1_EXPOSURE_PIN, true);
#ifdef USE_CAM2
Camera cam2(&nh, CAM2_TOPIC, CAM2_RATE, timer_cam2, CAM2_TYPE, CAM2_TRIGGER_PIN,
            CAM2_EXPOSURE_PIN, true);
#endif

/* ----- uLanding ----- */
#ifdef USE_U_LANDING
ULanding u_landing(&nh, U_LANDING_TOPIC, U_LANDING_RATE, timer_u_landing,
                   &U_LANDING_UART);
#endif

/* ----- Lidar Lite ----- */
#ifdef USE_LIDAR_LITE
LidarLite lidar_lite(&nh, LIDAR_LITE_TOPIC, LIDAR_LITE_RATE, timer_lidar_lite);
#endif

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
#ifdef USE_CAM2
  cam2.setup();
#endif
#ifdef USE_U_LANDING
  u_landing.setup();
#endif
#ifdef USE_LIDAR_LITE
  lidar_lite.setup();
#endif

  /* ----- Initialize all connected cameras. ----- */
  while (!cam0.isInitialized() || !cam1.isInitialized()
#ifdef USE_CAM2
         || !cam2.isInitialized()
#endif
  ) {
    DEBUG_PRINTLN(F("Main: Initializing."));
    cam0.initialize();
    cam1.initialize();
#ifdef USE_CAM2
    cam2.initialize();
#endif

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

  // Enable TC4 (LidarLite) and TC5 timers.
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5);
  while (GCLK->STATUS.bit.SYNCBUSY == 1) {
    ; // wait for sync
  }

  // enable InterruptVector.
  NVIC_EnableIRQ(TCC0_IRQn);
  NVIC_EnableIRQ(TCC1_IRQn);
#ifdef USE_CAM2
  NVIC_EnableIRQ(TC3_IRQn);
#endif
#ifdef USE_U_LANDING
  NVIC_EnableIRQ(TC3_IRQn);
#endif
#ifdef USE_LIDAR_LITE
  NVIC_EnableIRQ(TC4_IRQn);
#endif
  NVIC_EnableIRQ(TC5_IRQn);

  imu.begin();
  cam0.begin();
  cam1.begin();
#ifdef USE_CAM2
  cam2.begin();
#endif
#ifdef USE_U_LANDING
  u_landing.begin();
#endif
#ifdef USE_LIDAR_LITE
  lidar_lite.begin();
#endif

  /* ----- Interrupt for measuring the exposure time. ----- */
  noInterrupts(); // Disable interrupts to configure them --> delay()'s
  // currently not working!

  DEBUG_PRINTLN(F("Main: Attach interrupts."));
  attachInterrupt(digitalPinToInterrupt(cam0.exposurePin()), exposureEnd0,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(cam1.exposurePin()), exposureEnd1,
                  FALLING);
#ifdef USE_CAM2
  attachInterrupt(digitalPinToInterrupt(cam2.exposurePin()), exposureEnd2,
                  FALLING);
#endif
  interrupts();

  DEBUG_PRINTLN(F("Main: Setup done."));
}

void loop() {
  cam0.publish();
  cam1.publish();
#ifdef USE_CAM2
  cam2.publish();
#endif
  imu.publish();
#ifdef USE_U_LANDING
  u_landing.publish();
#endif
#ifdef USE_LIDAR_LITE
  lidar_lite.publish();
#endif

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

#ifdef USE_CAM2
void TC3_Handler() { // Called by cam2_timer for camera 2 trigger.
  cam2.triggerMeasurement();
}
#endif

#ifdef USE_U_LANDING
void TC3_Handler() { // Called by u_landing_timer for uLanding trigger.
  u_landing.triggerMeasurement();
}
#endif

#ifdef USE_LIDAR_LITE
void TC4_Handler() { // Called by lidar_lite_timer for lidar lite trigger.
  lidar_lite.triggerMeasurement();
}
#endif

void TC5_Handler() {
  imu.triggerMeasurement(); // Called by imu_timer for imu trigger.
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

#ifdef USE_CAM2
void exposureEnd2() {
  cam2.exposureEnd();
#ifdef ILLUMINATION_MODULE
  // Deactivate the LEDs with the last camera.
  if (!cam0.isExposing() && !cam1.isExposing()) {
    digitalWrite(ILLUMINATION_PIN, LOW);
  }
#endif
}
#endif
