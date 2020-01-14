#ifndef versavis_configuration_h
#define versavis_configuration_h

/* ----- General configuration -----*/
// Activate USB serial interface for ARM processor types.
#define USE_USBCON

// Specify the CPU frequency of the controller.
#define CPU_FREQ_HZ 48e6

// Specify the trigger pulse width;
#define TRIGGER_PULSE_US 10

/* ----- Camera configuration ----*/
// Camera 0
#define CAM0_TOPIC "/versavis/cam0/"
#define CAM0_RATE 20
#define CAM0_TYPE trigger_type::NON_INVERTED
#define CAM0_TRIGGER_PIN 14
#define CAM0_EXPOSURE_PIN 5

// Camera 1
#define CAM1_TOPIC ""
#define CAM1_RATE 20
#define CAM1_TYPE trigger_type::NON_INVERTED
#define CAM1_TRIGGER_PIN 15
#define CAM1_EXPOSURE_PIN 6

// Camera 2
#define CAM2_TOPIC ""
#define CAM2_RATE 5
#define CAM2_TYPE trigger_type::NON_INVERTED
#define CAM2_TRIGGER_PIN 16
#define CAM2_EXPOSURE_PIN 7

/* ----- IMU -----*/
// Possible values: USE_ADIS16445, USE_ADIS16448AMLZ, USE_ADIS16448BMLZ,
// USE_ADIS16460 and USE_VN100
#define USE_VN100
#define IMU_TOPIC "/versavis/imu_micro"
#define IMU_RATE 200

/* ----- Additional triggers ----- */
// Define whether additional test outputs should be used.
// #define ADD_TRIGGERS
#ifdef ADD_TRIGGERS
#define ADDITIONAL_TEST_PIN 2
#endif

/* ----- Illumination module ----- */
// Activation of the illumination module.
// #define ILLUMINATION_MODULE
#ifdef ILLUMINATION_MODULE
#define ILLUMINATION_PWM_PIN 2
#define ILLUMINATION_PIN 26
#endif

/* ----- Debug mode. ----- */
// Define whether debug mode should be used. This provides output on the
// standard console but invalidates ROS communication.
// #define DEBUG

#endif // versavis_configuration_h
