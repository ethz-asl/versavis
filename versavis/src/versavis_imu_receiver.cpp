////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  versavis_imu_reciever.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  Used to convert versavis/ImuMicro to sensor_msgs/Imu. Performs scaling and
//  setting the covarance constants.
//
////////////////////////////////////////////////////////////////////////////////

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>

#include <versavis/ImuMicro.h>

class VersaVISImuReciever {
public:
  VersaVISImuReciever(ros::NodeHandle &node_handle)
      : node_handle_(node_handle) {
    ROS_INFO("Versavis IMU message reciever. Version 1.0");
    readParameters();
    imu_sub_ = node_handle_.subscribe(imu_sub_topic_, 100u,
                                      &VersaVISImuReciever::imuCallback, this);
    imu_pub_ = node_handle_.advertise<sensor_msgs::Imu>(imu_pub_topic_, 100u);
    last_msg_time_ = ros::Time(0);
  }

  ~VersaVISImuReciever() { node_handle_.shutdown(); }

  void imuCallback(const versavis::ImuMicro &imu_micro_msg) {
    ROS_INFO_ONCE("Received first IMU message.");
    if (imu_micro_msg.time.data.toNSec() < last_msg_time_.toNSec()) {
      ROS_WARN(
          "IMU message is not strictly increasing (%ld vs %ld). Dropping this "
          "message. This is normal during startup (< 1 min).",
          imu_micro_msg.time.data.toNSec(), last_msg_time_.toNSec());
      return;
    }
    last_msg_time_ = imu_micro_msg.time.data;
    sensor_msgs::Imu imu_msg;

    imu_msg.header.stamp = imu_micro_msg.time.data;
    imu_msg.header.frame_id = "imu";
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;
    imu_msg.orientation_covariance[0] = -1;
    imu_msg.orientation_covariance[1] = 0.0;
    imu_msg.orientation_covariance[2] = 0.0;
    imu_msg.orientation_covariance[3] = 0.0;
    imu_msg.orientation_covariance[4] = 0;
    imu_msg.orientation_covariance[5] = 0.0;
    imu_msg.orientation_covariance[6] = 0.0;
    imu_msg.orientation_covariance[7] = 0.0;
    imu_msg.orientation_covariance[8] = 0;
    // --- Angular Velocity.
    imu_msg.angular_velocity.x = gyroScale(imu_micro_msg.gx);
    imu_msg.angular_velocity.y = gyroScale(imu_micro_msg.gy);
    imu_msg.angular_velocity.z = gyroScale(imu_micro_msg.gz);
    imu_msg.angular_velocity_covariance[0] = imu_gyro_covariance_;
    imu_msg.angular_velocity_covariance[1] = 0.0;
    imu_msg.angular_velocity_covariance[2] = 0.0;
    imu_msg.angular_velocity_covariance[3] = 0.0;
    imu_msg.angular_velocity_covariance[4] = imu_gyro_covariance_;
    imu_msg.angular_velocity_covariance[5] = 0.0;
    imu_msg.angular_velocity_covariance[6] = 0.0;
    imu_msg.angular_velocity_covariance[7] = 0.0;
    imu_msg.angular_velocity_covariance[8] = imu_gyro_covariance_;
    // --- Linear Acceleration.
    imu_msg.linear_acceleration.x = accelScale(imu_micro_msg.ax);
    imu_msg.linear_acceleration.y = accelScale(imu_micro_msg.ay);
    imu_msg.linear_acceleration.z = accelScale(imu_micro_msg.az);
    imu_msg.linear_acceleration_covariance[0] = imu_acceleration_covariance_;
    imu_msg.linear_acceleration_covariance[1] = 0.0;
    imu_msg.linear_acceleration_covariance[2] = 0.0;
    imu_msg.linear_acceleration_covariance[3] = 0.0;
    imu_msg.linear_acceleration_covariance[4] = imu_acceleration_covariance_;
    imu_msg.linear_acceleration_covariance[5] = 0.0;
    imu_msg.linear_acceleration_covariance[6] = 0.0;
    imu_msg.linear_acceleration_covariance[7] = 0.0;
    imu_msg.linear_acceleration_covariance[8] = imu_acceleration_covariance_;

    imu_pub_.publish(imu_msg);
  }

  void readParameters() {
    ROS_INFO("Read ROS parameters.");
    node_handle_.param("imu_sub_topic", imu_sub_topic_,
                       std::string("/versavis/imu_micro"));
    node_handle_.param("imu_pub_topic", imu_pub_topic_,
                       std::string("/versavis/imu"));
    node_handle_.param("imu_accelerator_sensitivity",
                       imu_accelerator_sensitivity_, 0.00025);
    node_handle_.param("imu_gyro_sensitivity", imu_gyro_sensitivity_, 0.05);
    node_handle_.param("imu_acceleration_covariance",
                       imu_acceleration_covariance_, 0.043864908);
    node_handle_.param("imu_gyro_covariance", imu_gyro_covariance_, 6e-9);
  }

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;
  // ros::Subscriber img_time_sub_;
  std::string imu_sub_topic_;
  std::string imu_pub_topic_;
  ros::Time last_msg_time_;
  double imu_accelerator_sensitivity_;
  double imu_gyro_sensitivity_;
  double imu_acceleration_covariance_;
  double imu_gyro_covariance_;

  // Converts accelerometer data output from the regRead() function and
  // returns
  // acceleration in m2/s.
  float accelScale(const int16_t &sensor_data) {
    return (sensor_data * imu_accelerator_sensitivity_ *
            9.81); // Multiply by accel sensitivity
                   // (e.g. 4000 LSB/g for ADIS16445)
                   // and acceleration of gravity.
  }

  // Converts gyro data output from the regRead() function and returns gyro rate
  // in rad/sec.
  float gyroScale(const int16_t &sensor_data) {
    return (
        sensor_data * imu_gyro_sensitivity_ * M_PI /
        180); // Multiply by gyro sensitivity (e.g. 100 LSB/dps for ADIS16445);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "versavis_imu_reciever");
  ros::NodeHandle node_handle("~");
  VersaVISImuReciever reciever(node_handle);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
