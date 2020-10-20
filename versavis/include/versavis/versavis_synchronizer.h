////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  versavis_synchronizer.h
////////////////////////////////////////////////////////////////////////////////
//
//  Used to merge image time stamps and image data.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ARDUINO_VI_SYNC_ARDUINO_VI_SYNCHRONIZER_H_
#define ARDUINO_VI_SYNC_ARDUINO_VI_SYNCHRONIZER_H_

#include <cmath>
#include <mutex>
#include <sstream>
#include <string>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>

#include <image_numbered_msgs/ImageNumbered.h>
#include <versavis/ImuMicro.h>
#include <versavis/TimeNumbered.h>

namespace versavis {
class VersaVISSynchronizer {
public:
  VersaVISSynchronizer(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private);
  ~VersaVISSynchronizer();

  void imageCallback(const image_numbered_msgs::ImageNumbered &image_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo& camera_info_msg);
  void imageTimeCallback(const versavis::TimeNumbered &image_time_msg);
  void publishImg(const image_numbered_msgs::ImageNumbered &image_msg);

  bool readParameters();

  // Find nearest time stamp in a list of candidates (only newer are possible
  // due to driver logic).
  void associateTimeStampsAndCleanUp();

private:
  // ROS members.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport image_transport_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  image_transport::Publisher image_fast_pub_;
  image_transport::Publisher image_slow_pub_;
  ros::Publisher camera_info_fast_pub_;
  ros::Publisher camera_info_slow_pub_;
  ros::Publisher initialized_pub_;
  ros::Subscriber image_time_sub_;

  // Topic names.
  std::string versavis_topic_;
  std::string driver_topic_;
  std::string camera_info_sub_topic_;
  std::string image_fast_pub_topic_;
  std::string image_slow_pub_topic_;
  std::string camera_info_fast_pub_topic_;
  std::string camera_info_slow_pub_topic_;
  std::string image_time_sub_topic_;
  std::string initialized_pub_topic_;

  // Association members.
  versavis::TimeNumbered init_time_;
  std::vector<versavis::TimeNumbered> image_time_stamp_candidates_;
  std::vector<image_numbered_msgs::ImageNumbered> image_candidates_;
  bool received_first_camera_info_;
  sensor_msgs::CameraInfo camera_info_msg_;
  ros::Time last_stamp_;
  ros::Time init_timestamp_;

  // Constants.
  const uint8_t kMaxImageCandidateLength;
  const uint8_t kMinSuccessfullConsecutiveMatches;
  const double kMaxImageDelayThreshold;

  // Image numbers and initialization.
  uint8_t init_number_;
  uint64_t last_image_number_;
  int64_t offset_;
  bool initialized_;

  // Configuration.
  uint8_t slow_publisher_image_counter_;
  bool publish_slow_images_;
  int publish_every_n_image_;
  bool forward_camera_info_;
  ros::Duration imu_offset_;

  std::mutex mutex_;
};
} // namespace versavis

#endif // ARDUINO_VI_SYNC_ARDUINO_VI_SYNCHRONIZER_H_
