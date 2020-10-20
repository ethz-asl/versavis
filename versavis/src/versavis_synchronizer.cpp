////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  versavis_synchronizer.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  Used to merge image time stamps and image data.
//
////////////////////////////////////////////////////////////////////////////////

#include "versavis/versavis_synchronizer.h"

namespace versavis {

VersaVISSynchronizer::VersaVISSynchronizer(const ros::NodeHandle &nh,
                                           const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), image_transport_(nh),
      received_first_camera_info_(false), kMaxImageCandidateLength(10),
      kMinSuccessfullConsecutiveMatches(4), kMaxImageDelayThreshold(0.1),
      slow_publisher_image_counter_(0), init_number_(0), initialized_(false),
      publish_slow_images_(false), publish_every_n_image_(10),
      forward_camera_info_(false) {
  ROS_INFO("Versavis message compiler started. Version 1.0");
  readParameters();

  // Subscriber of the non-corrected image directly from the ROS camera driver.
  image_sub_ = nh_.subscribe(driver_topic_, 10u,
                             &VersaVISSynchronizer::imageCallback, this);
  ROS_INFO("Subscribing to %s.", driver_topic_.c_str());

  // Subscriber to an exsternal publisher of a camera info topic, this can be
  // the driver or another node. It is not required that the camera info message
  // is synchrinized with the image. This node will simply take the most recent
  // camera info message and publish it in sync with the restamped image
  // messabge.
  if (forward_camera_info_) {
    camera_info_sub_ =
        nh_.subscribe(camera_info_sub_topic_, 10u,
                      &VersaVISSynchronizer::cameraInfoCallback, this);
    ROS_INFO("Subscribing to camera info topic %s.",
             camera_info_sub_topic_.c_str());
  }

  // Subscriber for the image time message containing both the corrected
  // timestamp and the sequence number.
  image_time_sub_ =
      nh_.subscribe(image_time_sub_topic_, 10u,
                    &VersaVISSynchronizer::imageTimeCallback, this);
  ROS_INFO("Subscribing to %s.", image_time_sub_topic_.c_str());

  // Publisher to let the triggering board know as soon as the camera is
  // initialized.
  initialized_pub_ = nh_.advertise<std_msgs::Bool>(initialized_pub_topic_, 1u);
  ROS_INFO("Publishing initialization status to %s.",
           initialized_pub_topic_.c_str());

  image_fast_pub_ = image_transport_.advertise(image_fast_pub_topic_, 10u);
  ROS_INFO("Publishing image to %s.", image_fast_pub_topic_.c_str());

  if (publish_slow_images_) {
    image_slow_pub_ = image_transport_.advertise(image_slow_pub_topic_, 1u);
    ROS_INFO("Publishing (slow) image to %s.", image_slow_pub_topic_.c_str());
  }

  if (forward_camera_info_) {
    camera_info_fast_pub_ =
        nh_.advertise<sensor_msgs::CameraInfo>(camera_info_fast_pub_topic_, 10u);
    ROS_INFO_STREAM("Publishing camera info to " << camera_info_fast_pub_topic_);

    if (publish_slow_images_) {
      camera_info_slow_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
          camera_info_slow_pub_topic_, 10u);
      ROS_INFO_STREAM("Publishing (slow) camera info to "
                      << camera_info_slow_pub_topic_);
    }
  }
}

VersaVISSynchronizer::~VersaVISSynchronizer() {
  image_time_stamp_candidates_.clear();
  image_candidates_.clear();
  nh_.shutdown();
}

void VersaVISSynchronizer::associateTimeStampsAndCleanUp() {
  if (image_candidates_.empty() || image_time_stamp_candidates_.empty()) {
    return;
  }
  // Association and cleanup.
  uint32_t last_image_time = 0u;
  uint32_t last_image = 0u;
  auto image_idx = image_candidates_.begin();
  auto image_time_idx = image_time_stamp_candidates_.begin();
  bool image_found = false;

  while (image_idx != image_candidates_.end()) {
    while (image_time_idx != image_time_stamp_candidates_.end()) {
      if (image_idx->number == image_time_idx->number + offset_) {
        image_idx->image.header.stamp = image_time_idx->time + imu_offset_;
        publishImg(*image_idx);
        last_image = image_idx->number;
        last_image_time = image_time_idx->number;
        image_idx = image_candidates_.erase(image_idx);
        image_time_idx = image_time_stamp_candidates_.erase(image_time_idx);
        image_found = true;
        break;
      } else {
        ++image_time_idx;
      }
    }
    image_time_idx = image_time_stamp_candidates_.begin();
    if (!image_found) {
      ++image_idx;
    } else {
      image_found = false;
    }
  }

  // Delete old messages to prevent non-consecutive publishing.
  image_idx = image_candidates_.begin();
  image_time_idx = image_time_stamp_candidates_.begin();
  while (image_idx != image_candidates_.end()) {
    if (image_idx->number <= last_image) {
      ROS_WARN("%s: Deleted old image message %ld.",
               image_fast_pub_topic_.c_str(), image_idx->number);
      image_idx = image_candidates_.erase(image_idx);
    } else {
      ++image_idx;
    }
  }
  while (image_time_idx != image_time_stamp_candidates_.end()) {
    if (image_time_idx->number <= last_image_time) {
      ROS_WARN("%s: Deleted old image time message %ld.",
               image_fast_pub_topic_.c_str(), image_time_idx->number + offset_);
      image_time_idx = image_time_stamp_candidates_.erase(image_time_idx);
    } else {
      ++image_time_idx;
    }
  }
}

void VersaVISSynchronizer::imageCallback(
    const image_numbered_msgs::ImageNumbered &image_msg) {
  std::lock_guard<std::mutex> mutex_lock(mutex_);
  ROS_INFO_ONCE("%s: Received first message with %s encoding.",
                image_fast_pub_topic_.c_str(),
                image_msg.image.encoding.c_str());
  if (initialized_) {
    image_candidates_.emplace_back(image_msg);
    associateTimeStampsAndCleanUp();
    if (image_candidates_.size() > kMaxImageCandidateLength) {
      image_candidates_.erase(image_candidates_.begin());
      ROS_WARN("%s: Image candidates buffer overflow at %ld.",
               image_fast_pub_topic_.c_str(), image_msg.number);
    }
  } else {
    // Initialization procedure.
    ROS_INFO_ONCE("%s: Initializing...", image_fast_pub_topic_.c_str());

    const int64_t offset = image_msg.number - init_time_.number;
    ROS_INFO("%s: Current offset %ld with time offset %0.5f",
             image_fast_pub_topic_.c_str(), offset,
             ros::Time::now().toSec() - init_timestamp_.toSec());
    // Ensure that we match to the right image by enforcing a similar arrival
    // time and the same offset.
    // Note: The image_time message should arrive prior to the image.
    if (ros::Time::now().toSec() - init_timestamp_.toSec() <
            kMaxImageDelayThreshold &&
        offset == offset_) {
      ++init_number_;
      if (init_number_ >= kMinSuccessfullConsecutiveMatches) {
        ROS_INFO("%s: Initialized with %ld offset.",
                 image_fast_pub_topic_.c_str(), offset);
        initialized_ = true;
        std_msgs::Bool init_msg;
        init_msg.data = true;
        initialized_pub_.publish(init_msg);

        ROS_INFO_STREAM("Versavis node '"
                        << nh_private_.getNamespace()
                        << "' confirmed initialization of sync!");
      }
    } else {
      init_number_ = 0;
    }
    offset_ = offset;
  }
}

void VersaVISSynchronizer::cameraInfoCallback(
    const sensor_msgs::CameraInfo &camera_info_msg) {
  std::lock_guard<std::mutex> mutex_lock(mutex_);
  ROS_INFO_ONCE("%s: Received first camera info message.",
                camera_info_sub_topic_.c_str());
  received_first_camera_info_ = true;
  camera_info_msg_ = camera_info_msg;
}

void VersaVISSynchronizer::imageTimeCallback(
    const versavis::TimeNumbered &image_triggered_time_msg) {
  std::lock_guard<std::mutex> mutex_lock(mutex_);
  ROS_INFO_ONCE("%s: Received first image time stamp message.",
                image_fast_pub_topic_.c_str());
  if (initialized_) {
    image_time_stamp_candidates_.emplace_back(image_triggered_time_msg);
    associateTimeStampsAndCleanUp();
    if (image_time_stamp_candidates_.size() > kMaxImageCandidateLength) {
      image_time_stamp_candidates_.erase(image_time_stamp_candidates_.begin());
      ROS_WARN("%s: Time candidates buffer overflow at %ld.",
               image_fast_pub_topic_.c_str(), image_triggered_time_msg.number);
    }
  } else {
    init_timestamp_ = ros::Time::now();
    init_time_ = image_triggered_time_msg;
  }
}

void VersaVISSynchronizer::publishImg(
    const image_numbered_msgs::ImageNumbered &image_msg) {
  if (image_msg.image.header.stamp.toSec() == 0) {
    ROS_WARN("%s: Zero timestamp for %ld.", image_fast_pub_topic_.c_str(),
             image_msg.number);
    return;
  }

  if (image_msg.image.header.stamp.toSec() <= last_stamp_.toSec()) {
    ROS_WARN("%s: Non-increasing timestamp for %ld.",
             image_fast_pub_topic_.c_str(), image_msg.number);
    return;
  }
  if (image_msg.number > last_image_number_ + 1) {
    ROS_WARN("%s: Skipped %ld frame for unknown reasons.",
             image_fast_pub_topic_.c_str(),
             image_msg.number - 1 - last_image_number_);
  }
  last_image_number_ = image_msg.number;
  last_stamp_ = image_msg.image.header.stamp;

  if (forward_camera_info_) {
    if (received_first_camera_info_) {
      camera_info_msg_.header.stamp = image_msg.image.header.stamp;
      camera_info_fast_pub_.publish(camera_info_msg_);
    } else {
      ROS_WARN_THROTTLE(2.0,
                        "No camera info received yet, will not publish image.");
    }
  }
  image_fast_pub_.publish(image_msg.image);


  if (publish_slow_images_) {
    // Publish color/raw image at lower rate.
    if (slow_publisher_image_counter_ >=
        publish_every_n_image_) {
      if (forward_camera_info_) {
        if (received_first_camera_info_) {
          camera_info_msg_.header.stamp = image_msg.image.header.stamp;
          camera_info_slow_pub_.publish(camera_info_msg_);
        } else {
          ROS_WARN_THROTTLE(
              2.0, "No camera info received yet, will not publish image.");
        }
      }
      image_slow_pub_.publish(image_msg.image);
      slow_publisher_image_counter_ = 0u;
    } else {
      ++slow_publisher_image_counter_;
    }
  }
}

bool VersaVISSynchronizer::readParameters() {
  ROS_INFO("Read ROS parameters.");
  nh_private_.param<bool>("publish_slow_images", publish_slow_images_,
                          publish_slow_images_);
  nh_private_.param<int>("publish_every_n_image", publish_every_n_image_,
                         publish_every_n_image_);

  if (!nh_private_.getParam("driver_topic", driver_topic_)) {
    ROS_ERROR("Define an image topic from the camera driver.");
  }

  if (!nh_private_.getParam("versavis_topic", versavis_topic_)) {
    ROS_ERROR("Define a topic range where the corrected image is published.");
  } else {
    if (versavis_topic_.back() != '/') {
      versavis_topic_ = versavis_topic_ + "/";
    }
    image_time_sub_topic_ = versavis_topic_ + "image_time";
    image_fast_pub_topic_ = versavis_topic_ + "image_raw";
    camera_info_fast_pub_topic_ = versavis_topic_ + "camera_info";
    initialized_pub_topic_ = versavis_topic_ + "init";
    if (publish_slow_images_) {
      image_slow_pub_topic_ = versavis_topic_ + "slow/image_raw";
      camera_info_slow_pub_topic_ = versavis_topic_ + "slow/camera_info";
    }
  }

  if (nh_private_.getParam("camera_info_topic", camera_info_sub_topic_)) {
    if (!camera_info_sub_topic_.empty()) {
      ROS_INFO_STREAM("A camera info topic has been provided, versavis will "
                      "synchronize and forward it to '"
                      << camera_info_sub_topic_ << "'");
      forward_camera_info_ = true;
    }
  }

  int imu_offset_us;
  nh_private_.param("imu_offset_us", imu_offset_us, 0);
  imu_offset_ = ros::Duration(imu_offset_us / 1e6);
  ROS_INFO("IMU has a calibrated dalay of %0.0f us.",
           imu_offset_.toSec() * 1e6);
  return true;
}

} // namespace versavis
