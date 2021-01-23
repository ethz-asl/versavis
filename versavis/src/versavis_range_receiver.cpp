////////////////////////////////////////////////////////////////////////////////
//  December 2019
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  versavis_range_reciever.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  Used to convert versavis/RangeMicro to sensor_msgs/Range. Performs scaling,
//  signal strength, and range filtering.
//
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>

#include <sensor_msgs/Range.h>
#include <versavis/RangeMicro.h>

class VersaVisRangeReceiver {
public:
  VersaVisRangeReceiver(ros::NodeHandle &node_handle)
      : node_handle_(node_handle) {
    ROS_INFO("Versavis range message receiver. Version 1.0");
    readParameters();
    range_sub_ = node_handle_.subscribe(
        range_sub_topic_, 100u, &VersaVisRangeReceiver::rangeCallback, this);
    range_pub_ =
        node_handle_.advertise<sensor_msgs::Range>(range_pub_topic_, 100u);
  }

  ~VersaVisRangeReceiver() { node_handle_.shutdown(); }

  void rangeCallback(const versavis::RangeMicro &range_micro_msg) {
    ROS_INFO_ONCE("Received first Range message from topic: %s",
                  range_sub_topic_.c_str());
    if (range_micro_msg.time.data.toNSec() < last_msg_time_.toNSec()) {
      ROS_WARN("Range message from topic %s is not strictly increasing (%ld vs "
               "%ld). Dropping this message. This is normal during startup (< "
               "1 min).",
               range_sub_topic_.c_str(), range_micro_msg.time.data.toNSec(),
               last_msg_time_.toNSec());
      return;
    }
    last_msg_time_ = range_micro_msg.time.data;

    // Populate range measurement.
    range_msg_.header.stamp = range_micro_msg.time.data;
    range_msg_.range = rangeScale(range_micro_msg.range);

    // Filter messages out of bounds.
    if (range_msg_.range < range_msg_.min_range ||
        range_msg_.range > range_msg_.max_range) {
      ROS_DEBUG(
          "Discarding range message out of bounds RANGE: %f MIN: %f MAX: %f.",
          range_msg_.range, range_msg_.min_range, range_msg_.max_range);
      return;
    }

    // Filter messages with bad signal strength.
    if (range_micro_msg.signal_strength < min_signal_strength_) {
      ROS_DEBUG("Discarding range message bad signal strength: %d MIN: %d.",
                range_micro_msg.signal_strength, min_signal_strength_);
      return;
    }

    range_pub_.publish(range_msg_);
  }

  void readParameters() {
    ROS_INFO("Read ROS parameters.");
    node_handle_.param("range_sub_topic", range_sub_topic_,
                       std::string("/versavis/range_micro"));
    node_handle_.param("range_pub_topic", range_pub_topic_,
                       std::string("/versavis/range"));

    std::string frame_id = "range_sensor";
    node_handle_.param("frame_id", frame_id, frame_id);
    range_msg_.header.frame_id = frame_id;

    int radiation_type = sensor_msgs::Range::INFRARED;
    node_handle_.param("radiation_type", radiation_type, radiation_type);
    range_msg_.radiation_type = static_cast<uint8_t>(radiation_type);

    node_handle_.param("field_of_view", range_msg_.field_of_view,
                       static_cast<float>(2.0 * M_PI));
    node_handle_.param("min_range", range_msg_.min_range,
                       std::numeric_limits<float>::lowest());
    node_handle_.param("max_range", range_msg_.max_range,
                       std::numeric_limits<float>::max());
    node_handle_.param("range_sensitivity", range_sensitivity_,
                       range_sensitivity_);

    int min_signal_strength = min_signal_strength_;
    node_handle_.param("min_signal_strength", min_signal_strength,
                       min_signal_strength);
    // Clamp signal strength value.
    min_signal_strength_ = std::max(0, min_signal_strength);
    min_signal_strength_ =
        std::min(min_signal_strength_, std::numeric_limits<uint8_t>::max());
  }

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber range_sub_;
  ros::Publisher range_pub_;
  std::string range_sub_topic_;
  std::string range_pub_topic_;
  ros::Time last_msg_time_;

  sensor_msgs::Range range_msg_;
  double range_sensitivity_ = 1.0;

  uint8_t min_signal_strength_ = 0;

  // Converts raw range data to distance in meters.
  inline float rangeScale(const int16_t sensor_data) const {
    return static_cast<float>(sensor_data * range_sensitivity_);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "versavis_range_receiver");
  ros::NodeHandle node_handle("~");
  VersaVisRangeReceiver receiver(node_handle);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
