////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  versavis_synchronizer_node.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  Node wrapper for versavis_synchronizer.
//
////////////////////////////////////////////////////////////////////////////////

#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "versavis_synchronizer_node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  ROS_INFO_STREAM("Started " << nodelet_name << " nodelet.");
  nodelet.load(nodelet_name, "versavis/VersaVISSynchronizerNodelet", remap,
               nargv);
  ros::spin();
  return 0;
}
