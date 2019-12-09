////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  versavis_synchronizer_nodelet.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  Nodelet wrapper for versavis_synchronizer.
//
////////////////////////////////////////////////////////////////////////////////

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/versavis_synchronizer.h"

namespace versavis {

class VersaVISSynchronizerNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      synchronizer_ = std::make_shared<VersaVISSynchronizer>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<VersaVISSynchronizer> synchronizer_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::VersaVISSynchronizerNodelet, nodelet::Nodelet);
