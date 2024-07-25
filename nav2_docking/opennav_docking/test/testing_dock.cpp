// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <vector>

#include "opennav_docking_core/charging_dock.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"

namespace opennav_docking
{

// Tests error cases in unit test handling
class TestFailureDock : public opennav_docking_core::ChargingDock
{
public:
  TestFailureDock()
  : ChargingDock()
  {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string &, std::shared_ptr<tf2_ros::Buffer>)
  {
    node_ = parent.lock();
    if (!node_) {
      throw std::runtime_error{"Failed to lock node"};
    }
  }

  virtual void cleanup() {}
  virtual void activate() {}
  virtual void deactivate() {}

  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose &, const std::string &)
  {
    // Declared in test
    std::string exception;
    node_->get_parameter("exception_to_throw", exception);
    if (exception == "TransformException") {
      throw tf2::TransformException("TransformException");
    } else if (exception == "DockNotInDB") {
      throw opennav_docking_core::DockNotInDB("DockNotInDB");
    } else if (exception == "DockNotValid") {
      throw opennav_docking_core::DockNotValid("DockNotValid");
    } else if (exception == "FailedToStage") {
      throw opennav_docking_core::FailedToStage("FailedToStage");
    } else if (exception == "FailedToDetectDock") {
      throw opennav_docking_core::FailedToDetectDock("FailedToDetectDock");
    } else if (exception == "FailedToControl") {
      throw opennav_docking_core::FailedToControl("FailedToControl");
    } else if (exception == "FailedToCharge") {
      throw opennav_docking_core::FailedToCharge("FailedToCharge");
    } else if (exception == "DockingException") {
      throw opennav_docking_core::DockingException("DockingException");
    } else if (exception == "exception") {
      throw std::exception();
    }
    return geometry_msgs::msg::PoseStamped();
  }

  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped &, std::string)
  {
    // Always return false to trigger a timeout, when no exceptions are thrown
    return false;
  }

  virtual bool isDocked()
  {
    bool dock_action_called;
    node_->get_parameter("dock_action_called", dock_action_called);
    return dock_action_called;
  }

  virtual bool isCharging()
  {
    return false;
  }

  virtual bool disableCharging()
  {
    return true;
  }

  virtual bool hasStoppedCharging()
  {
    return true;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::TestFailureDock, opennav_docking_core::ChargingDock)
