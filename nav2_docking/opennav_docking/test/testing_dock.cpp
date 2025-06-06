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
enum class TestFailureDockMode {
    NONE,
    FAIL_INITIAL_PERCEPTION_ONCE, // getRefinedPose returns false once, then true
    FAIL_ALL_PERCEPTION,          // getRefinedPose always returns false
    FAIL_IS_DOCKED_CHECK,         // isDocked always returns false
    FAIL_IS_CHARGING_CHECK        // isCharging always returns false (if applicable)
  };

// Tests error cases in unit test handling
class TestFailureDock : public opennav_docking_core::ChargingDock
{
public:
  TestFailureDock()
  : ChargingDock()
  {}

  // Public members for test inspection
  bool detector_started{false};
  bool detector_stopped{false};
  int get_refined_pose_call_count{0};
  int start_detection_call_count{0};
  int stop_detection_call_count{0};
  TestFailureDockMode current_failure_mode{TestFailureDockMode::NONE};

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer>)
  {
    node_ = parent.lock();
    if (!node_) {
      throw std::runtime_error{"Failed to lock node"};
    }
    dock_direction_ = opennav_docking_core::DockDirection::FORWARD;

    if (!node_->has_parameter(name_ + ".test_failure_mode")) {
      node_->declare_parameter(name_ + ".test_failure_mode", rclcpp::ParameterValue(std::string("NONE")));
    }
    name_ = name;
  }

  virtual void cleanup() {}
  virtual void activate()
  {
    // Reset test-specific state on activation
    std::string mode_str = node_->get_parameter(name_ + ".test_failure_mode").as_string();
    if (mode_str == "FAIL_INITIAL_PERCEPTION_ONCE") {
      current_failure_mode = TestFailureDockMode::FAIL_INITIAL_PERCEPTION_ONCE;
    } else if (mode_str == "FAIL_ALL_PERCEPTION") {
      current_failure_mode = TestFailureDockMode::FAIL_ALL_PERCEPTION;
    } else if (mode_str == "FAIL_IS_DOCKED_CHECK") {
      current_failure_mode = TestFailureDockMode::FAIL_IS_DOCKED_CHECK;
    } else if (mode_str == "FAIL_IS_CHARGING_CHECK") {
      current_failure_mode = TestFailureDockMode::FAIL_IS_CHARGING_CHECK;
    } else {
      current_failure_mode = TestFailureDockMode::NONE;
    }

    detector_started = false;
    detector_stopped = false;
    get_refined_pose_call_count = 0;
    start_detection_call_count = 0;
    stop_detection_call_count = 0;
  }
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
    get_refined_pose_call_count++;
    if (current_failure_mode == TestFailureDockMode::FAIL_ALL_PERCEPTION) {
      return false;
    }
    if (current_failure_mode == TestFailureDockMode::FAIL_INITIAL_PERCEPTION_ONCE &&
      get_refined_pose_call_count == 1)
    {
      return false; // Fail the first time, succeed subsequently
    }
    return true; // Default to successful perception
  }

  virtual bool isDocked()
  {
    bool dock_action_called = false;
    node_->get_parameter("dock_action_called", dock_action_called);
    if (dock_action_called) {
      return true;
    } // Compatibility with existing tests

    if (current_failure_mode == TestFailureDockMode::FAIL_IS_DOCKED_CHECK) {
      return false;
    }
    return true; // Default to successfully docked if not failing
  }

  virtual bool isCharging()
  {
    if (current_failure_mode == TestFailureDockMode::FAIL_IS_CHARGING_CHECK) {
      return false;
    }
    return true; // Default to charging if not failing (for charging docks)
  }

  virtual bool disableCharging()
  {
    return true;
  }

  virtual void startDetectionProcess() override
  {
    start_detection_call_count++;
    detector_started = true;
    detector_stopped = false; // Reset if re-started
  }

  virtual void stopDetectionProcess() override
  {
    stop_detection_call_count++;
    detector_stopped = true;
  }

  virtual bool hasStoppedCharging()
  {
    return true;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
};

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::TestFailureDock, opennav_docking_core::ChargingDock)
