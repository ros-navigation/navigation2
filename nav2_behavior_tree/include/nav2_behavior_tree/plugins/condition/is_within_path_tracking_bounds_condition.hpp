// Copyright (c) 2025 Berkan Tali
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_PATH_TRACKING_BOUNDS_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_PATH_TRACKING_BOUNDS_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>
#include <limits>

#include "behaviortree_cpp/condition_node.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that subscribes to /tracking_feedback and returns SUCCESS
 * if the error is within the max_error input port, FAILURE otherwise
 */
class IsWithinPathTrackingBoundsCondition : public BT::ConditionNode
{
public:
  IsWithinPathTrackingBoundsCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsWithinPathTrackingBoundsCondition() = delete;
  ~IsWithinPathTrackingBoundsCondition() override = default;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_error", "Maximum allowed tracking error")
    };
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("is_within_path_tracking_bounds_node")};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav2_msgs::msg::TrackingFeedback>::SharedPtr tracking_feedback_sub_;
  double last_error_{0.0};
  double max_error_{1.5};
  std::chrono::milliseconds bt_loop_duration_;

  void trackingFeedbackCallback(const nav2_msgs::msg::TrackingFeedback::SharedPtr msg);
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_PATH_TRACKING_BOUNDS_CONDITION_HPP_
