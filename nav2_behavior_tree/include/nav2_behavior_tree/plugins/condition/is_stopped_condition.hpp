// Copyright (c) 2024 Angsa Robotics
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STOPPED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STOPPED_CONDITION_HPP_

#include <string>
#include <atomic>
#include <deque>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"


using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that tracks robot odometry and returns SUCCESS
 * if robot is considered stopped for long enough, RUNNING if stopped but not for long enough and FAILURE otherwise
 */
class IsStoppedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsStoppedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsStoppedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsStoppedCondition() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::IsStoppedCondition
   */
  ~IsStoppedCondition() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("velocity_threshold", 0.01,
          "Velocity threshold below which robot is considered stopped"),
      BT::InputPort<std::chrono::milliseconds>("duration_stopped", 1000ms,
          "Duration (ms) the velocity must remain below the threshold"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;

  double velocity_threshold_;
  std::chrono::milliseconds duration_stopped_;
  rclcpp::Time stopped_stamp_;

  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STOPPED_CONDITION_HPP_
