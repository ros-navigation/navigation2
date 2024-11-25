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

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"

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
   * @brief Callback function for odom topic
   * @param msg Shared pointer to nav_msgs::msg::Odometry::SharedPtr message
   */
  void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg);

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
      BT::InputPort<double>("velocity_threshold", 0.1,
          "Velocity threshold below which robot is considered stopped"),
      BT::InputPort<std::chrono::milliseconds>("time_stopped_threshold", 1000,
          "Time threshold for which the velocity needs to be below the threshold to consider the robot stopped"),
      BT::InputPort<std::string>(
        "topic_name",
        "odom",
        "the odometry topic name"),
    };
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  std::string topic_name_;
  std::atomic<bool> is_stopped_;
  double velocity_threshold_;
  std::chrono::milliseconds time_stopped_threshold_;
  rclcpp::Time stopped_stamp_;
  std::mutex mutex_;

  // Listen to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STOPPED_CONDITION_HPP_
