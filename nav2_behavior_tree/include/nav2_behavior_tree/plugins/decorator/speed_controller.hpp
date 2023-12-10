// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/odometry_utils.hpp"

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child every at a rate proportional to
 * the speed of the robot. If the robot travels faster, this node will tick its child at a
 * higher frequency and reduce the tick frequency if the robot slows down
 */
class SpeedController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpeedController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  SpeedController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_rate", 0.1, "Minimum rate"),
      BT::InputPort<double>("max_rate", 1.0, "Maximum rate"),
      BT::InputPort<double>("min_speed", 0.0, "Minimum speed"),
      BT::InputPort<double>("max_speed", 0.5, "Maximum speed"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Scale the rate based speed
   * @return double Rate scaled by speed limits and clamped
   */
  inline double getScaledRate(const double & speed)
  {
    return std::max(
      std::min(
        (((speed - min_speed_) / d_speed_) * d_rate_) + min_rate_,
        max_rate_), min_rate_);
  }

  /**
   * @brief Update period based on current smoothed speed and reset timer
   */
  inline void updatePeriod()
  {
    auto velocity = odom_smoother_->getTwist();
    double speed = std::hypot(velocity.linear.x, velocity.linear.y);
    double rate = getScaledRate(speed);
    period_ = 1.0 / rate;
  }

  rclcpp::Node::SharedPtr node_;

  // To keep track of time to reset
  rclcpp::Time start_;

  // To get a smoothed velocity
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  bool first_tick_;

  // Time period after which child node should be ticked
  double period_;

  // Rates thresholds to tick child node
  double min_rate_;
  double max_rate_;
  double d_rate_;

  // Speed thresholds
  double min_speed_;
  double max_speed_;
  double d_speed_;

  // current goal
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_
