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
#include <deque>

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/odometry_utils.hpp"

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

class SpeedController : public BT::DecoratorNode
{
public:
  SpeedController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_rate", 0.1, "Minimum rate"),
      BT::InputPort<double>("max_rate", 1.0, "Maximum rate"),
      BT::InputPort<double>("min_speed", 0.0, "Minimum speed"),
      BT::InputPort<double>("max_speed", 0.5, "Maximum speed"),
      BT::InputPort<double>("filter_duration", 0.3, "Duration (secs) for velocity smoothing filter")
    };
  }

private:
  BT::NodeStatus tick() override;

  // Scale the rate based speed
  inline double getScaledRate(const double & speed)
  {
    return std::max(
      std::min(
        (((speed - min_speed_) / d_speed_) * d_rate_) + min_rate_,
        max_rate_), min_rate_);
  }

  // Update period based on current smoothed speed and reset timer
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
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_
