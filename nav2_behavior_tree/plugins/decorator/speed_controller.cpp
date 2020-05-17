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

#include <string>

#include "nav2_util/geometry_utils.hpp"

#include "speed_controller.hpp"

namespace nav2_behavior_tree
{

SpeedController::SpeedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false),
  period_(1.0)
{
  getInput("min_rate", min_rate_);
  getInput("max_rate", max_rate_);
  getInput("min_speed", min_speed_);
  getInput("max_speed", max_speed_);

  d_rate_ = max_rate_ - min_rate_;
  d_speed_ = max_speed_ - min_speed_;

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&SpeedController::onOdomReceived, this, std::placeholders::_1));
}

inline BT::NodeStatus SpeedController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    start_ = std::chrono::steady_clock::now();
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto now = std::chrono::steady_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::steady_clock::now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

void SpeedController::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Calculate speed using velocity components
  double speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

  // Calculate scaled time period based on current speed
  period_ = 1.0 / getScaledRate(speed);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SpeedController>("SpeedController");
}
