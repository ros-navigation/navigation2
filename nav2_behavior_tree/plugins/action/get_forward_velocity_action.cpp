// Copyright (c) 2026
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

#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/get_forward_velocity_action.hpp"

namespace nav2_behavior_tree
{

GetForwardVelocity::GetForwardVelocity(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  odom_smoother_ = config.blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>(
    "odom_smoother");
}

BT::NodeStatus GetForwardVelocity::tick()
{
  double max_odom_age;
  if (!getInput("max_odom_age", max_odom_age) || !std::isfinite(max_odom_age) ||
    max_odom_age <= 0.0 || !odom_smoother_)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto odom = odom_smoother_->getRawTwistStamped();
  if ((odom.header.stamp.sec == 0 && odom.header.stamp.nanosec == 0) ||
    !std::isfinite(odom.twist.linear.x))
  {
    return BT::NodeStatus::FAILURE;
  }
  const rclcpp::Time stamp(odom.header.stamp, node_->get_clock()->get_clock_type());
  const double age = (node_->now() - stamp).seconds();
  if (age < 0.0 || age > max_odom_age) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("forward_velocity", odom.twist.linear.x);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetForwardVelocity>("GetForwardVelocity");
}
