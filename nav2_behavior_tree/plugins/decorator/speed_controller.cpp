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
#include <memory>

#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/speed_controller.hpp"

namespace nav2_behavior_tree
{

SpeedController::SpeedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_tick_(false),
  period_(1.0),
  min_rate_(0.1),
  max_rate_(1.0),
  min_speed_(0.0),
  max_speed_(0.5)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("min_rate", min_rate_);
  getInput("max_rate", max_rate_);
  getInput("min_speed", min_speed_);
  getInput("max_speed", max_speed_);

  if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
    std::string err_msg = "SpeedController node cannot have rate <= 0.0";
    RCLCPP_FATAL(node_->get_logger(), err_msg);
    throw BT::BehaviorTreeException(err_msg);
  }

  d_rate_ = max_rate_ - min_rate_;
  d_speed_ = max_speed_ - min_speed_;

  double duration = 0.3;
  getInput("filter_duration", duration);
  std::string odom_topic;
  node_->get_parameter_or("odom_topic", odom_topic, std::string("odom"));
  odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node_, duration, odom_topic);
}

inline BT::NodeStatus SpeedController::tick()
{
  auto current_goal = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal");

  if (goal_ != current_goal) {
    // Reset state and set period to max since we have a new goal
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
    goal_ = current_goal;
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto elapsed = node_->now() - start_;

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    elapsed.seconds() >= period_)
  {
    first_tick_ = false;

    // update period if the last period is exceeded
    if (elapsed.seconds() >= period_) {
      updatePeriod();
      start_ = node_->now();
    }

    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SpeedController>("SpeedController");
}
