// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/goal_updater_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

GoalUpdater::GoalUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string goal_updater_topic;
  node->get_parameter_or<std::string>("goal_updater_topic", goal_updater_topic, "goal_update");

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_updater_topic, 10, std::bind(&GoalUpdater::callback_updated_goal, this, _1));
}

inline BT::NodeStatus GoalUpdater::tick()
{
  geometry_msgs::msg::PoseStamped goal;

  getInput("input_goal", goal);

  if (rclcpp::Time(last_goal_received_.header.stamp) > rclcpp::Time(goal.header.stamp)) {
    goal = last_goal_received_;
  }

  setOutput("output_goal", goal);
  return child_node_->executeTick();
}

void
GoalUpdater::callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_goal_received_ = *msg;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdater>("GoalUpdater");
}
