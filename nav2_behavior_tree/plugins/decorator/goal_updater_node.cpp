// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include <string>
#include <memory>

#include "behaviortree_cpp/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/goal_updater_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

GoalUpdater::GoalUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  goal_updater_topic_("goal_update"),
  goals_updater_topic_("goals_update")
{
  initialize();
}

void GoalUpdater::initialize()
{
  createROSInterfaces();
}

void GoalUpdater::createROSInterfaces()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string goal_updater_topic_new;
  std::string goals_updater_topic_new;
  node_->get_parameter_or<std::string>("goal_updater_topic", goal_updater_topic_new, "goal_update");
  node_->get_parameter_or<std::string>("goals_updater_topic", goals_updater_topic_new,
    "goals_update");

  // Only create a new subscriber if the topic has changed or subscriber is empty
  if (goal_updater_topic_new != goal_updater_topic_ || !goal_sub_) {
    goal_updater_topic_ = goal_updater_topic_new;
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_updater_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GoalUpdater::callback_updated_goal, this, _1),
      sub_option);
  }
  if (goals_updater_topic_new != goals_updater_topic_ || !goals_sub_) {
    goals_updater_topic_ = goals_updater_topic_new;
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    goals_sub_ = node_->create_subscription<nav_msgs::msg::Goals>(
      goals_updater_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GoalUpdater::callback_updated_goals, this, _1),
      sub_option);
  }
}

inline BT::NodeStatus GoalUpdater::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  geometry_msgs::msg::PoseStamped goal;
  nav_msgs::msg::Goals goals;

  getInput("input_goal", goal);
  getInput("input_goals", goals);

  // Spin multiple times due to rclcpp regression in Jazzy requiring a 'warm up' spin
  callback_group_executor_.spin_all(std::chrono::milliseconds(1));
  callback_group_executor_.spin_all(std::chrono::milliseconds(49));

  if (last_goal_received_set_) {
    if (last_goal_received_.header.stamp == rclcpp::Time(0)) {
        // if the goal doesn't have a timestamp, we reject it
      RCLCPP_WARN(
          node_->get_logger(), "The received goal has no timestamp. Ignoring.");
      setOutput("output_goal", goal);
    } else {
      auto last_goal_received_time = rclcpp::Time(last_goal_received_.header.stamp);
      auto goal_time = rclcpp::Time(goal.header.stamp);
      if (last_goal_received_time >= goal_time) {
        setOutput("output_goal", last_goal_received_);
      } else {
        RCLCPP_INFO(
            node_->get_logger(), "The timestamp of the received goal (%f) is older than the "
            "current goal (%f). Ignoring the received goal.",
            last_goal_received_time.seconds(), goal_time.seconds());
        setOutput("output_goal", goal);
      }
    }
  } else {
    setOutput("output_goal", goal);
  }

  if (last_goals_received_set_) {
    if (last_goals_received_.poses.empty()) {
      setOutput("output_goals", goals);
    } else if (last_goals_received_.header.stamp == rclcpp::Time(0)) {
      RCLCPP_WARN(
          node_->get_logger(), "The received goals array has no timestamp. Ignoring.");
      setOutput("output_goals", goals);
    } else {
      auto last_goals_received_time = rclcpp::Time(last_goals_received_.header.stamp);
      auto goals_time = rclcpp::Time(goals.header.stamp);
      if (last_goals_received_time >= goals_time) {
        setOutput("output_goals", last_goals_received_);
      } else {
        RCLCPP_INFO(
            node_->get_logger(), "The timestamp of the received goals (%f) is older than the "
            "current goals (%f). Ignoring the received goals.",
            last_goals_received_time.seconds(), goals_time.seconds());
        setOutput("output_goals", goals);
      }
    }
  } else {
    setOutput("output_goals", goals);
  }

  return child_node_->executeTick();
}

void
GoalUpdater::callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_goal_received_ = *msg;
  last_goal_received_set_ = true;
}

void
GoalUpdater::callback_updated_goals(const nav_msgs::msg::Goals::SharedPtr msg)
{
  last_goals_received_ = *msg;
  last_goals_received_set_ = true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdater>("GoalUpdater");
}
