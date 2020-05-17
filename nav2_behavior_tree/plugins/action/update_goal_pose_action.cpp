// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__UPDATE_GOAL_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__UPDATE_GOAL_POSE_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "behaviortree_cpp_v3/control_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

class UpdateGoalPoseAction : public BT::ControlNode
{
public:
  UpdateGoalPoseAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : ControlNode(xml_tag_name, conf),
    initialized_(false)
  {
  }

  UpdateGoalPoseAction() = delete;

  BT::NodeStatus tick() override
  { 
    if (!initialized_) {
      initialize();
    }

    auto current_goal = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal");
    if (rclcpp::Time(goal_.header.stamp) < rclcpp::Time(current_goal.header.stamp)) {
      goal_ = current_goal;
    }

    config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal_);

    return BT::NodeStatus::SUCCESS;
  }

  void initialize()
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    updated_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("goal_update",
      10, std::bind(&UpdateGoalPoseAction::updatedGoalCallback, this, std::placeholders::_1));

    initialized_ = true;
  }

  void updatedGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = *msg;
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {
      BT::BidirectionalPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
    };

    return ports;
  }
private:
  rclcpp::Node::SharedPtr node_;
  
  geometry_msgs::msg::PoseStamped goal_;
  bool initialized_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr updated_goal_sub_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::UpdateGoalPoseAction>("UpdateGoalPose");
}

#endif  // NAV2_BEHAVIOR_TREE__UPDATE_GOAL_POSE_ACTION_HPP_
