// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__GOAL_REACHED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__GOAL_REACHED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

class GoalReachedCondition : public BT::ConditionNode
{
public:
  GoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    initialized_(false),
    global_frame_("map"),
    robot_base_frame_("base_link")
  {
    getInput("global_frame", global_frame_);
    getInput("robot_base_frame", robot_base_frame_);
  }

  GoalReachedCondition() = delete;

  ~GoalReachedCondition()
  {
    cleanup();
  }

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      initialize();
    }

    if (isGoalReached()) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  void initialize()
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    node_->get_parameter("transform_tolerance", transform_tolerance_);

    initialized_ = true;
  }

  bool
  isGoalReached()
  {
    geometry_msgs::msg::PoseStamped current_pose;

    if (!nav2_util::getCurrentPose(current_pose, *tf_, "map", "base_link", transform_tolerance_)) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return false;
    }

    geometry_msgs::msg::PoseStamped goal;
    getInput("goal", goal);
    double dx = goal.pose.position.x - current_pose.pose.position.x;
    double dy = goal.pose.position.y - current_pose.pose.position.y;

    return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

protected:
  void cleanup()
  {
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool initialized_;
  double goal_reached_tol_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");
}

#endif  // NAV2_BEHAVIOR_TREE__GOAL_REACHED_CONDITION_HPP_
