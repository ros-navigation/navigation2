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
#include "behaviortree_cpp/condition_node.h"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace nav2_behavior_tree
{

class GoalReachedCondition : public BT::ConditionNode
{
public:
  explicit GoalReachedCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name), initialized_(false)
  {
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
    node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");
    node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(),
      node_->get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    initialized_ = true;
  }

  bool
  isGoalReached()
  {
    geometry_msgs::msg::PoseStamped current_pose;

    rclcpp::spin_some(node_);
    if (!nav2_util::getCurrentPose(current_pose, *tf_)) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return false;
    }
    // TODO(mhpanah): replace this with a function
    blackboard()->get<geometry_msgs::msg::PoseStamped::SharedPtr>("goal", goal_);
    double dx = goal_->pose.position.x - current_pose.pose.position.x;
    double dy = goal_->pose.position.y - current_pose.pose.position.y;

    if ( (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_) ) {
      return true;
    } else {
      return false;
    }
  }

protected:
  void cleanup()
  {
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;

  bool initialized_;
  double goal_reached_tol_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__GOAL_REACHED_CONDITION_HPP_
