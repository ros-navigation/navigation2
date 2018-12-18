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

#ifndef NAV2_TASKS__IS_LOCALIZED_CONDITION_HPP_
#define NAV2_TASKS__IS_LOCALIZED_CONDITION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "nav2_robot/robot.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace nav2_tasks
{

class IsLocalizedCondition : public BT::ConditionNode
{
public:
  explicit IsLocalizedCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name), initialized_(false)
  {
  }

  IsLocalizedCondition() = delete;

  ~IsLocalizedCondition()
  {
  }

  virtual void onInit() {}

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      // Get the required items from the blackboard
      node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");

      node_loop_timeout_ =
        blackboard()->template get<std::chrono::milliseconds>("node_loop_timeout");

      robot_ = std::make_unique<nav2_robot::Robot>(node_);

      onInit();
      initialized_ = true;
    }

    if (isLocalized()) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;

  }


  bool
  isLocalized()
  {
    auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    if (!robot_->getCurrentPose(current_pose)) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return false;
    }

    // Naive way to check if the robot has been localized
    // TODO(mhpanah): come up with a method to properly check particles convergence
    if (current_pose->pose.covariance[0] < 0.25 &&
      current_pose->pose.covariance[7] < 0.25 &&
      current_pose->pose.covariance[35] < M_PI / 4)
    {
      RCLCPP_INFO(node_->get_logger(), "AutoLocalization Passed!");
      blackboard()->set<bool>("initial_pose", true);
      return true;
    }

    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<nav2_robot::Robot> robot_;
  std::chrono::milliseconds node_loop_timeout_;
  bool initialized_;

};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_LOCALIZED_CONDITION_HPP_
