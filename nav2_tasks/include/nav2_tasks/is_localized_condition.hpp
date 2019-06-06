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

#include <string>
#include <memory>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_robot/robot.hpp"
#include "rclcpp/rclcpp.hpp"

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
    cleanup();
  }

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      initialize();
    }

    return isLocalized() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void initialize()
  {
    node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");
    node_->get_parameter_or<double>("is_localized_condition.x_tol", x_tol_, 0.25);
    node_->get_parameter_or<double>("is_localized_condition.y_tol", y_tol_, 0.25);
    node_->get_parameter_or<double>("is_localized_condition.rot_tol", rot_tol_, M_PI / 4);
    robot_ = std::make_unique<nav2_robot::Robot>(
      node_->get_node_base_interface(),
      node_->get_node_topics_interface(),
      node_->get_node_logging_interface(),
      true);
    initialized_ = true;
  }

  bool isLocalized()
  {
    auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    rclcpp::spin_some(node_);
    if (!robot_->getCurrentPose(current_pose)) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return false;
    }

    // Naive way to check if the robot has been localized
    // TODO(mhpanah): come up with a method to properly check particles convergence
    if (current_pose->pose.covariance[cov_x_] < x_tol_ &&
      current_pose->pose.covariance[cov_y_] < y_tol_ &&
      current_pose->pose.covariance[cov_a_] < rot_tol_)
    {
      RCLCPP_INFO(node_->get_logger(), "AutoLocalization Passed!");
      blackboard()->set<bool>("initial_pose_received", true);  // NOLINT
      return true;
    }

    return false;
  }

protected:
  void cleanup()
  {
    robot_.reset();
  }

private:
  static const int cov_x_ = 0;
  static const int cov_y_ = 7;
  static const int cov_a_ = 35;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<nav2_robot::Robot> robot_;

  bool initialized_;
  double x_tol_;
  double y_tol_;
  double rot_tol_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_LOCALIZED_CONDITION_HPP_
