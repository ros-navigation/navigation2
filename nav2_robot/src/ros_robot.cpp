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

#include <string>
#include <exception>
#include "urdf/model.h"
#include "nav2_robot/ros_robot.hpp"

namespace nav2_robot
{

RosRobot::RosRobot(rclcpp::Node * node)
: node_(node), initial_pose_received_(false)
{
  // Open and parser the URDF file
  if (!(urdf_file_ = std::getenv("URDF_FILE")))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get URDF_FILE environment.");
    throw std::runtime_error("Failed to read URDF file. Please make sure path environment"
    " to urdf file is set correctly." );
  }

  if (!model_.initFile(urdf_file_)){
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF file.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Parsed URDF file");
  }

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", std::bind(&RosRobot::onPoseReceived, this, std::placeholders::_1));
}

RosRobot::~RosRobot()
{
}

void
RosRobot::enterSafeState()
{
}

void
RosRobot::onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "RosRobot::onPoseReceved");

  // TODO(mjeronimo): serialize access
  current_pose_ = msg;
  initial_pose_received_ = true;
}

geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
RosRobot::getCurrentPose()
{
  // TODO(mjeronimo): better to throw an exception or return an empty result/nullptr?
  if (!initial_pose_received_) {
    throw std::runtime_error("RosRobot::getCurrentPose: initial pose not received yet");
  }

  return current_pose_;
}

}  // namespace nav2_robot
