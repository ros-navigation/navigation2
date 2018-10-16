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

#ifndef NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_
#define NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_

#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace BT
{

// Custom type
struct Pose2D
{
    double x,y,theta;
};

// This template specialization is required to be able to automatically convert 
// a NodeParameter into a Pose2D. In other words, to be able to do this:
//
//   TreeNode::getParam<Pose2D>(key, ...)
//
template <> 
inline Pose2D convertFromString(const std::string & key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid input)");
  } else {
    Pose2D output;
    output.x     = BT::convertFromString<double>(parts[0]);
    output.y     = BT::convertFromString<double>(parts[1]);
    output.theta = BT::convertFromString<double>(parts[2]);
    return output;
  }
}

template <> 
inline geometry_msgs::msg::Point convertFromString(const std::string & key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
      throw std::runtime_error("invalid input)");
  } else {
    geometry_msgs::msg::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

template <> 
inline geometry_msgs::msg::Quaternion convertFromString(const std::string & key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
      throw std::runtime_error("invalid input)");
  } else {
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

}

namespace nav2_tasks
{

class NavigateToPoseAction: public BtActionNode<NavigateToPoseCommand, NavigateToPoseResult>
{
public:
  NavigateToPoseAction(const std::string & action_name, const BT::NodeParameters & params)
  : BtActionNode<NavigateToPoseCommand, NavigateToPoseResult>(action_name, params)
  {
    // Create the input and output messages
    command_ = std::make_shared<nav2_tasks::NavigateToPoseCommand>();
    result_ = std::make_shared<nav2_tasks::NavigateToPoseResult>();

    // Use the position and orientation fields from the behavior tree node parameter
    geometry_msgs::msg::Point position;
    bool position_passed = getParam<geometry_msgs::msg::Point>("position", position);

    geometry_msgs::msg::Quaternion orientation;
    bool orientation_passed = getParam<geometry_msgs::msg::Quaternion>("orientation", orientation);

    // TODO(mjeronimo): use asserts
    if (!position_passed || !orientation_passed)
      printf("foobar\n"); // RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseAction: position or orientation not provided");

    command_->pose.position = position;
    command_->pose.orientation = orientation;
  }

  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {{"goal","0;0;0"}};
    return params;
  }

private:
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_
