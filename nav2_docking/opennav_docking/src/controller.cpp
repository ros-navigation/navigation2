// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2026 Karinca Robotics
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

#include <memory>
#include <string>
#include <utility>

#include "opennav_docking/controller.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace opennav_docking
{

Controller::Controller(
  const nav2::LifecycleNode::SharedPtr & node, nav2::TransformBuffer::SharedPtr tf,
  std::string fixed_frame, std::string base_frame)
{
  // Seed the frames given into this instance's parameter namespace, so that
  // configure() resolves them. Workaround to keep Controller class usable
  // in its call sites.
  nav2::declare_parameter_if_not_declared(
    node, "controller.fixed_frame", rclcpp::ParameterValue(fixed_frame));
  nav2::declare_parameter_if_not_declared(
    node, "controller.base_frame", rclcpp::ParameterValue(base_frame));

  configure(node, "controller", tf);
}

Controller::~Controller()
{
  control_law_.reset();
  trajectory_pub_.reset();
  collision_checker_.reset();
  costmap_sub_.reset();
  footprint_sub_.reset();
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd, bool is_docking,
  bool backward)
{
  nav_msgs::msg::Path trajectory;
  trajectory.header.frame_id = base_frame_;

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = base_frame_;
  target.pose = pose;
  trajectory.poses.push_back(target);

  TrajectoryOptions options;
  options.reverse = backward;
  options.approaching = is_docking;
  setTrajectory(trajectory, options);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  return computeVelocityCommands(robot_pose, geometry_msgs::msg::Twist(), 0.0, cmd);
}

}  // namespace opennav_docking
