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

#ifndef NAV2_ROBOT__ROS_ROBOT_HPP_
#define NAV2_ROBOT__ROS_ROBOT_HPP_

#include <string>
#include "nav2_robot/robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace nav2_robot
{

class RosRobot : public Robot
{
public:
  /**
   * Construct a RosRobot with a provided URDF file.
   *
   * @param[in] filename The filename of the URDF file describing this robot.
   */
  explicit RosRobot(rclcpp::Node *node/*, const std::string & urdf_filename*/);
  RosRobot() = delete;
  ~RosRobot();

  void enterSafeState() override;

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr getCurrentPose();

protected:
  rclcpp::Node * node_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  bool initial_pose_received_;

  void onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

}  // namespace nav2_robot

#endif  // NAV2_ROBOT__ROS_ROBOT_HPP_
