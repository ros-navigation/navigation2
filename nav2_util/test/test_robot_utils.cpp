// Copyright (c) 2020 Samsung Research
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
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "nav2_util/node_thread.hpp"
#include "tf2_ros/create_timer_ros.h"

TEST(RobotUtils, LookupExceptionError)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("name", rclcpp::NodeOptions());
  geometry_msgs::msg::PoseStamped global_pose;
  tf2_ros::Buffer tf(node->get_clock());
  ASSERT_FALSE(nav2_util::getCurrentPose(global_pose, tf, "map", "base_link", 0.1));
  global_pose.header.frame_id = "base_link";
  ASSERT_FALSE(nav2_util::transformPoseInTargetFrame(global_pose, global_pose, tf, "map", 0.1));
}
