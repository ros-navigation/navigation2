// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef TRANSFORM_HANDLER_HPP_
#define TRANSFORM_HANDLER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/node_thread.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{
class TransformHandler : public rclcpp::Node
{
public:
  TransformHandler();
  ~TransformHandler();

  // Activate the tester before running tests
  void activate();
  void deactivate();

  std::shared_ptr<tf2_ros::Buffer> getBuffer()
  {
    return tf_buffer_;
  }

  void waitForTransform();

  void updateRobotPose(const geometry_msgs::msg::Pose & pose);

private:
  void publishRobotTransform();
  void startRobotTransform();

  bool is_active_;

  // A thread for spinning the ROS node
  std::unique_ptr<nav2_util::NodeThread> spin_thread_;

  // Subscriber

  // The tester must provide the robot pose through a transform
  std::unique_ptr<geometry_msgs::msg::TransformStamped> base_transform_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
};

}  // namespace nav2_behavior_tree

#endif  // TRANSFORM_HANDLER_HPP_
