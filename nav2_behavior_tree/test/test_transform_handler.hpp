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

#ifndef TEST_TRANSFORM_HANDLER_HPP_
#define TEST_TRANSFORM_HANDLER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <chrono>
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

using namespace std::chrono_literals; // NOLINT
using namespace std::chrono;  // NOLINT

namespace nav2_behavior_tree
{
class TransformHandler
{
public:
  explicit TransformHandler(rclcpp::Node::SharedPtr & node)
  : node_(node),
    is_active_(false),
    base_transform_(nullptr),
    tf_broadcaster_(nullptr)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  ~TransformHandler()
  {
    if (is_active_) {
      deactivate();
    }
  }

  // Activate the tester before running tests
  void activate()
  {
    if (is_active_) {
      throw std::runtime_error("Trying to activate while already active");
    }
    is_active_ = true;

    // Launch a thread to process the messages for this node
    spin_thread_ = std::make_unique<nav2_util::NodeThread>(node_->get_node_base_interface());

    startRobotTransform();
  }

  void deactivate()
  {
    if (!is_active_) {
      throw std::runtime_error("Trying to deactivate while already inactive");
    }
    is_active_ = false;
    spin_thread_.reset();
    tf_broadcaster_.reset();
    tf_buffer_.reset();
    tf_listener_.reset();
  }

  std::shared_ptr<tf2_ros::Buffer> getBuffer()
  {
    return tf_buffer_;
  }

  void waitForTransform()
  {
    if (is_active_) {
      while (!tf_buffer_->canTransform("map", "base_link", rclcpp::Time(0))) {
        std::this_thread::sleep_for(100ms);
      }
      RCLCPP_INFO(node_->get_logger(), "Transforms are available now!");
      return;
    }
    throw std::runtime_error("Trying to deactivate while already inactive");
  }

  void updateRobotPose(const geometry_msgs::msg::Pose & pose)
  {
    // Update base transform to publish
    base_transform_->transform.translation.x = pose.position.x;
    base_transform_->transform.translation.y = pose.position.y;
    base_transform_->transform.translation.z = pose.position.z;
    base_transform_->transform.rotation.x = pose.orientation.x;
    base_transform_->transform.rotation.y = pose.orientation.y;
    base_transform_->transform.rotation.z = pose.orientation.z;
    base_transform_->transform.rotation.w = pose.orientation.w;
    publishRobotTransform();
  }

private:
  void publishRobotTransform()
  {
    base_transform_->header.stamp = node_->now();
    tf_broadcaster_->sendTransform(*base_transform_);
  }

  void startRobotTransform()
  {
    // Provide the robot pose transform
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    if (!base_transform_) {
      base_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
      base_transform_->header.frame_id = "map";
      base_transform_->child_frame_id = "base_link";
    }

    // Set an initial pose
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = 0;
    robot_pose.position.y = 0;
    robot_pose.orientation.w = 1;
    updateRobotPose(robot_pose);

    // Publish the transform periodically
    transform_timer_ = node_->create_wall_timer(
      100ms, std::bind(&TransformHandler::publishRobotTransform, this));
  }

  rclcpp::Node::SharedPtr node_;

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

#endif  // TEST_TRANSFORM_HANDLER_HPP_
