// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_LOOPBACK_SIM__LOOPBACK_SIMULATOR_HPP_
#define NAV2_LOOPBACK_SIM__LOOPBACK_SIMULATOR_HPP_

#pragma once
#include <iostream>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/transform_datatypes.h"

namespace nav2_loopback_sim
{
/**
 * @class nav2_planner::LoopbackSimulator
 * @brief A class that is a lightweight simulation alternative designed to facilitate
 * testing of higher-level behavioral attributes
*/
class LoopbackSimulator : public rclcpp::Node
{
public:
  /**
  * @brief A constructor for nav2_planner::LoopbackSimulator
  */
  LoopbackSimulator();

private:
  /**
  * @brief Called when velocity commands are received
  * @param msg twist velocity message
  */
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
  * @brief Called when initial pose is set by the user
  * @param msg initial pose
  */
  void initposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
  * @brief Timer function to continously broadcast map->odom tf
  */
  void timerCallback();

  // Poses
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_;  // init odom pose wrt map frame
  geometry_msgs::msg::PoseWithCovarianceStamped odom_updated_pose_;
  // Transformed init_pose_ from "map" to "odom" frame
  bool init_pose_set_ = false;
  bool init_odom_base_published_ = false;
  bool transform_initpose_once_ = true;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    init_pose_subscriber_;

  // tf
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_loopback_sim

#endif  // NAV2_LOOPBACK_SIM__LOOPBACK_SIMULATOR_HPP_
