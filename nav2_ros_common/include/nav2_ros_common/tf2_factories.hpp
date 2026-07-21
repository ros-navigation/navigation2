// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROS_COMMON__TF2_FACTORIES_HPP_
#define NAV2_ROS_COMMON__TF2_FACTORIES_HPP_

#include <memory>
#include <string>

#include "rclcpp/version.h"
#include "rclcpp/rclcpp.hpp"

// Lyrical and newer deprecate the .h headers in favor of .hpp
#if RCLCPP_VERSION_GTE(30, 0, 0)
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/message_filter.hpp"
#else
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/message_filter.h"
#endif

namespace nav2
{

/**
 * @brief Nav2 type alias for tf2_ros::Buffer
 */
using TransformBuffer = tf2_ros::Buffer;

/**
 * @brief Nav2 type alias for tf2_ros::TransformListener
 */
using TransformListener = tf2_ros::TransformListener;

/**
 * @brief Nav2 type alias for tf2_ros::TransformBroadcaster
 */
using TransformBroadcaster = tf2_ros::TransformBroadcaster;

/**
 * @brief Nav2 type alias for tf2_ros::StaticTransformBroadcaster
 */
using StaticTransformBroadcaster = tf2_ros::StaticTransformBroadcaster;

/**
 * @brief Nav2 type alias for tf2_ros::MessageFilter
 */
template<typename MessageT>
using MessageFilter = tf2_ros::MessageFilter<MessageT>;

/**
 * @brief Create a transform buffer with timer interface configured
 * @param node Pointer-like node (shared_ptr or raw pointer)
 * @param callback_group Optional callback group for the timer interface
 * @return Shared pointer to the configured TransformBuffer
 */
template<typename NodeT>
inline std::shared_ptr<nav2::TransformBuffer> create_transform_buffer(
  const NodeT & node,
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  auto buffer = std::make_shared<nav2::TransformBuffer>(node->get_clock());
#if RCLCPP_VERSION_GTE(30, 0, 0)
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(*node, callback_group);
#else
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(), node->get_node_timers_interface(), callback_group);
#endif
  buffer->setCreateTimerInterface(timer_interface);
  return buffer;
}

/**
 * @brief Create a transform broadcaster
 * @param node Pointer-like node (shared_ptr or raw pointer)
 * @return Shared pointer to the TransformBroadcaster
 */
template<typename NodeT>
inline std::shared_ptr<nav2::TransformBroadcaster> create_transform_broadcaster(
  const NodeT & node)
{
#if RCLCPP_VERSION_GTE(30, 0, 0)
  return std::make_shared<nav2::TransformBroadcaster>(*node);
#else
  return std::make_shared<nav2::TransformBroadcaster>(node);
#endif
}

/**
 * @brief Create a static transform broadcaster
 * @param node Pointer-like node (shared_ptr or raw pointer)
 * @return Shared pointer to the StaticTransformBroadcaster
 */
template<typename NodeT>
inline std::shared_ptr<nav2::StaticTransformBroadcaster> create_static_transform_broadcaster(
  const NodeT & node)
{
#if RCLCPP_VERSION_GTE(30, 0, 0)
  return std::make_shared<nav2::StaticTransformBroadcaster>(*node);
#else
  return std::make_shared<nav2::StaticTransformBroadcaster>(node);
#endif
}

/**
 * @brief Create a transform listener
 * @param buffer Transform buffer reference
 * @param node Pointer-like node (shared_ptr or raw pointer)
 * @param spin_thread Whether to spin a dedicated thread
 * @return Shared pointer to the TransformListener
 */
template<typename NodeT>
inline std::shared_ptr<nav2::TransformListener> create_transform_listener(
  nav2::TransformBuffer & buffer, const NodeT & node, bool spin_thread = true)
{
  return std::make_shared<nav2::TransformListener>(buffer, node, spin_thread);
}

/**
 * @brief Create a message filter for transform-aware message subscription
 * @param sub Subscriber to filter
 * @param buffer Transform buffer reference
 * @param target_frame Target frame for transforms
 * @param queue_size Queue size for the filter
 * @param node Pointer-like node (shared_ptr or raw pointer)
 * @param tolerance Transform tolerance duration
 * @return Shared pointer to the MessageFilter
 */
template<typename MessageT, typename SubscriberT, typename NodeT>
inline std::shared_ptr<nav2::MessageFilter<MessageT>> create_message_filter(
  SubscriberT & sub, nav2::TransformBuffer & buffer,
  const std::string & target_frame, uint32_t queue_size,
  const NodeT & node, tf2::Duration tolerance)
{
#if RCLCPP_VERSION_GTE(30, 0, 0)
  return std::make_shared<nav2::MessageFilter<MessageT>>(
    sub, buffer, target_frame, queue_size, *node, tolerance);
#else
  return std::make_shared<nav2::MessageFilter<MessageT>>(
    sub, buffer, target_frame, queue_size,
    node->get_node_logging_interface(), node->get_node_clock_interface(), tolerance);
#endif
}

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__TF2_FACTORIES_HPP_
