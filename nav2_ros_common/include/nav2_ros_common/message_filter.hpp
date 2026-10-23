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

#ifndef NAV2_ROS_COMMON__MESSAGE_FILTER_HPP_
#define NAV2_ROS_COMMON__MESSAGE_FILTER_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/version.h"
#include "nav2_ros_common/tf_manager.hpp"

#include "tf2/time.hpp"
#include "tf2_ros/message_filter.hpp"

namespace nav2
{

/**
 * @brief Nav2 type wrapper for tf2_ros::MessageFilter
 */
template<typename MessageT>
struct MessageFilter : tf2_ros::MessageFilter<MessageT>
{
  using tf2_ros::MessageFilter<MessageT>::MessageFilter;
  using SharedPtr = std::shared_ptr<MessageFilter<MessageT>>;
};

/**
 * @brief Create a message filter for transform-aware message subscription
 * @param sub Subscriber to filter
 * @param buffer Transform manager reference
 * @param target_frame Target frame for transforms
 * @param queue_size Queue size for the filter
 * @param node Pointer-like node (shared_ptr or raw pointer)
 * @param tolerance Transform tolerance duration
 * @return Shared pointer to the MessageFilter
 */
template<typename MessageT, typename SubscriberT, typename NodeT>
inline typename nav2::MessageFilter<MessageT>::SharedPtr create_message_filter(
  SubscriberT & sub, nav2::TransformManager & buffer,
  const std::string & target_frame, uint32_t queue_size,
  const NodeT & node, tf2::Duration tolerance)
{
#if RCLCPP_VERSION_GTE(30, 0, 0)
  return std::make_shared<nav2::MessageFilter<MessageT>>(
    sub, buffer.getRawBuffer(), target_frame, queue_size, *node, tolerance);
#else
  return std::make_shared<nav2::MessageFilter<MessageT>>(
    sub, buffer.getRawBuffer(), target_frame, queue_size,
    node->get_node_logging_interface(), node->get_node_clock_interface(), tolerance);
#endif
}

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__MESSAGE_FILTER_HPP_
