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

#ifndef NAV2_ROS_COMMON__QOS_PROFILES_HPP_
#define NAV2_ROS_COMMON__QOS_PROFILES_HPP_

#include "rclcpp/rclcpp.hpp"

namespace nav2
{

namespace qos
{

/**
 * @class nav2::qos::StandardTopicQoS
 * @brief A QoS profile for standard reliable topics with a history of 10 messages
 */
class StandardTopicQoS : public rclcpp::QoS
{
public:
  /**
   * @brief Constructor for StandardTopicQoS
   * @param depth The history depth for the QoS profile, default is 10
   */
  explicit
  StandardTopicQoS(const int depth = 10)  // NOLINT
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->durability_volatile();
  }
};

/**
 * @class nav2::qos::LatchedPublisherQoS
 * @brief A QoS profile for latched, reliable topics with a history of 1 messages
 */
class LatchedPublisherQoS : public rclcpp::QoS
{
public:
  /**
   * @brief Constructor for LatchedPublisherQoS
   * @param depth The history depth for the QoS profile, default is 1
   */
  explicit
  LatchedPublisherQoS(const int depth = 1)  // NOLINT
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->transient_local();
  }
};

/**
 * @class nav2::qos::LatchedSubscriptionQoS
 * @brief A QoS profile for latched, reliable topics with a history of 1 messages
 */
class LatchedSubscriptionQoS : public rclcpp::QoS
{
public:
  /**
   * @brief Constructor for LatchedSubscriptionQoS
   * @param depth The history depth for the QoS profile, default is 1
   */
  explicit
  LatchedSubscriptionQoS(const int depth = 10)  // NOLINT
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->transient_local();
  }
};

/**
 * @class nav2::qos::SensorDataQoS
 * @brief A QoS profile for best-effort sensor data with a history of 10 messages
 */
class SensorDataQoS : public rclcpp::QoS
{
public:
  /**
   * @brief Constructor for SensorDataQoS
   * @param depth The history depth for the QoS profile, default is 10
   */
  explicit
  SensorDataQoS(const int depth = 10)  // NOLINT
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->best_effort();
    this->durability_volatile();
  }
};

}  // namespace qos

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__QOS_PROFILES_HPP_
