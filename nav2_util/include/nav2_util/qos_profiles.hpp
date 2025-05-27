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

#ifndef NAV2_UTIL__QOS_PROFILES_HPP_
#define NAV2_UTIL__QOS_PROFILES_HPP_

#include "rclcpp/rclcpp.hpp"

namespace nav2_qos
{

/**
 * @class nav2_qos::StandardTopicQoS
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
  StandardTopicQoS(const int depth = 10)
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->durability_volatile();
  };
};

/**
 * @class nav2_qos::LatchedTopicQoS
 * @brief A QoS profile for latched, reliable topics with a history of 1 messages
 */
class LatchedTopicQoS : public rclcpp::QoS
{
public:
  /**
   * @brief Constructor for LatchedTopicQoS
   * @param depth The history depth for the QoS profile, default is 1
   */
  explicit
  LatchedTopicQoS(const int depth = 1)
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->transient_local();
  };
};

/**
 * @class nav2_qos::SensorDataQoS
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
  SensorDataQoS(const int depth = 10)
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->best_effort();
    this->durability_volatile();
  };
};

}  // namespace nav2_qos

#endif  // NAV2_UTIL__QOS_PROFILES_HPP_
