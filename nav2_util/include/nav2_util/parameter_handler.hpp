// Copyright (c) 2025 Maurice Alexander Purnawan
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

#ifndef NAV2_UTIL__PARAMETER_HANDLER_HPP_
#define NAV2_UTIL__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_util
{

/**
 * @class ParameterHandler
 * Handles parameters and dynamic parameters
 */
template<typename ParamsT>
class ParameterHandler
{
public:
  /**
   * @brief Constructor for nav2_util::ParameterHandler
   */
  ParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    rclcpp::Logger & logger)
  : node_(node), logger_(logger) {}

  /**
   * @brief Destructor for nav2_util::ParameterHandler
   */
  virtual ~ParameterHandler() = default;

  /**
   * @brief Get the internal mutex used for thread-safe parameter access.
   * @return Reference to the mutex.
   */
  std::mutex & getMutex() {return mutex_;}

  /**
   * @brief Get a pointer to the internal parameter structure.
   * @return Pointer to the stored parameter structure.
   */
  ParamsT * getParams() {return &params_;}

  /**
  * @brief Registers callbacks for dynamic parameter handling.
  */
  void activate()
  {
    auto node = node_.lock();
    post_set_params_handler_ = node->add_post_set_parameters_callback(
      std::bind(&ParameterHandler::updateParametersCallback, this, std::placeholders::_1));
    on_set_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&ParameterHandler::validateParameterUpdatesCallback, this, std::placeholders::_1));
  }

  /**
  * @brief Resets callbacks for dynamic parameter handling.
  */
  void deactivate()
  {
    auto node = node_.lock();
    if (post_set_params_handler_ && node) {
      node->remove_post_set_parameters_callback(post_set_params_handler_.get());
    }
    if (on_set_params_handler_ && node) {
      node->remove_on_set_parameters_callback(on_set_params_handler_.get());
    }
    post_set_params_handler_.reset();
    on_set_params_handler_.reset();
  }

protected:
  nav2::LifecycleNode::WeakPtr node_;
  std::mutex mutex_;
  ParamsT params_;
  rclcpp::Logger logger_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  virtual rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters) = 0;

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  virtual void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) = 0;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__PARAMETER_HANDLER_HPP_
