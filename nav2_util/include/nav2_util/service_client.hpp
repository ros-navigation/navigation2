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

#ifndef NAV2_UTIL__SERVICE_CLIENT_HPP_
#define NAV2_UTIL__SERVICE_CLIENT_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::ServiceClient
 * @brief A simple wrapper on ROS2 services for invoke() and block-style calling
 */
template<class ServiceT>
class ServiceClient
{
public:
  /**
  * @brief A constructor
  * @param service_name name of the service to call
  * @param provided_node Node to create the service client off of
  * @param use_internal_executor Whether to create and use SingleThreadedExecutor, defalut True.
  */
  explicit ServiceClient(
    const std::string & service_name,
    const rclcpp::Node::SharedPtr & provided_node,
    bool use_internal_executor = true)
  : service_name_(service_name), node_(provided_node),
    use_internal_executor_(use_internal_executor)
  {
    if (use_internal_executor_) {
      callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
      callback_group_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      callback_group_executor_->add_callback_group(
        callback_group_,
        node_->get_node_base_interface());
    } else {
      callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    client_ = node_->create_client<ServiceT>(
      service_name,
      rmw_qos_profile_services_default,
      callback_group_);
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  /**
  * @brief Invoke the service and block until completed or timed out
  * @param request The request object to call the service using
  * @param timeout Maximum timeout to wait for, default infinite
  * @return Response A pointer to the service response from the request
  */
  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    bool success = true;
    if (use_internal_executor_) {
      auto status = callback_group_executor_->spin_until_future_complete(future_result, timeout);
      success = (status == rclcpp::FutureReturnCode::SUCCESS);
    } else {
      std::future_status status;
      if (timeout < std::chrono::nanoseconds::zero()) {
        // Block forever until future completed
        do {
          status = future_result.wait_for(std::chrono::seconds(1));
        } while (status != std::future_status::ready);
      } else {
        status = future_result.wait_for(timeout);
      }
      success = (status == std::future_status::ready);
    }
    if (!success) {
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    return future_result.get();
  }

  /**
  * @brief Invoke the service and block until completed
  * @param request The request object to call the service using
  * @param Response A pointer to the service response from the request
  * @return bool Whether it was successfully called
  */
  bool invoke(
    typename RequestType::SharedPtr & request,
    typename ResponseType::SharedPtr & response)
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    bool success = true;
    if (use_internal_executor_) {
      auto status = callback_group_executor_->spin_until_future_complete(future_result);
      success = (status == rclcpp::FutureReturnCode::SUCCESS);
    } else {
      std::future_status status;
      // Block forever until future completed
      do {
        status = future_result.wait_for(std::chrono::seconds(1));
      } while (status != std::future_status::ready);
    }

    if (!success) {
      return false;
    }

    response = future_result.get();
    return response.get();
  }

  /**
  * @brief Block until a service is available or timeout
  * @param timeout Maximum timeout to wait for, default infinite
  * @return bool true if service is available
  */
  bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    return client_->wait_for_service(timeout);
  }

protected:
  std::string service_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> callback_group_executor_;
  bool use_internal_executor_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_
