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
#include <chrono>
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::ServiceClient
 * @brief A simple wrapper on ROS2 services client
 */
template<class ServiceT, typename NodeT = rclcpp::Node::SharedPtr>
class ServiceClient
{
public:
  /**
  * @brief A constructor
  * @param service_name name of the service to call
  * @param provided_node Node to create the service client off of
  * @param use_internal_executor Whether to create an internal executor or not
  */
  explicit ServiceClient(
    const std::string & service_name,
    const NodeT & provided_node, bool use_internal_executor = false)
  : service_name_(service_name), node_(provided_node), use_internal_executor_(use_internal_executor)
  {
    if (use_internal_executor) {
      callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
      callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      callback_group_executor_->add_callback_group(callback_group_,
          node_->get_node_base_interface());
    }
    // When a nullptr is passed, the client will use the default callback group
    client_ = node_->template create_client<ServiceT>(
      service_name,
      rclcpp::SystemDefaultsQoS(),
      callback_group_);
    rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    if (!node_->has_parameter("service_introspection_mode")) {
      node_->declare_parameter("service_introspection_mode", "disabled");
    }
    std::string service_introspection_mode =
      node_->get_parameter("service_introspection_mode").as_string();
    if (service_introspection_mode == "metadata") {
      introspection_state = RCL_SERVICE_INTROSPECTION_METADATA;
    } else if (service_introspection_mode == "contents") {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    }

    this->client_->configure_introspection(
        node_->get_clock(), rclcpp::SystemDefaultsQoS(), introspection_state);
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;
  using SharedPtr = std::shared_ptr<ServiceClient<ServiceT, NodeT>>;

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
    if (use_internal_executor_) {
      if (callback_group_executor_->spin_until_future_complete(future_result, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        // Pending request must be manually cleaned up if execution is interrupted or timed out
        client_->remove_pending_request(future_result);
        throw std::runtime_error(service_name_ + " service client: async_send_request failed");
      }
    } else {
      if (rclcpp::spin_until_future_complete(node_, future_result, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        // Pending request must be manually cleaned up if execution is interrupted or timed out
        client_->remove_pending_request(future_result);
        throw std::runtime_error(service_name_ + " service client: async_send_request failed");
      }
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
    if(use_internal_executor_) {
      if (callback_group_executor_->spin_until_future_complete(future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
        client_->remove_pending_request(future_result);
        return false;
      }
    } else {
      if (rclcpp::spin_until_future_complete(node_, future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        // Pending request must be manually cleaned up if execution is interrupted or timed out
        client_->remove_pending_request(future_result);
        return false;
      }
    }
    response = future_result.get();
    return response.get();
  }

  /**
  * @brief Asynchronously call the service
  * @param request The request object to call the service using
  * @return std::shared_future<typename ResponseType::SharedPtr> The shared future of the service response
  */
  std::shared_future<typename ResponseType::SharedPtr> async_call(
    typename RequestType::SharedPtr & request)
  {
    auto future_result = client_->async_send_request(request);
    return future_result.share();
  }


  /**
  * @brief Asynchronously call the service with a callback
  * @param request The request object to call the service using
  * @param callback The callback to call when the service response is received
  */
  template<typename CallbackT>
  void async_call(typename RequestType::SharedPtr request, CallbackT && callback)
  {
    client_->async_send_request(request, callback);
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

  /**
   * @brief Spins the executor until the provided future is complete or the timeout is reached.
   *
   * @param future The shared future to wait for completion.
   * @param timeout The maximum time to wait for the future to complete. Default is -1 (no timeout).
   * @return rclcpp::FutureReturnCode indicating the result of the spin operation.
   */
  template<typename FutureT>
  rclcpp::FutureReturnCode spin_until_complete(
    std::shared_future<FutureT> future,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
  {
    // return callback_group_executor_->spin_until_future_complete(future, timeout);
    if(use_internal_executor_) {
      return callback_group_executor_->spin_until_future_complete(future, timeout);
    } else {
      return rclcpp::spin_until_future_complete(node_, future, timeout);
    }
  }

  /**
  * @brief Gets the service name
  * @return string Service name
  */
  std::string getServiceName()
  {
    return service_name_;
  }

protected:
  std::string service_name_;
  NodeT node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  bool use_internal_executor_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_
