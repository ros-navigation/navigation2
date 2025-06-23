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

#ifndef NAV2_ROS_COMMON__SERVICE_CLIENT_HPP_
#define NAV2_ROS_COMMON__SERVICE_CLIENT_HPP_

#include <string>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2
{

/**
 * @class nav2::ServiceClient
 * @brief A simple wrapper on ROS2 services client
 */
template<typename ServiceT>
class ServiceClient
{
public:
  using SharedPtr = std::shared_ptr<nav2::ServiceClient<ServiceT>>;
  using UniquePtr = std::unique_ptr<nav2::ServiceClient<ServiceT>>;

  /**
  * @brief A constructor
  * @param service_name name of the service to call
  * @param provided_node Node to create the service client off of
  * @param use_internal_executor Whether to create an internal executor or not
  */
  template<typename NodeT>
  explicit ServiceClient(
    const std::string & service_name,
    const NodeT & provided_node, bool use_internal_executor = false)
  : service_name_(service_name),
    clock_(provided_node->get_clock()),
    logger_(provided_node->get_logger()),
    node_base_interface_(provided_node->get_node_base_interface()),
    use_internal_executor_(use_internal_executor)
  {
    if (use_internal_executor) {
      callback_group_ = provided_node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
      callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      callback_group_executor_->add_callback_group(callback_group_,
          provided_node->get_node_base_interface());
    }
    // When a nullptr is passed, the client will use the default callback group
    client_ = rclcpp::create_client<ServiceT>(
      provided_node->get_node_base_interface(),
      provided_node->get_node_graph_interface(),
      provided_node->get_node_services_interface(),
      service_name,
      rclcpp::ServicesQoS(),  // Use consistent QoS settings
      callback_group_);

    nav2::setIntrospectionMode(this->client_,
      provided_node->get_node_parameters_interface(), clock_);
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
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1),
    const std::chrono::nanoseconds wait_for_service_timeout = std::chrono::seconds(10))
  {
    auto now = clock_->now();
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        logger_, "%s service client: waiting for service to appear...",
        service_name_.c_str());

      if (clock_->now() - now > wait_for_service_timeout) {
        throw std::runtime_error(
                service_name_ + " service client: timed out waiting for service");
      }
    }

    RCLCPP_DEBUG(
      logger_, "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);
    if (spin_until_complete(future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      client_->remove_pending_request(future_result);
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
    typename ResponseType::SharedPtr & response,
    const std::chrono::nanoseconds wait_for_service_timeout = std::chrono::seconds(10))
  {
    auto now = clock_->now();
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        logger_, "%s service client: waiting for service to appear...",
        service_name_.c_str());

      if (clock_->now() - now > wait_for_service_timeout) {
        throw std::runtime_error(
                service_name_ + " service client: timed out waiting for service");
      }
    }

    RCLCPP_DEBUG(
      logger_, "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);
    if (spin_until_complete(future_result) != rclcpp::FutureReturnCode::SUCCESS) {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      client_->remove_pending_request(future_result);
      return false;
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
    const FutureT & future,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
  {
    if (use_internal_executor_) {
      return callback_group_executor_->spin_until_future_complete(future, timeout);
    } else {
      return rclcpp::spin_until_future_complete(node_base_interface_, future, timeout);
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
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_ros_common")};
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  bool use_internal_executor_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__SERVICE_CLIENT_HPP_
