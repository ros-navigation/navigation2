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
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

template<class ServiceT>
class ServiceClient
{
public:
  explicit ServiceClient(
    const std::string & name,
    const rclcpp::Node::SharedPtr & provided_node = rclcpp::Node::SharedPtr())
  {
    if (provided_node) {
      node_ = provided_node;
    } else {
      node_ = generate_internal_node(name + "_Node");
    }
    client_ = node_->create_client<ServiceT>(name);
  }

  ServiceClient(const std::string & parent_name, const std::string & service_name)
  {
    node_ = rclcpp::Node::make_shared(parent_name + std::string("_") + service_name + "_client");
    client_ = node_->create_client<ServiceT>(service_name);
  }

  ServiceClient(rclcpp::Node::SharedPtr node, const std::string & service_name)
  : node_(node)
  {
    client_ = node_->create_client<ServiceT>(service_name);
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::seconds timeout = std::chrono::seconds::max())
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("ServiceClient: service call interrupted while waiting for service");
      }
      RCLCPP_DEBUG(node_->get_logger(), "Waiting for service to appear...");
	  rclcpp::spin_some(node_);
    }

    RCLCPP_INFO(node_->get_logger(), "send async request");
    auto future_result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future_result, timeout) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("ServiceClient::async_send_request: service call failed");
    }

    return future_result.get();
  }

  bool invoke(
    typename RequestType::SharedPtr & request,
    typename ResponseType::SharedPtr & response)
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("ServiceClient: service call interrupted while waiting for service");
      }
      RCLCPP_DEBUG(node_->get_logger(), "Waiting for service to appear...");
    }

    RCLCPP_INFO(node_->get_logger(), "send async request");
    auto future_result = client_->async_send_request(request);

    rclcpp::executor::FutureReturnCode status = rclcpp::executor::FutureReturnCode::TIMEOUT;
    do {
      RCLCPP_INFO(node_->get_logger(), "calling spin until future complete");
      status = rclcpp::spin_until_future_complete(node_, future_result, std::chrono::milliseconds(500));
    } while (rclcpp::ok() && status != rclcpp::executor::FutureReturnCode::SUCCESS);

    RCLCPP_INFO(node_->get_logger(), "service call succeeded");
    response = future_result.get();

    return true;
  }

  void wait_for_service(const std::chrono::seconds timeout = std::chrono::seconds::max())
  {
    while (!client_->wait_for_service(timeout)) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("waitForServer: interrupted while waiting for service to appear");
      }
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace nav2_tasks

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_
