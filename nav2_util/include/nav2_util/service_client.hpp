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

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

// Create a service client with a given type from node interfaces.
// TODO(orduno) rclcpp is missing a similar method, remove once it's added:
//              https://github.com/ros2/rclcpp/issues/768
template<typename ServiceT>
typename std::shared_ptr<rclcpp::Client<ServiceT>>
create_client(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph,
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
  std::shared_ptr<rclcpp::callback_group::CallbackGroup> group = nullptr)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;

  using rclcpp::Client;
  using rclcpp::ClientBase;

  auto cli = Client<ServiceT>::make_shared(
    node_base.get(),
    node_graph,
    // TODO(orduno) Extend name with sub-namespace, see rclcpp::Node::create_client()
    service_name,
    options);

  auto cli_base_ptr = std::dynamic_pointer_cast<ClientBase>(cli);
  node_services->add_client(cli_base_ptr, group);
  return cli;
}

template<class ServiceT>
class ServiceClient
{
public:
  // When possible, provide a spinning node to avoid creating additional internal nodes
  template<typename NodeT = rclcpp::Node>
  explicit ServiceClient(
    const std::string & service_name,
    const std::shared_ptr<NodeT> & provided_node = std::shared_ptr<NodeT>(),
    std::shared_ptr<rclcpp::callback_group::CallbackGroup> group = nullptr)
  : service_name_(service_name)
  {
    if (provided_node) {
      set_interfaces_from_node<NodeT>(provided_node);
    } else {
      // TODO(orduno) Remove usage of internally generated nodes
      RCLCPP_WARN(
        node_logging_->get_logger(), "%s service client: using an internally generated node",
        service_name_.c_str());
      auto node = generate_internal_node(service_name + "_Node");
      set_interfaces_from_node<rclcpp::Node>(generate_internal_node(service_name + "_Node"));
    }

    client_ = nav2_util::create_client<ServiceT>(
      node_base_, node_graph_, node_services_, service_name,
      rmw_qos_profile_services_default, group);
  }

  // TODO(orduno) Remove this constructor which relies on generating an internal node
  [[deprecated("use alternative constructors")]]
  explicit ServiceClient(const std::string & service_name, const std::string & parent_name)
  : service_name_(service_name)
  {
    auto options = rclcpp::NodeOptions().arguments(
      {"__node:=" + parent_name + std::string("_") + service_name +
        "_client"});
    auto node = rclcpp::Node::make_shared("_", options);

    set_interfaces_from_node<rclcpp::Node>(node);

    client_ = nav2_util::create_client<ServiceT>(
      node_base_, node_graph_, node_services_, service_name);
  }

  ServiceClient() = delete;

  template<typename NodeT>
  void set_interfaces_from_node(const std::shared_ptr<NodeT> & node)
  {
    node_base_ = node->get_node_base_interface();
    node_graph_ = node->get_node_graph_interface();
    node_services_ = node->get_node_services_interface();
    node_logging_ = node->get_node_logging_interface();
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_logging_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(node_logging_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (!nodeIsSpinable()) {
      auto status = future_result.wait_for(timeout);
      if (status != std::future_status::ready) {
        throw std::runtime_error(service_name_ + " service client: async_send_request failed");
      }
    } else {
      if (rclcpp::spin_until_future_complete(node_base_, future_result, timeout) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        throw std::runtime_error(service_name_ + " service client: async_send_request failed");
      }
    }

    return future_result.get();
  }

  bool invoke(
    typename RequestType::SharedPtr & request,
    typename ResponseType::SharedPtr & response,
    const std::chrono::seconds timeout = std::chrono::seconds::max())
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_logging_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(node_logging_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (!nodeIsSpinable()) {
      auto status = future_result.wait_for(timeout);
      if (status != std::future_status::ready) {
        return false;
      }
    } else {
      if (rclcpp::spin_until_future_complete(node_base_, future_result, timeout) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        return false;
      }
    }

    response = future_result.get();
    return response.get();
  }

  void wait_for_service(
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max()) const
  {
    while (!client_->wait_for_service(timeout)) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
    }
  }

  bool nodeIsSpinable() const
  {
    // Nodes can only be added to one executor. If it hasn't, then it can be spinned
    return !static_cast<bool>(node_base_->get_associated_with_executor_atomic());
  }

protected:
  std::string service_name_;

  // Interfaces used for logging and creating the service client
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_;
  std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_;
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_;
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_;

  typename std::shared_ptr<rclcpp::Client<ServiceT>> client_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_
