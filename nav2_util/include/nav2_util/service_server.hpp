// Copyright (c) 2025 Maurice
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

#ifndef NAV2_UTIL__SERVICE_SERVER_HPP_
#define NAV2_UTIL__SERVICE_SERVER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::ServiceServer
 * @brief A simple wrapper on ROS2 services for invoke() and block-style calling
 */
template<class ServiceT, typename NodeT = rclcpp::Node::SharedPtr>
class ServiceServer
{
public:
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;
  using CallbackType = std::function<void(const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<RequestType>, std::shared_ptr<ResponseType>)>;

  explicit ServiceServer(
    const std::string & service_name,
    std::string service_introspection_mode,
    const NodeT & node,
    CallbackType callback,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : service_name_(service_name), callback_(callback)
  {
    server_ = node->template create_service<ServiceT>(
      service_name,
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<RequestType> request, std::shared_ptr<ResponseType> response) {
        this->callback_(request_header, request, response);
      },
      qos,
      callback_group);

    rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    if (service_introspection_mode == "disabled") {
      introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    } else if (service_introspection_mode == "metadata") {
      introspection_state = RCL_SERVICE_INTROSPECTION_METADATA;
    } else if (service_introspection_mode == "contents") {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    }

    this->server_->configure_introspection(
    node->get_clock(), rclcpp::SystemDefaultsQoS(), introspection_state);
  }

protected:
  std::string service_name_;
  CallbackType callback_;
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
};

}  // namespace nav2_util


#endif  // NAV2_UTIL__SERVICE_SERVER_HPP_
