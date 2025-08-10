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

#ifndef NAV2_ROS_COMMON__SERVICE_SERVER_HPP_
#define NAV2_ROS_COMMON__SERVICE_SERVER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2
{

/**
 * @class nav2::ServiceServer
 * @brief A simple wrapper on ROS2 services server
 */
template<class ServiceT>
class ServiceServer
{
public:
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;
  using CallbackType = std::function<void(const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<RequestType>, std::shared_ptr<ResponseType>)>;
  using SharedPtr = std::shared_ptr<ServiceServer<ServiceT>>;
  using UniquePtr = std::unique_ptr<ServiceServer<ServiceT>>;

  template<typename NodeT>
  explicit ServiceServer(
    const std::string & service_name,
    const NodeT & node,
    CallbackType callback,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : service_name_(service_name), callback_(callback)
  {
    server_ = rclcpp::create_service<ServiceT>(
      node->get_node_base_interface(),
      node->get_node_services_interface(),
      service_name,
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<RequestType> request, std::shared_ptr<ResponseType> response) {
        this->callback_(request_header, request, response);
      },
      rclcpp::ServicesQoS(),  // Use consistent QoS settings
      callback_group);

    nav2::setIntrospectionMode(this->server_,
      node->get_node_parameters_interface(), node->get_clock());
  }

protected:
  std::string service_name_;
  CallbackType callback_;
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
};

}  // namespace nav2


#endif  // NAV2_ROS_COMMON__SERVICE_SERVER_HPP_
