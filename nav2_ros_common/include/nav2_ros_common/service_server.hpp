// Copyright (c) 2026 Maurice Alexander Purnawan
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
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"
#include "nav2_ros_common/qos_profiles.hpp"

namespace nav2
{

/**
 * @class nav2::ServiceServer
 * @brief A simple wrapper on ROS2 services server
 */
template<class ServiceT>
class ServiceServer : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ServiceServer)

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;
  using CallbackType = std::function<void (const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<RequestType>, std::shared_ptr<ResponseType>)>;
  // using SharedPtr = std::shared_ptr<ServiceServer<ServiceT>>;
  // using UniquePtr = std::unique_ptr<ServiceServer<ServiceT>>;

  template<typename NodeT>
  explicit ServiceServer(
    const std::string & service_name,
    const NodeT & node,
    CallbackType callback,
    // const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : service_name_(service_name),
    callback_(callback),
    clock_(node->get_clock()),
    logger_(node->get_logger())
  {
    server_ = rclcpp::create_service<ServiceT>(
      node->get_node_base_interface(),
      node->get_node_services_interface(),
      service_name,
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<RequestType> request, std::shared_ptr<ResponseType> response) {
        this->handle_service(request_header, request, response);
      },
      rclcpp::ServicesQoS(),  // Use consistent QoS settings
      callback_group);

    nav2::setIntrospectionMode(
      this->server_,
      node->get_node_parameters_interface(), node->get_clock());
  }

  void on_activate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_activate();
  }

  void on_deactivate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
  }

protected:
  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<RequestType> request,
    std::shared_ptr<ResponseType> response)
  {
    if (!this->is_activated()) {
      RCLCPP_WARN_THROTTLE(
          logger_,
          *clock_,
          1000,
          "Service '%s' called while not activate .",
          service_name_.c_str());
      return;
    }

    callback_(request_header, request, response);
  }
  std::string service_name_;
  CallbackType callback_;
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
};

}  // namespace nav2


#endif  // NAV2_ROS_COMMON__SERVICE_SERVER_HPP_
