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

namespace nav2
{

/**
 * @class nav2::ServiceServer
 * @brief A simple wrapper on ROS2 services server that integrates with Lifecycle states.
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

  /**
   * @brief Constructor for nav2::ServiceServer
   */
  template<typename NodeT>
  explicit ServiceServer(
    const std::string & service_name,
    const NodeT & node,
    CallbackType callback,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS())
  : service_name_(service_name),
    callback_(callback),
    logger_(rclcpp::get_logger("ServiceServer"))
  {
    // Resolve node pointer to handle both shared_ptr and weak_ptr cases
    auto node_shared_ptr = resolve_node(node);
    auto node_base = node_shared_ptr->get_node_base_interface();

    logger_ = rclcpp::get_logger(node_base->get_fully_qualified_name());

    server_ = rclcpp::create_service<ServiceT>(
      node_base,
      node_shared_ptr->get_node_services_interface(),
      service_name,
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<RequestType> request, std::shared_ptr<ResponseType> response) {
        this->handle_service(request_header, request, response);
      },
      qos,
      callback_group);

    nav2::setIntrospectionMode(
      this->server_,
      node_shared_ptr->get_node_parameters_interface(),
      node_shared_ptr->get_node_clock_interface()->get_clock());
  }

  /**
   * @brief Activate the service server
   */
  void activate() {this->on_activate();}

  /**
   * @brief Deactivate the service server
   */
  void deactivate() {this->on_deactivate();}

protected:
  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<RequestType> request,
    std::shared_ptr<ResponseType> response)
  {
    if (!this->is_activated()) {
      RCLCPP_DEBUG(
        logger_, "Service '%s' called while not activated. Rejecting request.",
        service_name_.c_str());
      return;
    }
    callback_(request_header, request, response);
  }

private:
  /**
   * @brief Internal utilities to resolve shared_ptr from diverse node pointer .
   */
  template<typename T>
  std::shared_ptr<T> resolve_node(std::shared_ptr<T> ptr) {return ptr;}

  template<typename T>
  std::shared_ptr<T> resolve_node(std::weak_ptr<T> ptr)
  {
    auto locked = ptr.lock();
    if (!locked) {
      throw std::runtime_error("Node expired before creating service: " + service_name_);
    }
    return locked;
  }

  std::string service_name_;
  CallbackType callback_;
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
  rclcpp::Logger logger_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__SERVICE_SERVER_HPP_
