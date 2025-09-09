// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROUTE__PLUGINS__ROUTE_OPERATION_CLIENT_HPP_
#define NAV2_ROUTE__PLUGINS__ROUTE_OPERATION_CLIENT_HPP_

#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_route/interfaces/route_operation.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/service_client.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_route
{

using namespace std::chrono_literals;  // NOLINT

/**
 * @class RouteOperationClient
 * @brief A route operation base class to trigger a service event at a node or edge.
 * This is meant to be named accordingly to the event of query in the parameter file
 * (e.g. OpenDoor, CallElevator). Thus, a single RouteOperationClient implementation can
 * support many different operation instances calling different services. The template type
 * may be overridden for any applications. It may be set up to either trigger a
 * single service if the `service_name` is set in the parameter file at launch for all calls
 * **or** trigger different services depending on the `service_name` set in the metadata of the
 * node or edge operation given.
 *
 * For example: OpenDoor<nav2_msgs::srv::OpenDoor>
 * RouteOperationClient instance can open a door at service "/open_door" set in the parameter file
 * that it calls every time the OpenDoor operation is triggered in the graph (ceneralized service).
 * Or the OpenDoor instance can call any service whose `service_name` is set in the operation's
 * metadata (e.g. "/open_door1", "/open_door2", ...) corresponding to other instances of similar
 * use. In this case, the specification of "OpenDoor" operation is less to do with the service name
 * and more to do with the type of service template (nav2_msgs) used to populate the request
 * (decentralized). This allows for massive reuse of a single plugin implementation in centralized
 * or decentralized applications.
 *
 * The templated nature of this node makes it a base class for any such service
 * containing additional request or response fields by implementing the virtual interfaces
 * configureEvent, populateRequest, processResponse appropriately. The std_srvs/Trigger
 * example TriggerEvent can be thought of as a useful entry level demo and useful working primitive
 */
template<typename SrvT>
class RouteOperationClient : public RouteOperation
{
public:
  /**
   * @brief Constructor
   */
  RouteOperationClient() = default;

  /**
   * @brief destructor
   */
  virtual ~RouteOperationClient() = default;

  /**
   * @brief Configure client with any necessary parameters, etc.
   * May change or reset `main_srv_name_` variable to control
   * main service name and existence.
   */
  virtual void configureEvent(
    const nav2_util::LifecycleNode::SharedPtr /*node*/,
    const std::string & /*name*/) {}

  /**
   * @brief Populate request with details for service, if necessary
   */
  virtual void populateRequest(
    std::shared_ptr<typename SrvT::Request>/*request*/, const Metadata * /*mdata*/) {}

  /**
   * @brief Process response from service to populate a result, if necessary
   */
  virtual OperationResult processResponse(
    std::shared_ptr<typename SrvT::Response>/*response*/) {return OperationResult();}

protected:
  /**
   * @brief Configure
   */
  void configure(
    const nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    const std::string & name) final
  {
    RCLCPP_INFO(node->get_logger(), "Configuring route operation client: %s.", name.c_str());
    name_ = name;
    logger_ = node->get_logger();
    node_ = node;
    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

    nav2_util::declare_parameter_if_not_declared(
      node, getName() + ".service_name", rclcpp::ParameterValue(""));
    main_srv_name_ = node->get_parameter(getName() + ".service_name").as_string();

    configureEvent(node, name);

    // There exists a single central service to use, create client.
    // If this is set to empty string after configuration, then the individual nodes will
    // indicate the endpoint for the particular service call.
    if (!main_srv_name_.empty()) {
      main_client_ = node->create_client<SrvT>(
        main_srv_name_, rclcpp::SystemDefaultsQoS(), callback_group_);
    }
  }

  /**
   * @brief The main operation to call a service of arbitrary type and arbitrary name
   * @param mdata Metadata corresponding to the operation in the navigation graph.
   * If metadata is invalid or irrelevant, a nullptr is given
   * @param node_achieved Node achieved, for additional context
   * @param edge_entered Edge entered by node achievement, for additional context
   * @param edge_exited Edge exited by node achievement, for additional context
   * @param route Current route being tracked in full, for additional context
   * @param curr_pose Current robot pose in the route frame, for additional context
   * @return Whether to perform rerouting and report blocked edges in that case
   */
  OperationResult perform(
    NodePtr node_achieved,
    EdgePtr /*edge_entered*/,
    EdgePtr /*edge_exited*/,
    const Route & /*route*/,
    const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
    const Metadata * mdata) final
  {
    auto req = std::make_shared<typename SrvT::Request>();
    std::shared_ptr<typename SrvT::Response> response;
    populateRequest(req, mdata);

    std::string srv_name;
    srv_name = mdata->getValue<std::string>("service_name", srv_name);
    if (srv_name.empty() && !main_client_) {
      throw nav2_core::OperationFailed(
              "Route operation service (" + getName() + ") needs 'server_name' "
              "set in the param file or in the operation's metadata!");
    }

    try {
      if (srv_name.empty()) {
        srv_name = main_srv_name_;
        response = callService(main_client_, req);
      } else {
        auto node = node_.lock();
        if (!node) {
          throw nav2_core::OperationFailed(
            "Route operation service (" + getName() + ") failed to lock node.");
        }
        auto client =
          node->create_client<SrvT>(srv_name, true);
        response = callService(client, req);
      }
    } catch (const std::exception & e) {
      throw nav2_core::OperationFailed(
        "Route operation service (" + getName() + ") failed to call service: " +
        srv_name + " at Node " + std::to_string(node_achieved->nodeid));
    }

    RCLCPP_INFO(
      logger_,
      "%s: Processed operation at Node %i with service %s.",
      name_.c_str(), node_achieved->nodeid, srv_name.c_str());
    return processResponse(response);
  }

  std::shared_ptr<typename SrvT::Response> callService(
    typename rclcpp::Client<SrvT>::SharedPtr client,
    std::shared_ptr<typename SrvT::Request> req,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(500ms))
  {
    auto node = node_.lock();
    if (!client->wait_for_service(1s)) {
      throw nav2_core::OperationFailed(
              "Route operation service " +
              std::string(client->get_service_name()) + "is not available!");
    }

    auto result = client->async_send_request(req);
    if (callback_group_executor_.spin_until_future_complete(result, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw nav2_core::OperationFailed(
              "Route operation service " +
              std::string(client->get_service_name()) + "failed to call!");
    }

    return result.get();
  }

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  std::string getName() override {return name_;}

  /**
   * @brief Indication that the adjust speed limit route operation is performed
   * on all state changes
   * @return The type of operation (on graph call, on status changes, or constantly)
   */
  RouteOperationType processType() final {return RouteOperationType::ON_GRAPH;}

  std::string name_, main_srv_name_;
  std::atomic_bool reroute_;
  rclcpp::Logger logger_{rclcpp::get_logger("RouteOperationClient")};
  typename rclcpp::Client<SrvT>::SharedPtr main_client_;
  nav2_util::LifecycleNode::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__ROUTE_OPERATION_CLIENT_HPP_
