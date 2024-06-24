// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 */
template<class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtServiceNode constructor
   * @param service_node_name BT node name
   * @param conf BT node configuration
   * @param service_name Optional service name this node creates a client for instead of from input port
   */
  BtServiceNode(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf,
    const std::string & service_name = "")
  : BT::ActionNodeBase(service_node_name, conf), service_name_(service_name), service_node_name_(
      service_node_name)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    // timeout should be less than bt_loop_duration to be able to finish the current tick
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // Now that we have node_ to use, create the service client for this BT service
    getInput("service_name", service_name_);
    service_client_ = node_->create_client<ServiceT>(
      service_name_,
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);

    // Make a request for the service without parameter
    request_ = std::make_shared<typename ServiceT::Request>();

    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for \"%s\" service",
      service_name_.c_str());
    if (!service_client_->wait_for_service(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" service server not available after waiting for %.2fs",
        service_name_.c_str(), wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
              std::string(
                "Service server %s not available",
                service_node_name.c_str()));
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "\"%s\" BtServiceNode initialized",
      service_node_name_.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode()
  {
  }

  /**
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override
  {
    if (!request_sent_) {
      // reset the flag to send the request or not,
      // allowing the user the option to set it in on_tick
      should_send_request_ = true;

      // user defined callback, may modify "should_send_request_".
      on_tick();

      if (!should_send_request_) {
        return BT::NodeStatus::FAILURE;
      }

      future_result_ = service_client_->async_send_request(request_).share();
      sent_time_ = node_->now();
      request_sent_ = true;
    }
    return check_future();
  }

  /**
   * @brief The other (optional) override required by a BT service.
   */
  void halt() override
  {
    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  /**
   * @brief Function to perform some user-defined operation on tick
   * Fill in service request with information if necessary
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the service. Could put a value on the blackboard.
   * @param response can be used to get the result of the service call in the BT Node.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override to return another value
   */
  virtual BT::NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response>/*response*/)
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Check the future and decide the status of BT
   * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE otherwise
   */
  virtual BT::NodeStatus check_future()
  {
    auto elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;

      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Node timed out while executing service call to %s.", service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout waiting
   * for a result that hasn't been received yet
   */
  virtual void on_wait_for_result()
  {
  }

protected:
  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string service_name_, service_node_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds max_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // To track the server response when a new request is sent
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;

  // Can be set in on_tick or on_wait_for_result to indicate if a request should be sent.
  bool should_send_request_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
