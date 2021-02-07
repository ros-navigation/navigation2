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

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

template<class ServiceT>
class BtServiceNode : public BT::SyncActionNode
{
public:
  BtServiceNode(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(service_node_name, conf), service_node_name_(service_node_name)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    // Get the required items from the blackboard
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // Now that we have node_ to use, create the service client for this BT service
    getInput("service_name", service_name_);
    service_client_ = node_->create_client<ServiceT>(service_name_);

    // Make a request for the service without parameter
    request_ = std::make_shared<typename ServiceT::Request>();

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for \"%s\" service",
      service_name_.c_str());
    service_client_->wait_for_service();

    RCLCPP_INFO(
      node_->get_logger(), "\"%s\" BtServiceNode initialized",
      service_node_name_.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode()
  {
  }

  // Any subclass of BtServiceNode that accepts parameters must provide a providedPorts method
  // and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // The main override required by a BT service
  BT::NodeStatus tick() override
  {
    on_tick();
    auto future_result = service_client_->async_send_request(request_);
    return check_future(future_result);
  }

  // Fill in service request with information if necessary
  virtual void on_tick()
  {
  }

  // Check the future and decide the status of Behaviortree
  virtual BT::NodeStatus check_future(
    std::shared_future<typename ServiceT::Response::SharedPtr> future_result)
  {
    rclcpp::FutureReturnCode rc;
    rc = rclcpp::spin_until_future_complete(
      node_,
      future_result, server_timeout_);
    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      return BT::NodeStatus::SUCCESS;
    } else if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Node timed out while executing service call to %s.", service_name_.c_str());
      on_wait_for_result();
    }
    return BT::NodeStatus::FAILURE;
  }

  // An opportunity to do something after
  // a timeout waiting for a result that hasn't been received yet
  virtual void on_wait_for_result()
  {
  }

protected:
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

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds server_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
