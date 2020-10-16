// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__LIFECYCLE_NODE_HPP_
#define NAV2_UTIL__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "nav2_util/node_thread.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// The following is a temporary wrapper for rclcpp_lifecycle::LifecycleNode. This class
// adds the optional creation of an rclcpp::Node that can be used by derived classes
// to interface to classes, such as MessageFilter and TransformListener, that don't yet
// support lifecycle nodes. Once we get the fixes into ROS2, this class will be removed.

class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_ = "",
    bool use_rclcpp_node = false,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();

  typedef struct
  {
    double from_value;
    double to_value;
    double step;
  } floating_point_range;

  typedef struct
  {
    int from_value;
    int to_value;
    int step;
  } integer_range;

  // Declare a parameter that has no integer or floating point range constraints
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  // Declare a parameter that has a floating point range constraint
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const floating_point_range fp_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = fp_range.from_value;
    descriptor.floating_point_range[0].to_value = fp_range.to_value;
    descriptor.floating_point_range[0].step = fp_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  // Declare a parameter that has an integer range constraint
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const integer_range int_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = int_range.from_value;
    descriptor.integer_range[0].to_value = int_range.to_value;
    descriptor.integer_range[0].step = int_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  std::shared_ptr<nav2_util::LifecycleNode> shared_from_this()
  {
    return std::static_pointer_cast<nav2_util::LifecycleNode>(
      rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_FATAL(
      get_logger(),
      "Lifecycle node %s does not have error state implemented", get_name());
    return nav2_util::CallbackReturn::SUCCESS;
  }

protected:
  void print_lifecycle_node_notification();

  // Whether or not to create a local rclcpp::Node which can be used for ROS2 classes that don't
  // yet support lifecycle nodes
  bool use_rclcpp_node_;

  // The local node
  rclcpp::Node::SharedPtr rclcpp_node_;

  // When creating a local node, this class will launch a separate thread created to spin the node
  std::unique_ptr<NodeThread> rclcpp_thread_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_NODE_HPP_
