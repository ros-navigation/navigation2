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

#include "nav2_util/lifecycle_helper_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

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

protected:
  // Whether or not to create a local rclcpp::Node which can be used for ROS2 classes that don't
  // yet support lifecycle nodes
  bool use_rclcpp_node_;

  // The local node
  rclcpp::Node::SharedPtr rclcpp_node_;

  // When creating a local node, this class will launch a separate thread created to spin the node
  std::unique_ptr<std::thread> rclcpp_thread_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> rclcpp_exec_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_NODE_HPP_
