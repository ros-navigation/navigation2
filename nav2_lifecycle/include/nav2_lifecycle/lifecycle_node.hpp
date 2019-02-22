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

#ifndef NAV2_LIFECYCLE__LIFECYCLE_NODE_HPP_
#define NAV2_LIFECYCLE__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "nav2_lifecycle/lifecycle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_lifecycle
{

class LifecycleNode : public rclcpp_lifecycle::LifecycleNode, public ILifecycle
{
public:
  LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_ = "",
    bool use_rclcpp_node = false,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();

protected:
  // These two are recommended to be implemented by derived classes but have empty,
  // default implementations
  virtual CallbackReturn onShutdown(const rclcpp_lifecycle::State & state);
  virtual CallbackReturn onError(const rclcpp_lifecycle::State & state);

  // Overrides, for the standard rclcpp::LifecycleNode change_state service callbacks.
  // These simply redirect to our own versions (OnConfigure, etc.) so that we can
  // maintain a consistent naming convention. This also provides a degree of
  // between the two, which could be useful.
  nav2_lifecycle::CallbackReturn on_configure(const rclcpp_lifecycle::State & cur_state) override;
  nav2_lifecycle::CallbackReturn on_activate(const rclcpp_lifecycle::State & cur_state) override;
  nav2_lifecycle::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & cur_state) override;
  nav2_lifecycle::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & cur_state) override;
  nav2_lifecycle::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & cur_state) override;
  nav2_lifecycle::CallbackReturn on_error(const rclcpp_lifecycle::State & cur_state) override;

  // Whether or not to create a local rclcpp::Node which can be used for ROS2 classes that don't
  // yet support lifecycle nodes
  bool use_rclcpp_node_;

  // The local node
  rclcpp::Node::SharedPtr rclcpp_node_;

  // When creating a local node, this class will launch a separate thread created to spin the node
  std::unique_ptr<std::thread> rclcpp_thread_;
};

}  // namespace nav2_lifecycle

#endif  // NAV2_LIFECYCLE__LIFECYCLE_NODE_HPP_
