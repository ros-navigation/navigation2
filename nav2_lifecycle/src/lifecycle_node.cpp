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

#include "nav2_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <string>

namespace nav2_lifecycle
{

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & namespace_,
  bool use_rclcpp_node,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, namespace_, options),
  use_rclcpp_node_(use_rclcpp_node)
{
  if (use_rclcpp_node_) {
    // Avoid the extra services we don't need for this node
    rclcpp::NodeOptions local_node_options;
    local_node_options.start_parameter_services(false);
    local_node_options.start_parameter_event_publisher(false);

    rclcpp_node_ = std::make_shared<rclcpp::Node>(node_name + "_rclcpp_node", namespace_,
        local_node_options);

    rclcpp_thread_ = std::make_unique<std::thread>(
      [](rclcpp::Node::SharedPtr node) {rclcpp::spin(node);}, rclcpp_node_
    );
  }
}

LifecycleNode::~LifecycleNode()
{
  if (use_rclcpp_node_) {
    rclcpp_thread_->join();
  }
}

CallbackReturn
LifecycleNode::onShutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "onShutdown");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
LifecycleNode::onError(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "onError");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

CallbackReturn
LifecycleNode::on_configure(const rclcpp_lifecycle::State & cur_state)
{
  return onConfigure(cur_state);
}

CallbackReturn
LifecycleNode::on_activate(const rclcpp_lifecycle::State & cur_state)
{
  return onActivate(cur_state);
}

CallbackReturn
LifecycleNode::on_deactivate(const rclcpp_lifecycle::State & cur_state)
{
  return onDeactivate(cur_state);
}

CallbackReturn
LifecycleNode::on_cleanup(const rclcpp_lifecycle::State & cur_state)
{
  return onCleanup(cur_state);
}

CallbackReturn
LifecycleNode::on_shutdown(const rclcpp_lifecycle::State & cur_state)
{
  return onShutdown(cur_state);
}

CallbackReturn
LifecycleNode::on_error(const rclcpp_lifecycle::State & cur_state)
{
  return onError(cur_state);
}

}  // namespace nav2_lifecycle
