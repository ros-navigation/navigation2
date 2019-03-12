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

#include "nav2_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <string>

namespace nav2_lifecycle
{

// The nav2_lifecycle::LifecycleNode class is temporary until we get the
// required support for lifecycle nodes in MessageFilter, TransformListener,
// and TransforBroadcaster. We have submitted issues for these and will
// be submitting PRs to add the fixes:
//
//     https://github.com/ros2/geometry2/issues/95
//     https://github.com/ros2/geometry2/issues/94
//     https://github.com/ros2/geometry2/issues/70
//
// Until then, this class can provide a normal ROS node that has a thread
// that processes the node's messages. If a derived class needs to interface
// to one of these classes - MessageFilter, etc. - that don't yet support
// lifecycle nodes, it can simply set the use_rclcpp_node flag in the constructor
// and then provide the rclcpp_node_ to the helper classes, like MessageFilter.
//

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

}  // namespace nav2_lifecycle
