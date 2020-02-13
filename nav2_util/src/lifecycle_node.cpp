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

#include "nav2_util/lifecycle_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace nav2_util
{

// The nav2_util::LifecycleNode class is temporary until we get the
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
// lifecycle nodes, it can simply set the use_rclcpp_node flag in the
// constructor and then provide the rclcpp_node_ to the helper classes, like
// MessageFilter.
//

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & namespace_, bool use_rclcpp_node,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, namespace_, options),
  use_rclcpp_node_(use_rclcpp_node)
{
  if (use_rclcpp_node_) {
    std::vector<std::string> new_args = options.arguments();
    new_args.push_back("--ros-args");
    new_args.push_back("-r");
    new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
    new_args.push_back("--");
    rclcpp_node_ = std::make_shared<rclcpp::Node>(
      "_", namespace_, rclcpp::NodeOptions(options).arguments(new_args));
    rclcpp_thread_ = std::make_unique<NodeThread>(rclcpp_node_);
  }
  print_lifecycle_node_notification();
}

LifecycleNode::~LifecycleNode()
{
  // In case this lifecycle node wasn't properly shut down, do it here
  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
  }
}

void LifecycleNode::print_lifecycle_node_notification()
{
  RCLCPP_INFO(
    get_logger(),
    "\n\t%s lifecycle node launched. \n"
    "\tWaiting on external lifecycle transitions to activate\n"
    "\tSee https://design.ros2.org/articles/node_lifecycle.html for more information.", get_name());
}

}  // namespace nav2_util
