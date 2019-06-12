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

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

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
    // Create a non-lifecycle node to use to interface to ROS2 functionality that is
    // not yet lifecycle enabled
    rclcpp_node_ = std::make_shared<rclcpp::Node>(node_name + "_rclcpp_node", namespace_);

    // The lambda function for this thread will efficiently spin the node
    auto f = [this](rclcpp::Node::SharedPtr node) {
      std::shared_future<void> future_result = std::async(std::launch::async,
        [this]{ 
          std::unique_lock<std::mutex> lk(m_); 
          cv_.wait(lk); 
        }); 

      // Wait for the result
      rclcpp::spin_until_future_complete(node, future_result);
    };

    // Create the thread to spin this node
    rclcpp_thread_ = std::make_unique<std::thread>(f, rclcpp_node_);
  }
}

LifecycleNode::~LifecycleNode()
{
  // In case this lifecycle node wasn't properly shut down, do it here
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
  }

  if (use_rclcpp_node_) {
    cv_.notify_one();

    auto timer_callback = [this]() -> void {RCLCPP_INFO(this->get_logger(), "Hello, world!");};
    auto timer_ = rclcpp_node_->create_wall_timer(1ms, timer_callback);

    rclcpp_thread_->join();
  }
}

}  // namespace nav2_util
