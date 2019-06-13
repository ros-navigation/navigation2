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
// Until we get the required support in ROS2 and then utilize the updated
// interfaces, this class will provide a bridging solution. This class provides
// an optional normal ROS node an automatically launches a thread that processes
// the node's messages. If a derived class needs to interface to one of these classes
// like MessageFilter that don't yet support lifecycle nodes, it can simply set the
//  use_rclcpp_node flag in the constructor and then provide the rclcpp_node_ to the
//  helper classes, like MessageFilter.
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
    // If requested, create a non-lifecycle node (an "rclcpp_node") to use to interface
    // to ROS2 functionality that is not yet lifecycle enabled
    rclcpp_node_ = std::make_shared<rclcpp::Node>(node_name + "_rclcpp_node", namespace_);

    // Create a lambda function to efficiently spin the node
    auto f = [this](rclcpp::Node::SharedPtr node) {
        // Create an async task w/ a shared_future so that we can use this with
        // spin_until_future_complete. The task simply waits until the condition variable
        // is signaled. This happens when the LifecycleNode is destructed.
        std::shared_future<void> future_result = std::async(std::launch::async,
            [this] {
              std::unique_lock<std::mutex> lk(mutex_);
              should_exit_cv_.wait(lk);
            });

        // Process messages for this thread until the future result is complete, which,
        // in our case, indicates that that the lifecycle node is exiting
        rclcpp::spin_until_future_complete(node, future_result);
      };

    // Create the thread to spin this node, providing the above lambda function
    rclcpp_thread_ = std::make_unique<std::thread>(f, rclcpp_node_);
  }
}

LifecycleNode::~LifecycleNode()
{
  // In case this lifecycle node wasn't properly shut down, manually execute the
  // state transitions
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
  }

  if (use_rclcpp_node_) {
    // If we're using an rclcpp_node, notify that thread so that it can terminate
    should_exit_cv_.notify_one();

    // Make sure there is at least one item on the node's input queue so that it will
    // break out of the spin_until_future_complete
    auto timer_ =
      rclcpp_node_->create_wall_timer(1ms, [](rclcpp::TimerBase & timer) {timer.cancel();});

    // Then, join with that thread before continuing
    rclcpp_thread_->join();
  }
}

}  // namespace nav2_util
