// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef NAV2_LIFECYCLE__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NAV2_LIFECYCLE__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_lifecycle
{

class LifecycleServiceClient
{
public:
  LifecycleServiceClient(rclcpp::Node::SharedPtr node, const std::string & target_node_name);

  // Initiate a state transition on the target lifecycle node
  bool changeState(
    std::uint8_t transition, std::chrono::seconds time_out = std::chrono::seconds::max());

protected:
  // The node for this service client to use  (can't already be on an executor)
  rclcpp::Node::SharedPtr node_;

  // The name of the target node that this service client will be controlling
  std::string target_node_name_;

  // The rclcpp service client we'll use to make the service calls
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_;

  // A support function, for logging, to convert a transition code to a string
  const char * transition_to_str(uint8_t transition);
};

}  // namespace nav2_lifecycle

#endif  // NAV2_LIFECYCLE__LIFECYCLE_SERVICE_CLIENT_HPP_
