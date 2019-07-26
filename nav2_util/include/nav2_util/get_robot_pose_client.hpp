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

#ifndef NAV2_UTIL__GET_ROBOT_POSE_CLIENT_HPP_
#define NAV2_UTIL__GET_ROBOT_POSE_CLIENT_HPP_

#include <string>

#include "nav2_util/service_client.hpp"
#include "nav2_msgs/srv/get_robot_pose.hpp"

namespace nav2_util
{

class GetRobotPoseClient : public ServiceClient<nav2_msgs::srv::GetRobotPose>
{
public:
  explicit GetRobotPoseClient(const std::string & parent_node_name)
  : ServiceClient<nav2_msgs::srv::GetRobotPose>("GetRobotPose", parent_node_name)
  {
  }

  explicit GetRobotPoseClient(rclcpp::Node::SharedPtr & node)
  : ServiceClient<nav2_msgs::srv::GetRobotPose>("GetRobotPose", node)
  {
  }

  using GetRobotPoseRequest =
    ServiceClient<nav2_msgs::srv::GetRobotPose>::RequestType;
  using GetRobotPoseResponse =
    ServiceClient<nav2_msgs::srv::GetRobotPose>::ResponseType;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__GET_ROBOT_POSE_CLIENT_HPP_
