// Copyright (c) 2020 Vinny Ruia
// Copyright (c) 2020 Sarthak Mittal
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
// limitations under the License. Reserved.

#ifndef BEHAVIOR_TREE__SERVER_HANDLER_HPP_
#define BEHAVIOR_TREE__SERVER_HANDLER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "dummy_servers.hpp"

class ComputePathToPoseActionServer
  : public DummyActionServer<nav2_msgs::action::ComputePathToPose>
{
public:
  explicit ComputePathToPoseActionServer(const rclcpp::Node::SharedPtr & node)
  : DummyActionServer(node, "compute_path_to_pose")
  {
    result_ = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result_->path.header;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 6; ++i) {
      result_->path.poses.push_back(pose);
    }
  }

  std::shared_ptr<nav2_msgs::action::ComputePathToPose::Result> fillResult() override
  {
    return result_;
  }

private:
  std::shared_ptr<nav2_msgs::action::ComputePathToPose::Result> result_;
};

class ServerHandler
{
public:
  ServerHandler();
  ~ServerHandler();

  void activate();

  void deactivate();

  bool isActive() const
  {
    return is_active_;
  }

  void reset() const;

public:
  std::unique_ptr<DummyService<nav2_msgs::srv::ClearEntireCostmap>> clear_local_costmap_server;
  std::unique_ptr<DummyService<nav2_msgs::srv::ClearEntireCostmap>> clear_global_costmap_server;
  std::unique_ptr<ComputePathToPoseActionServer> compute_path_to_pose_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::FollowPath>> follow_path_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::Spin>> spin_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::Wait>> wait_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::BackUp>> backup_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::ComputePathThroughPoses>> ntp_server;

private:
  void spinThread();

  bool is_active_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<std::thread> server_thread_;
};

#endif  //  BEHAVIOR_TREE__SERVER_HANDLER_HPP_
