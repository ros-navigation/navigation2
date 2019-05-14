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

#ifndef DWB_CRITICS__COLLISION_CHECKER_HPP_
#define DWB_CRITICS__COLLISION_CHECKER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace dwb_critics
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;

class CollisionChecker
{
public:
  CollisionChecker(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub,
    tf2_ros::Buffer & tf_buffer,
    std::string name = "collision_checker");
  
  ~CollisionChecker();

  double scorePose(
    const geometry_msgs::msg::Pose2D & pose);
  bool isCollisionFree(
    const geometry_msgs::msg::Pose2D & pose);

protected:
  double lineCost(int x0, int x1, int y0, int y1);
  double pointCost(int x, int y);
  bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose) const;
  void unorientFootprint(const Footprint & oriented_footprint, Footprint & reset_footprint);

  // Transform buffer
  tf2_ros::Buffer & tf_buffer_;

  // TF Frames
  std::string global_frame_;
  std::string robot_base_frame_;

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;  
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__COLLISION_CHECKER_HPP_
