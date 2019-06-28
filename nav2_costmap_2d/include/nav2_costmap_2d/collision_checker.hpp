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

#ifndef NAV2_COSTMAP_2D__COLLISION_CHECKER_HPP_
#define NAV2_COSTMAP_2D__COLLISION_CHECKER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/get_robot_pose_client.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_costmap_2d
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;

class CollisionChecker
{
public:
  CollisionChecker(
    CostmapSubscriber & costmap_sub,
    FootprintSubscriber & footprint_sub,
    nav2_util::GetRobotPoseClient & get_robot_pose_client,
    std::string name = "collision_checker");

  ~CollisionChecker();

  // Returns the obstacle footprint score for a particular pose
  double scorePose(const geometry_msgs::msg::Pose2D & pose);
  bool isCollisionFree(const geometry_msgs::msg::Pose2D & pose);

protected:
  double lineCost(int x0, int x1, int y0, int y1) const;
  double pointCost(int x, int y) const;
  bool getRobotPose(geometry_msgs::msg::Pose & current_pose);
  void unorientFootprint(const Footprint & oriented_footprint, Footprint & reset_footprint);
  void worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  Footprint getFootprint(const geometry_msgs::msg::Pose2D & pose);
  double footprintCost(const Footprint footprint);

  std::shared_ptr<Costmap2D> costmap_;

  // Name used for logging
  std::string name_;
  nav2_util::GetRobotPoseClient & get_robot_pose_client_;
  CostmapSubscriber & costmap_sub_;
  FootprintSubscriber & footprint_sub_;
};
}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COLLISION_CHECKER_HPP_
