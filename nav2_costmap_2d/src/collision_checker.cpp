// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Shivang Patel
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

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "nav2_costmap_2d/collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

CollisionChecker::CollisionChecker(
  CostmapSubscriber & costmap_sub,
  FootprintSubscriber & footprint_sub,
  tf2_ros::Buffer & tf,
  std::string name,
  std::string global_frame)
: CollisionBase(),
  name_(name),
  global_frame_(global_frame),
  tf_(tf),
  costmap_sub_(costmap_sub),
  footprint_sub_(footprint_sub)
{
}

bool CollisionChecker::isCollisionFree(
  const geometry_msgs::msg::Pose2D & pose)
{
  try {
    if (scorePose(pose) >= LETHAL_OBSTACLE) {
      return false;
    }
    return true;
  } catch (const IllegalPoseException & e) {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());
    return false;
  } catch (const CollisionCheckerException & e) {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "Failed to check pose score!");
    return false;
  }
}

double CollisionChecker::scorePose(
  const geometry_msgs::msg::Pose2D & pose)
{
  try {
    costmap_ = costmap_sub_.getCostmap();
  } catch (const std::runtime_error & e) {
    throw CollisionCheckerException(e.what());
  }

  unsigned int cell_x, cell_y;
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", cell_x, cell_y);
    throw IllegalPoseException(name_, "Pose Goes Off Grid.");
  }

  return footprintCost(getFootprint(pose));
}

void CollisionChecker::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", mx, my);
    throw IllegalPoseException(name_, "Footprint Goes Off Grid.");
  }
}

Footprint CollisionChecker::getFootprint(const geometry_msgs::msg::Pose2D & pose)
{
  Footprint footprint;
  if (!footprint_sub_.getFootprint(footprint)) {
    throw CollisionCheckerException("Current footprint not available.");
  }

  Footprint footprint_spec;
  unorientFootprint(footprint, footprint_spec);
  transformFootprint(pose.x, pose.y, pose.theta, footprint_spec, footprint);

  return footprint;
}

double CollisionChecker::pointCost(int x, int y) const
{
  unsigned char cost = costmap_->getCost(x, y);
  // if the cell is in an obstacle the path is invalid or unknown
  if (cost == LETHAL_OBSTACLE) {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", x, y);
    throw IllegalPoseException(name_, "Footprint Hits Obstacle.");
  } else if (cost == NO_INFORMATION) {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", x, y);
    throw IllegalPoseException(name_, "Footprint Hits Unknown Region.");
  }

  return cost;
}

void CollisionChecker::unorientFootprint(
  const std::vector<geometry_msgs::msg::Point> & oriented_footprint,
  std::vector<geometry_msgs::msg::Point> & reset_footprint)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, tf_, global_frame_)) {
    throw CollisionCheckerException("Robot pose unavailable.");
  }

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  Footprint temp;
  transformFootprint(-x, -y, 0, oriented_footprint, temp);
  transformFootprint(0, 0, -theta, temp, reset_footprint);
}


}  // namespace nav2_costmap_2d
