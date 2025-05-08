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
//
// Modified by: Shivang Patel (shivaan14@gmail.com)

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

CostmapTopicCollisionChecker::CostmapTopicCollisionChecker(
  CostmapSubscriber & costmap_sub,
  FootprintSubscriber & footprint_sub,
  std::string name)
: name_(name),
  costmap_sub_(costmap_sub),
  footprint_sub_(&footprint_sub),
  collision_checker_(nullptr)
{}

CostmapTopicCollisionChecker::CostmapTopicCollisionChecker(
  CostmapSubscriber & costmap_sub,
  std::string footprint_string,
  std::string name)
: name_(name),
  costmap_sub_(costmap_sub),
  collision_checker_(nullptr)
{
  if (!makeFootprintFromString(footprint_string, footprint_)) {
    throw CollisionCheckerException("Failed to create footprint from string");
  }
}

bool CostmapTopicCollisionChecker::isCollisionFree(
  const geometry_msgs::msg::Pose2D & pose,
  bool fetch_costmap_and_footprint)
{
  try {
    if (scorePose(pose, fetch_costmap_and_footprint) >= LETHAL_OBSTACLE) {
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

double CostmapTopicCollisionChecker::scorePose(
  const geometry_msgs::msg::Pose2D & pose,
  bool fetch_costmap_and_footprint)
{
  if (fetch_costmap_and_footprint) {
    try {
      collision_checker_.setCostmap(costmap_sub_.getCostmap());
    } catch (const std::runtime_error & e) {
      throw CollisionCheckerException(e.what());
    }
  }

  unsigned int cell_x, cell_y;
  if (!collision_checker_.worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", cell_x, cell_y);
    throw IllegalPoseException(name_, "Pose Goes Off Grid.");
  }

  return collision_checker_.footprintCost(getFootprint(pose, fetch_costmap_and_footprint));
}

Footprint CostmapTopicCollisionChecker::getFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  bool fetch_latest_footprint)
{
  if (fetch_latest_footprint) {
    std_msgs::msg::Header header;

    // if footprint_sub_ was not initialized (alternative constructor), we are using the
    // footprint built from the footprint_string alternative constructor argument.
    if (footprint_sub_ && !footprint_sub_->getFootprintInRobotFrame(footprint_, header)) {
      throw CollisionCheckerException("Current footprint not available.");
    }
  }
  Footprint footprint;
  transformFootprint(pose.x, pose.y, pose.theta, footprint_, footprint);

  return footprint;
}

}  // namespace nav2_costmap_2d
