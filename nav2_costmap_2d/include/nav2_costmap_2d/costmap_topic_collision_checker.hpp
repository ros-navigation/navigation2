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

#ifndef NAV2_COSTMAP_2D__COSTMAP_TOPIC_COLLISION_CHECKER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_TOPIC_COLLISION_CHECKER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"

namespace nav2_costmap_2d
{
/**
 * @class CostmapTopicCollisionChecker
 * @brief Using a costmap via a ros topic, this object is used to
 * find if robot poses are in collision with the costmap environment
 */
class CostmapTopicCollisionChecker
{
public:
  /**
   * @brief A constructor
   */
  CostmapTopicCollisionChecker(
    CostmapSubscriber & costmap_sub,
    FootprintSubscriber & footprint_sub,
    std::string name = "collision_checker");

  /**
   * @brief A destructor
   */
  ~CostmapTopicCollisionChecker() = default;

  /**
   * @brief Returns the obstacle footprint score for a particular pose
   */
  double scorePose(const geometry_msgs::msg::Pose2D & pose, bool updateCostmap = true);
  /**
   * @brief Returns if a pose is collision free
   */
  bool isCollisionFree(const geometry_msgs::msg::Pose2D & pose, bool updateCostmap = true);

protected:
  /**
   * @brief Get a footprint at a set pose
   */
  Footprint getFootprint(const geometry_msgs::msg::Pose2D & pose);

  // Name used for logging
  std::string name_;
  CostmapSubscriber & costmap_sub_;
  FootprintSubscriber & footprint_sub_;
  FootprintCollisionChecker<std::shared_ptr<Costmap2D>> collision_checker_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_TOPIC_COLLISION_CHECKER_HPP_
