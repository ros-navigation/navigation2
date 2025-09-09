// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__COLLISION_MONITOR_HPP_
#define NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__COLLISION_MONITOR_HPP_

#include <math.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_route/interfaces/route_operation.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/route_exceptions.hpp"

namespace nav2_route
{

/**
 * @class CollisionMonitor
 * @brief A route operation to process a costmap to determine if a route is blocked
 * requiring immediate rerouting. If using the local costmap topic, then it will check
 * in the local time horizon only. if using the global, it may check the full route for
 * continued validity. It is however recommended to specify a maximum collision distance
 * for evaluation to prevent necessarily long-term evaluation of collision information which
 * may not be representative of the conditions in that area by the time the robot gets there.
 */
class CollisionMonitor : public RouteOperation
{
public:
  struct LineSegment
  {
    unsigned int x0, y0, x1, y1;
  };

  /**
   * @brief Constructor
   */
  CollisionMonitor() = default;

  /**
   * @brief destructor
   */
  virtual ~CollisionMonitor() = default;

  /**
   * @brief Configure
   */
  void configure(
    const nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) override;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  std::string getName() override {return name_;}

  /**
   * @brief Indication that the adjust speed limit route operation is performed
   * on all state changes
   * @return The type of operation (on graph call, on status changes, or constantly)
   */
  RouteOperationType processType() override {return RouteOperationType::ON_QUERY;}

  /**
   * @brief The main speed limit operation to adjust the maximum speed of the vehicle
   * @param mdata Metadata corresponding to the operation in the navigation graph.
   * If metadata is invalid or irrelevant, a nullptr is given
   * @param node_achieved Node achieved, for additional context
   * @param edge_entered Edge entered by node achievement, for additional context
   * @param edge_exited Edge exited by node achievement, for additional context
   * @param route Current route being tracked in full, for additional context
   * @param curr_pose Current robot pose in the route frame, for additional context
   * @return Whether to perform rerouting and report blocked edges in that case
   */
  OperationResult perform(
    NodePtr /*node*/,
    EdgePtr curr_edge,
    EdgePtr /*edge_exited*/,
    const Route & route,
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const Metadata * /*mdata*/) override;

protected:
  /**
   * @brief Backs out the end coordinate along the line segment start-end to length dist
   * @param start Start of line segment
   * @param end End of line segment
   * @param dist Distance along line segment to find the new end point along
   * @return Coordinates of the new end point `dist` away from the start along the line
   */
  Coordinates backoutValidEndPoint(
    const Coordinates & start, const Coordinates & end, const float dist);

  /**
   * @brief Backs out the line end coordinates of the start-end line segment
   * where costmap transforms are possible
   * @param start Start of line segment
   * @param line LineSegment object to replace the x1/y1 values for along segment until invalid
   * @return If any part s of the segment requested is valid
   */
  bool backoutValidEndPoint(const Coordinates & start, LineSegment & line);

  /**
   * @brief Converts a line segment start-end into a LineSegment struct in costmap frame
   * @param start Start of line segment
   * @param end End of line segment
   * @param line LineSegment object to populate
   * @return If line segment is valid (e.g. start and end both in costmap transforms)
   */
  bool lineToMap(const Coordinates & start, const Coordinates & end, LineSegment & line);

  /**
   * @brief Checks a line segment in costmap frame for validity
   * @param line LineSegment object to collision check in costmap set
   * @return If any part of the line segment is in collision
   */
  bool isInCollision(const LineSegment & line);

  /**
   * @brief Gets the latest costmap from the costmap subscriber
   */
  void getCostmap();

  std::string name_, topic_;
  std::atomic_bool reroute_;
  rclcpp::Logger logger_{rclcpp::get_logger("CollisionMonitor")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_check_time_;
  rclcpp::Duration checking_duration_{0, 0};
  float max_collision_dist_, max_cost_;
  bool reroute_on_collision_;
  unsigned int check_resolution_{1u};
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_{nullptr};
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__COLLISION_MONITOR_HPP_
