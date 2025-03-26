// Copyright (c) 2024, Polymath Robotics Inc.
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

#ifndef NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__GOAL_ORIENTATION_SCORER_HPP_
#define NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__GOAL_ORIENTATION_SCORER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_route/interfaces/edge_cost_function.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

namespace nav2_route
{

/**
 * @class GoalOrientationScorer
 * @brief Rejects or accepts final edge depending on the edge's deviation 
 * from the goal pose orientation. If the deviation is above some user 
 * defined threshold, false is returned, if it is below, true is returned
 */
class GoalOrientationScorer : public EdgeCostFunction
{
public:
  /**
   * @brief Constructor
   */
  GoalOrientationScorer() = default;

  /**
   * @brief destructor
   */
  virtual ~GoalOrientationScorer() = default;

  /**
   * @brief Configure
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & name) override;

  /**
   * @brief Main scoring plugin API
   * @param edge The edge pointer to score, which has access to the
   * start/end nodes and their associated metadata and actions
   * @param cost of the edge scored
   * @return bool if this edge is open valid to traverse
   */
  bool score(
    const EdgePtr edge, const RouteData & route_data,
    const EdgeType & edge_type, float & cost) override;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  std::string getName() override;

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("GoalOrientationScorer")};
  std::string name_;
  double orientation_tolerance_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__GOAL_ORIENTATION_SCORER_HPP_
