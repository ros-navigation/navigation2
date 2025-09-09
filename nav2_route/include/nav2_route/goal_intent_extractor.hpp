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

#ifndef NAV2_ROUTE__GOAL_INTENT_EXTRACTOR_HPP_
#define NAV2_ROUTE__GOAL_INTENT_EXTRACTOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "tf2_ros/transform_listener.h"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/action/compute_route.hpp"
#include "nav2_msgs/action/compute_and_track_route.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/node_spatial_tree.hpp"
#include "nav2_route/goal_intent_search.hpp"

namespace nav2_route
{

/**
 * @class nav2_route::GoalIntentExtractor
 * @brief Extracts the start and goal nodes in the graph to use for the routing
 * request from the action request. This may use additional information about the
 * environment or simply find the corresponding nodes specified in the request.
 */
class GoalIntentExtractor
{
public:
  /**
   * @brief Constructor
   */
  GoalIntentExtractor() = default;

  /**
   * @brief Destructor
   */
  ~GoalIntentExtractor() = default;

  /**
   * @brief Configure extractor
   * @param node Node to use to create any interface needed
   * @param graph Graph to populate kD tree using
   * @param id_to_graph_map Remapping vector to correlate nodeIDs
   * @param tf TF buffer for transformations
   * @param costmap_subscriber Costmap subscriber to use for traversability
   * @param route_frame Planning frame
   * @param global_frame Global frame for costmap
   * @param base_frame Robot reference frame
   */
  void configure(
    nav2_util::LifecycleNode::SharedPtr node,
    Graph & graph,
    GraphToIDMap * id_to_graph_map,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & route_frame,
    const std::string & global_frame,
    const std::string & base_frame);

  /**
   * @brief Sets a new graph when updated
   * @param graph Graph to populate kD tree using
   * @param graph id_to_graph_map to get graph IDX for node IDs
   */
  void setGraph(Graph & graph, GraphToIDMap * id_to_graph_map);

  /**
   * @brief Transforms a pose into the route frame
   * @param pose Pose to transform (e.g. start, goal)
   * @param frame_id Frame to transform to
   */
  geometry_msgs::msg::PoseStamped transformPose(
    geometry_msgs::msg::PoseStamped & pose,
    const std::string & frame_id);
  /**
   * @brief Main API to find the start and goal graph IDX (not IDs) for routing
   * @param goal Action request goal
   * @return start, goal node IDx
   */
  template<typename GoalT>
  NodeExtents
  findStartandGoal(const std::shared_ptr<const GoalT> goal);

  /**
   * @brief Prune the start and end nodes in a route if the start or goal poses,
   * respectively, are already along the route to avoid backtracking
   * @param inpute_route Route to prune
   * @param goal Action request goal
   * @param first_time If this is the first time routing
   * @return Pruned route
   */
  template<typename GoalT>
  Route pruneStartandGoal(
    const Route & input_route,
    const std::shared_ptr<const GoalT> goal,
    ReroutingState & rerouting_info);

  /**
   * @brief Override the start pose for use in pruning if it is externally overridden
   * Usually by the rerouting logic
   * @param start_pose Starting pose of robot to prune using
   */
  void overrideStart(const geometry_msgs::msg::PoseStamped & start_pose);

  /**
   * @brief gets the desired start pose
   * @return PoseStamped of start pose
   */
  geometry_msgs::msg::PoseStamped getStart();

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("GoalIntentExtractor")};
  std::shared_ptr<NodeSpatialTree> node_spatial_tree_;
  GraphToIDMap * id_to_graph_map_;
  Graph * graph_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  std::string route_frame_;
  std::string base_frame_;
  std::string global_frame_;
  geometry_msgs::msg::PoseStamped start_, goal_;
  bool prune_goal_, enable_search_;
  int max_nn_search_iterations_;
  float max_dist_from_edge_, min_dist_from_goal_, min_dist_from_start_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__GOAL_INTENT_EXTRACTOR_HPP_
