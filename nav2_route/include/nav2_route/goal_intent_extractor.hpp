// Copyright (c) 2023, Samsung Research America
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

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/action/compute_route.hpp"
#include "nav2_msgs/action/compute_and_track_route.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/node_spatial_tree.hpp"
#include "nav2_route/dijkstra_search.hpp"

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
   * @brief Configure extactor
   * @param node Node to use to create any interface needed
   * @param graph Graph to populate kD tree using
   * @param id_to_graph_map Remapping vector to correlate nodeIDs
   * @param tf TF buffer for transformations
   * @param route_frame Planning frame
   * @param base_frame Robot reference frame
   */
  void configure(
    nav2_util::LifecycleNode::SharedPtr node,
    Graph & graph,
    GraphToIDMap * id_to_graph_map,
    std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string & route_frame,
    const std::string & base_frame);

  /**
   * @brief Initialize extractor
   */
  void initialize();

  /**
   * @brief Sets a new graph when updated
   * @param graph Graph to populate kD tree using
   * @param graph id_to_graph_map to get graph IDX for node IDs
   */
  void setGraph(Graph & graph, GraphToIDMap * id_to_graph_map);

  /**
   * @brief Transforms a pose into the desired frame
   * @param pose Pose to transform (e.g. start, goal)
   * @param frame_id The frame to transform the pose into
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
   * @brief Set the start pose for use in pruning if it is externally overridden
   * @param start_pose Starting pose of robot to prune using
   */
  void setStart(const geometry_msgs::msg::PoseStamped & start_pose);

protected:
  /**
   * @brief Checks if there is a valid connection between a graph node and a pose by 
   * preforming a breadth first search through the costmap
   * @param node_indices A list of graph node indices to check
   * @param pose The pose that needs to be associated with a graph node
   * @return The index of the closest graph node found in the search 
   * @throws nav2_core::StartOutsideMapBounds If the start index is not in the costmap
   * @throws nav2_core::StartOccupied If the start is in lethal cost
   */
  unsigned int associatePoseWithGraphNode(
    std::vector<unsigned int> node_indices,
    geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Visualize the search expansions
   * @param occ_grid_pub A occupancy grid publisher to view the expansions
   */
  void visualizeExpansions(
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub);

  rclcpp::Logger logger_{rclcpp::get_logger("GoalIntentExtractor")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<NodeSpatialTree> node_spatial_tree_;
  GraphToIDMap * id_to_graph_map_;
  Graph * graph_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string route_frame_;
  std::string base_frame_;
  std::string costmap_frame_;
  geometry_msgs::msg::PoseStamped start_, goal_;
  bool prune_goal_;
  float max_dist_from_edge_, min_dist_from_goal_, min_dist_from_start_;

  bool enable_search_;
  bool use_closest_node_on_search_failure_;
  bool enable_search_viz_;
  int max_iterations_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<DijkstraSearch> ds_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr start_expansion_viz_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr goal_expansion_viz_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr route_start_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr route_goal_pose_pub_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__GOAL_INTENT_EXTRACTOR_HPP_
