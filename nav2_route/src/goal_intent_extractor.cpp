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

#include <boost/concept/detail/has_constraints.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <memory>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_route/breadth_first_search.hpp"
#include "nav2_route/goal_intent_extractor.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_route
{

static float EPSILON = 1e-6;

void GoalIntentExtractor::configure(
  nav2_util::LifecycleNode::SharedPtr node,
  Graph & graph,
  GraphToIDMap * id_to_graph_map,
  std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string & route_frame,
  const std::string & base_frame)
{
  logger_ = node->get_logger();
  id_to_graph_map_ = id_to_graph_map;
  graph_ = &graph;
  tf_ = tf;
  route_frame_ = route_frame;
  base_frame_ = base_frame;

  nav2_util::declare_parameter_if_not_declared(
    node, "prune_goal", rclcpp::ParameterValue(true));
  prune_goal_ = node->get_parameter("prune_goal").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "max_dist_from_edge", rclcpp::ParameterValue(8.0));
  max_dist_from_edge_ = static_cast<float>(node->get_parameter("max_dist_from_edge").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "min_dist_from_goal", rclcpp::ParameterValue(0.15));
  min_dist_from_goal_ = static_cast<float>(node->get_parameter("min_dist_from_goal").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "min_dist_from_start", rclcpp::ParameterValue(0.10));
  min_dist_from_start_ = static_cast<float>(node->get_parameter("min_dist_from_start").as_double());

  nav2_util::declare_parameter_if_not_declared(
    node, "num_of_nearest_nodes", rclcpp::ParameterValue(3)); 
  int num_of_nearest_nodes = node->get_parameter("num_of_nearest_nodes").as_int();

  node_spatial_tree_ = std::make_shared<NodeSpatialTree>(num_of_nearest_nodes);
  node_spatial_tree_->computeTree(graph);

  nav2_util::declare_parameter_if_not_declared(
    node, "enable_search", rclcpp::ParameterValue(false));
  enable_search_ = node->get_parameter("enable_search").as_bool(); 

  if (enable_search_) {
    std::string global_costmap_topic;
    nav2_util::declare_parameter_if_not_declared(
      node, "global_costmap_topic", 
        rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
    node->get_parameter("global_costmap_topic", global_costmap_topic);

    int max_iterations;
    nav2_util::declare_parameter_if_not_declared(
      node, "max_iterations", rclcpp::ParameterValue(500));
    max_iterations = node->get_parameter("max_iterations").as_int();

    costmap_sub_ = 
      std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, global_costmap_topic);
    bfs_ = std::make_unique<BreadthFirstSearch>();
    bfs_->initialize(costmap_sub_->getCostmap().get(), max_iterations);
  }
}

void GoalIntentExtractor::setGraph(Graph & graph, GraphToIDMap * id_to_graph_map)
{
  id_to_graph_map_ = id_to_graph_map;
  node_spatial_tree_->computeTree(graph);
}

geometry_msgs::msg::PoseStamped GoalIntentExtractor::transformPose(
  geometry_msgs::msg::PoseStamped & pose)
{
  if (pose.header.frame_id != route_frame_) {
    RCLCPP_INFO(
      logger_,
      "Request pose in %s frame. Converting to route server frame: %s.",
      pose.header.frame_id.c_str(), route_frame_.c_str());
    if (!nav2_util::transformPoseInTargetFrame(pose, pose, *tf_, route_frame_)) {
      throw nav2_core::RouteTFError("Failed to transform starting pose to: " + route_frame_);
    }
  }
  return pose;
}

void GoalIntentExtractor::setStart(const geometry_msgs::msg::PoseStamped & start_pose)
{
  start_ = start_pose;
}

template<typename GoalT>
NodeExtents
GoalIntentExtractor::findStartandGoal(const std::shared_ptr<const GoalT> goal)
{
  // If not using the poses, then use the requests Node IDs to establish start and goal
  if (!goal->use_poses) {
    unsigned int start_idx = id_to_graph_map_->at(goal->start_id);
    unsigned int goal_idx = id_to_graph_map_->at(goal->goal_id);
    const Coordinates & start_coords = graph_->at(start_idx).coords;
    const Coordinates & goal_coords = graph_->at(goal_idx).coords;
    start_.pose.position.x = start_coords.x;
    start_.pose.position.y = start_coords.y;
    goal_.pose.position.x = goal_coords.x;
    goal_.pose.position.y = goal_coords.y;
    return {start_idx, goal_idx};
  }

  // Find request start pose
  geometry_msgs::msg::PoseStamped start_pose, goal_pose = goal->goal;
  if (goal->use_start) {
    start_pose = goal->start;
  } else {
    if (!nav2_util::getCurrentPose(start_pose, *tf_, route_frame_, base_frame_)) {
      throw nav2_core::RouteTFError("Failed to obtain starting pose in: " + route_frame_);
    }
  }

  // transform to route_frame
  start_ = transformPose(start_pose);
  goal_ = transformPose(goal_pose);

  // Find closest route graph nodes to start and goal to plan between.
  // Note that these are the location indices in the graph
  std::vector<unsigned int> start_route_ids, end_route_ids;
  if (!node_spatial_tree_->findNearestGraphNodesToPose(start_, start_route_ids) ||
    !node_spatial_tree_->findNearestGraphNodesToPose(goal_, end_route_ids))
  {
    throw nav2_core::IndeterminantNodesOnGraph(
            "Could not determine node closest to start or goal pose requested!");
  }

  if (enable_search_) {
    unsigned int valid_start_route_id, valid_end_route_id;
    findValidGraphNode(start_route_ids, start_, valid_start_route_id);
    findValidGraphNode(end_route_ids, goal_, valid_end_route_id);
    return {valid_end_route_id, valid_end_route_id};
  }

  return {start_route_ids.front(), start_route_ids.front()};
}

template<typename GoalT>
Route GoalIntentExtractor::pruneStartandGoal(
  const Route & input_route,
  const std::shared_ptr<const GoalT> goal,
  ReroutingState & rerouting_info)
{
  Route pruned_route = input_route;

  // Grab and update the rerouting state
  EdgePtr last_curr_edge = rerouting_info.curr_edge;
  rerouting_info.curr_edge = nullptr;
  bool first_time = rerouting_info.first_time;
  rerouting_info.first_time = false;

  // Cannot prune if no edges to prune or if using nodeIDs (no effect)
  if (input_route.edges.empty() || (!goal->use_poses && first_time)) {
    return pruned_route;
  }

  NodePtr first = pruned_route.start_node;
  NodePtr next = pruned_route.edges[0]->end;
  float vrx = next->coords.x - first->coords.x;
  float vry = next->coords.y - first->coords.y;
  float vpx = start_.pose.position.x - first->coords.x;
  float vpy = start_.pose.position.y - first->coords.y;
  float dot_prod = utils::normalizedDot(vrx, vry, vpx, vpy);
  Coordinates closest_pt_on_edge = utils::findClosestPoint(start_, first->coords, next->coords);
  if (dot_prod > EPSILON &&  // A projection exists
    hypotf(vpx, vpy) > min_dist_from_start_ &&  // We're not on the node to prune entire edge
    utils::distance(closest_pt_on_edge, start_) <= max_dist_from_edge_)  // Close enough to edge
  {
    // Record the pruned edge information if its the same edge as previously routed so that
    // the tracker can seed this information into its state to proceed with its task losslessly
    if (last_curr_edge && last_curr_edge->edgeid == pruned_route.edges.front()->edgeid) {
      rerouting_info.closest_pt_on_edge = closest_pt_on_edge;
      rerouting_info.curr_edge = pruned_route.edges.front();
    }

    pruned_route.start_node = next;
    pruned_route.route_cost -= pruned_route.edges.front()->end->search_state.traversal_cost;
    pruned_route.edges.erase(pruned_route.edges.begin());
  }

  // Don't prune the goal if requested, if given a known goal_id (no effect), or now empty
  if (!prune_goal_ || !goal->use_poses || pruned_route.edges.empty()) {
    return pruned_route;
  }

  next = pruned_route.edges.back()->start;
  NodePtr last = pruned_route.edges.back()->end;
  vrx = last->coords.x - next->coords.x;
  vry = last->coords.y - next->coords.y;
  vpx = goal_.pose.position.x - last->coords.x;
  vpy = goal_.pose.position.y - last->coords.y;

  dot_prod = utils::normalizedDot(vrx, vry, vpx, vpy);
  closest_pt_on_edge = utils::findClosestPoint(goal_, next->coords, last->coords);
  if (dot_prod < -EPSILON &&  // A projection exists
    hypotf(vpx, vpy) > min_dist_from_goal_ &&  // We're not on the node to prune entire edge
    utils::distance(closest_pt_on_edge, goal_) <= max_dist_from_edge_)  // Close enough to edge
  {
    pruned_route.route_cost -= pruned_route.edges.back()->end->search_state.traversal_cost;
    pruned_route.edges.pop_back();
  }

  return pruned_route;
}

void GoalIntentExtractor::findValidGraphNode(std::vector<unsigned int> node_indices, 
                                             const geometry_msgs::msg::PoseStamped & pose,
                                             unsigned int & best_node_index)
{
  unsigned int s_mx, s_my, g_mx, g_my; 
  if(!costmap_sub_->getCostmap()->worldToMap(pose.pose.position.x, pose.pose.position.y, 
                                         s_mx, s_my)) {
    throw nav2_core::StartOutsideMapBounds(
      "Start Coordinates of(" + std::to_string(pose.pose.position.x) + ", " + 
      std::to_string(pose.pose.position.y) + ") was outside bounds");
  }

  if (costmap_sub_->getCostmap()->getCost(s_mx, s_my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::StartOccupied("Start was in lethal cost");
  }

  // double check the frames, probably need to move into the map frame...
  std::vector<unsigned int> valid_node_indices;
  std::vector<nav2_costmap_2d::MapLocation> goals; 
  for (const auto & node_index : node_indices) {
    float goal_x = (*graph_)[node_index].coords.x;
    float goal_y = (*graph_)[node_index].coords.y;
    if (!costmap_sub_->getCostmap()->worldToMap(goal_x, goal_y, g_mx, g_my)) {
      RCLCPP_WARN_STREAM(logger_, "Goal coordinate of(" + std::to_string(goal_x) + ", " + 
                  std::to_string(goal_y) + ") was outside bounds. Removing from goal list");
      continue;
    }

    if (costmap_sub_->getCostmap()->getCost(g_mx, g_my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN_STREAM(logger_, "Goal corrdinate of(" + std::to_string(goal_x) + ", " + 
                        std::to_string(goal_y) + ") was in lethal cost. Removing from goal list.");continue;
    }
    valid_node_indices.push_back(node_index);
    goals.push_back({g_mx, g_my});
  }

  if (goals.empty()) {
    throw nav2_core::PlannerException("All goals were invalid");
  }

  bfs_->setStart(s_mx, s_my);
  bfs_->setGoals(goals);
  if (bfs_->isNodeVisible()) {
    // The visiblity check only validates the first node in goal array
    best_node_index = valid_node_indices.front();
    return;
  }

  unsigned int goal;
  bfs_->search(goal);
  best_node_index = goal;
}

template Route GoalIntentExtractor::pruneStartandGoal<nav2_msgs::action::ComputeRoute::Goal>(
  const Route & input_route,
  const std::shared_ptr<const nav2_msgs::action::ComputeRoute::Goal> goal,
  ReroutingState & rerouting_info);
template
Route GoalIntentExtractor::pruneStartandGoal<nav2_msgs::action::ComputeAndTrackRoute::Goal>(
  const Route & input_route,
  const std::shared_ptr<const nav2_msgs::action::ComputeAndTrackRoute::Goal> goal,
  ReroutingState & rerouting_info);
template NodeExtents GoalIntentExtractor::findStartandGoal<nav2_msgs::action::ComputeRoute::Goal>(
  const std::shared_ptr<const nav2_msgs::action::ComputeRoute::Goal> goal);
template
NodeExtents GoalIntentExtractor::findStartandGoal<nav2_msgs::action::ComputeAndTrackRoute::Goal>(
  const std::shared_ptr<const nav2_msgs::action::ComputeAndTrackRoute::Goal> goal);

}  // namespace nav2_route
