// Copyright (c) 2025, Open Navigation LLC
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

#include <string>
#include <memory>
#include <vector>

#include "nav2_route/goal_intent_extractor.hpp"

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
  node_spatial_tree_ = std::make_shared<NodeSpatialTree>();
  node_spatial_tree_->computeTree(graph);

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
}

void GoalIntentExtractor::setGraph(Graph & graph, GraphToIDMap * id_to_graph_map)
{
  id_to_graph_map_ = id_to_graph_map;
  graph_ = &graph;
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

void GoalIntentExtractor::overrideStart(const geometry_msgs::msg::PoseStamped & start_pose)
{
  // Override the start pose when rerouting is requested, using the current pose
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
  unsigned int start_route = 0, end_route = 0;
  if (!node_spatial_tree_->findNearestGraphNodeToPose(start_, start_route) ||
    !node_spatial_tree_->findNearestGraphNodeToPose(goal_, end_route))
  {
    throw nav2_core::IndeterminantNodesOnGraph(
            "Could not determine node closest to start or goal pose requested!");
  }

  return {start_route, end_route};
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

  // Cannot prune if no edges to prune or if using nodeIDs in the initial request (no effect)
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

geometry_msgs::msgs::PoseStamped  GoalIntentExtractor::getStart()
{
  return start_;
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
