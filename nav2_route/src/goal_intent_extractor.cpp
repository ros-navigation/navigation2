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
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
  const std::string & route_frame,
  const std::string & global_frame,
  const std::string & base_frame)
{
  logger_ = node->get_logger();
  id_to_graph_map_ = id_to_graph_map;
  graph_ = &graph;
  tf_ = tf;
  costmap_subscriber_ = costmap_subscriber;
  route_frame_ = route_frame;
  base_frame_ = base_frame;
  global_frame_ = global_frame;
  node_spatial_tree_ = std::make_shared<NodeSpatialTree>();
  node_spatial_tree_->computeTree(graph);

  nav2_util::declare_parameter_if_not_declared(
    node, "prune_goal", rclcpp::ParameterValue(true));
  prune_goal_ = node->get_parameter("prune_goal").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "max_prune_dist_from_edge", rclcpp::ParameterValue(8.0));
  max_dist_from_edge_ = static_cast<float>(
    node->get_parameter("max_prune_dist_from_edge").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "min_prune_dist_from_goal", rclcpp::ParameterValue(0.15));
  min_dist_from_goal_ = static_cast<float>(
    node->get_parameter("min_prune_dist_from_goal").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "min_prune_dist_from_start", rclcpp::ParameterValue(0.10));
  min_dist_from_start_ = static_cast<float>(
    node->get_parameter("min_prune_dist_from_start").as_double());

  nav2_util::declare_parameter_if_not_declared(
    node, "enable_nn_search", rclcpp::ParameterValue(true));
  enable_search_ = node->get_parameter("enable_nn_search").as_bool();
  nav2_util::declare_parameter_if_not_declared(
    node, "max_nn_search_iterations", rclcpp::ParameterValue(10000));
  max_nn_search_iterations_ = node->get_parameter("max_nn_search_iterations").as_int();

  nav2_util::declare_parameter_if_not_declared(
    node, "num_nearest_nodes", rclcpp::ParameterValue(5));
  int num_of_nearest_nodes = node->get_parameter("num_nearest_nodes").as_int();
  node_spatial_tree_->setNumOfNearestNodes(num_of_nearest_nodes);
}

void GoalIntentExtractor::setGraph(Graph & graph, GraphToIDMap * id_to_graph_map)
{
  id_to_graph_map_ = id_to_graph_map;
  graph_ = &graph;
  node_spatial_tree_->computeTree(graph);
}

geometry_msgs::msg::PoseStamped GoalIntentExtractor::transformPose(
  geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame)
{
  if (pose.header.frame_id != target_frame) {
    RCLCPP_INFO(
      logger_,
      "Request pose in %s frame. Converting to route server frame: %s.",
      pose.header.frame_id.c_str(), target_frame.c_str());
    if (!nav2_util::transformPoseInTargetFrame(pose, pose, *tf_, target_frame)) {
      throw nav2_core::RouteTFError("Failed to transform starting pose to: " + target_frame);
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
  start_ = transformPose(start_pose, route_frame_);
  goal_ = transformPose(goal_pose, route_frame_);

  // Find closest route graph nodes to start and goal to plan between.
  // Note that these are the location indices in the graph
  std::vector<unsigned int> start_route, end_route;
  if (!node_spatial_tree_->findNearestGraphNodesToPose(start_, start_route) ||
    !node_spatial_tree_->findNearestGraphNodesToPose(goal_, end_route))
  {
    throw nav2_core::IndeterminantNodesOnGraph(
            "Could not determine node closest to start or goal pose requested!");
  }

  unsigned int start_route_loc = start_route.front();
  unsigned int end_route_loc = end_route.front();

  // If given cost information, check which of the nearest graph nodes is nearest by
  // traversability, not just Euclidean distance, in case of obstacles, walls, etc.
  // However, if the closest node has Line of Sight to the goal, then use that node
  // skipping the search as we know it is the closest and now optimally traversible node.
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = nullptr;
  std::string costmap_frame_id;
  bool enable_search = enable_search_;
  if (enable_search) {
    try {
      costmap = costmap_subscriber_->getCostmap();
      costmap_frame_id = global_frame_;
    } catch (const std::exception & ex) {
      enable_search = false;
      RCLCPP_WARN(
        logger_,
        "Failed to get costmap for goal intent extractor: %s. "
        "Falling back to closest euclidean route node instead.", ex.what());
    }
  }

  if (enable_search && start_route.size() > 1u) {
    // Convert the nearest node candidates to the costmap frame for search
    std::vector<geometry_msgs::msg::PoseStamped> candidate_nodes;
    candidate_nodes.reserve(start_route.size());
    for (const auto & node : start_route) {
      auto & node_data = graph_->at(node);
      geometry_msgs::msg::PoseStamped node_pose;
      node_pose.pose.position.x = node_data.coords.x;
      node_pose.pose.position.y = node_data.coords.y;
      node_pose.header.frame_id = node_data.coords.frame_id;
      node_pose.header.stamp = start_pose.header.stamp;
      candidate_nodes.push_back(transformPose(node_pose, costmap_frame_id));
    }

    auto transformed_start = transformPose(start_, costmap_frame_id);
    GoalIntentSearch::LoSCollisionChecker los_checker(costmap);
    if (los_checker.worldToMap(
      candidate_nodes.front().pose.position, transformed_start.pose.position))
    {
      if (los_checker.isInCollision()) {
        GoalIntentSearch::BreadthFirstSearch bfs(costmap);
        if (bfs.search(transformed_start, candidate_nodes, max_nn_search_iterations_)) {
          start_route_loc = start_route[bfs.getClosestNodeIdx()];
        }
      }
    }
  }

  if (enable_search && end_route.size() > 1u) {
    // Convert the nearest node candidates to the costmap frame for search
    std::vector<geometry_msgs::msg::PoseStamped> candidate_nodes;
    candidate_nodes.reserve(end_route.size());
    for (const auto & node : end_route) {
      auto & node_data = graph_->at(node);
      geometry_msgs::msg::PoseStamped node_pose;
      node_pose.pose.position.x = node_data.coords.x;
      node_pose.pose.position.y = node_data.coords.y;
      node_pose.header.frame_id = node_data.coords.frame_id;
      node_pose.header.stamp = goal_pose.header.stamp;
      candidate_nodes.push_back(transformPose(node_pose, costmap_frame_id));
    }

    auto transformed_end = transformPose(goal_, costmap_frame_id);
    GoalIntentSearch::LoSCollisionChecker los_checker(costmap);
    if (los_checker.worldToMap(
      candidate_nodes.front().pose.position, transformed_end.pose.position))
    {
      if (los_checker.isInCollision()) {
        GoalIntentSearch::BreadthFirstSearch bfs(costmap);
        if (bfs.search(transformed_end, candidate_nodes)) {
          end_route_loc = end_route[bfs.getClosestNodeIdx()];
        }
      }
    }
  }

  return {start_route_loc, end_route_loc};
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

  // Check on pruning the start node
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

  // Check on pruning the goal node
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

geometry_msgs::msg::PoseStamped GoalIntentExtractor::getStart()
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
