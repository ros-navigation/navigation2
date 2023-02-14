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

#include <string>
#include <memory>
#include <vector>

#include "nav2_route/goal_intent_extractor.hpp"

namespace nav2_route
{

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
  tf_ = tf;
  route_frame_ = route_frame;
  base_frame_ = base_frame;
  node_spatial_tree_ = std::make_shared<NodeSpatialTree>();
  node_spatial_tree_->computeTree(graph);
}

void GoalIntentExtractor::setGraph(Graph & graph, GraphToIDMap * id_to_graph_map)
{
  id_to_graph_map_ = id_to_graph_map;
  node_spatial_tree_->computeTree(graph);
}

void GoalIntentExtractor::transformPose(geometry_msgs::msg::PoseStamped & pose)
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
}

template<typename GoalT>
NodeExtents
GoalIntentExtractor::findStartandGoal(const std::shared_ptr<const GoalT> goal)
{
  // If not using the poses, then use the requests Node IDs to establish start and goal
  if (!goal->use_poses) {
    return {id_to_graph_map_->at(goal->start_id), id_to_graph_map_->at(goal->goal_id)};
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

  // transform start or goal to route_frame, store for pruning
  transformPose(start_pose);
  start_ = start_pose;
  transformPose(goal_pose);
  goal_ = goal_pose;

  // Find closest route graph nodes to start and goal to plan between.
  // Note that these are the location indices in the graph
  unsigned int start_route = 0, end_route = 0;
  if (!node_spatial_tree_->findNearestGraphNodeToPose(start_pose, start_route) ||
    !node_spatial_tree_->findNearestGraphNodeToPose(goal_pose, end_route))
  {
    throw nav2_core::IndeterminantNodesOnGraph(
            "Could not determine node closest to start or goal pose requested!");
  }

  return {start_route, end_route};
}

template<typename GoalT>
Route GoalIntentExtractor::pruneStartandGoal(
  const Route & input_route, const std::shared_ptr<const GoalT> goal)
{
  Route pruned_route = input_route;
  if (input_route.edges.empty() || !goal->use_poses) {
    return pruned_route;
  }

  NodePtr first = pruned_route.start_node;
  NodePtr next = pruned_route.edges[0]->end;
  float vrx = next->coords.x - first->coords.x;
  float vry = next->coords.y - first->coords.y;
  float vpx = start_.pose.position.x - first->coords.x;
  float vpy = start_.pose.position.y - first->coords.y;
  if (utils::normalizedDot(vrx, vry, vpx, vpy) > 0.0) {
    pruned_route.start_node = next;
    pruned_route.route_cost -= pruned_route.edges.front()->end->search_state.traversal_cost;
    pruned_route.edges.erase(pruned_route.edges.begin());
  }

  next = pruned_route.edges.back()->start;
  NodePtr last = pruned_route.edges.back()->end;
  vrx = last->coords.x - next->coords.x;
  vry = last->coords.y - next->coords.y;
  vpx = goal_.pose.position.x - last->coords.x;
  vpy = goal_.pose.position.y - last->coords.y;
  if (utils::normalizedDot(vrx, vry, vpx, vpy) < 0.0) {
    pruned_route.route_cost -= pruned_route.edges.back()->end->search_state.traversal_cost;
    pruned_route.edges.pop_back();
  }

  return pruned_route;
}

template Route GoalIntentExtractor::pruneStartandGoal<nav2_msgs::action::ComputeRoute::Goal>(
  const Route & input_route,
  const std::shared_ptr<const nav2_msgs::action::ComputeRoute::Goal> goal);
template
Route GoalIntentExtractor::pruneStartandGoal<nav2_msgs::action::ComputeAndTrackRoute::Goal>(
  const Route & input_route,
  const std::shared_ptr<const nav2_msgs::action::ComputeAndTrackRoute::Goal> goal);
template NodeExtents GoalIntentExtractor::findStartandGoal<nav2_msgs::action::ComputeRoute::Goal>(
  const std::shared_ptr<const nav2_msgs::action::ComputeRoute::Goal> goal);
template
NodeExtents GoalIntentExtractor::findStartandGoal<nav2_msgs::action::ComputeAndTrackRoute::Goal>(
  const std::shared_ptr<const nav2_msgs::action::ComputeAndTrackRoute::Goal> goal);

}  // namespace nav2_route
