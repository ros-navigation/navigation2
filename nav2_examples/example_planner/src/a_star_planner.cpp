// Copyright 2024 Nav2 Contributors
// Licensed under the Apache License, Version 2.0

#include "example_planner/a_star_planner.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace example_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Failed to lock parent node");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(1000000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_factor", rclcpp::ParameterValue(0.55));

  // Get parameters
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);
  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".cost_factor", cost_factor_);

  RCLCPP_INFO(
    logger_,
    "Configured A* planner '%s': allow_unknown=%d, tolerance=%.2f, max_iterations=%d",
    name_.c_str(), allow_unknown_, tolerance_, max_iterations_);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up planner: %s", name_.c_str());
  costmap_ = nullptr;
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating planner: %s", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating planner: %s", name_.c_str());
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  // Validate costmap
  if (!costmap_) {
    throw nav2_core::PlannerException("Costmap is not initialized");
  }

  // Validate frames
  if (start.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
      "Start frame '" + start.header.frame_id +
      "' does not match global frame '" + global_frame_ + "'");
  }

  if (goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
      "Goal frame '" + goal.header.frame_id +
      "' does not match global frame '" + global_frame_ + "'");
  }

  // Convert world coordinates to map coordinates
  unsigned int start_x, start_y, goal_x, goal_y;

  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_x, start_y))
  {
    throw nav2_core::PlannerException("Start pose is outside costmap bounds");
  }

  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    throw nav2_core::PlannerException("Goal pose is outside costmap bounds");
  }

  // Check start and goal are not in collision
  if (!isTraversable(start_x, start_y)) {
    throw nav2_core::PlannerException("Start position is in collision");
  }

  if (!isTraversable(goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Goal is in collision, attempting to find nearby valid goal");
    // Could implement goal tolerance search here
  }

  // Initialize A* search
  std::unordered_map<size_t, std::unique_ptr<Node>> all_nodes;
  auto compare = [](Node * a, Node * b) { return a->f_cost > b->f_cost; };
  std::priority_queue<Node *, std::vector<Node *>, decltype(compare)> open_set(compare);
  std::unordered_map<size_t, bool> closed_set;

  // Create start node
  auto start_node = std::make_unique<Node>(start_x, start_y);
  start_node->g_cost = 0.0;
  start_node->h_cost = calculateHeuristic(start_x, start_y, goal_x, goal_y);
  start_node->f_cost = start_node->g_cost + start_node->h_cost;

  Node * start_ptr = start_node.get();
  all_nodes[getKey(start_x, start_y)] = std::move(start_node);
  open_set.push(start_ptr);

  int iterations = 0;
  Node * goal_node = nullptr;

  // A* main loop
  while (!open_set.empty() && iterations < max_iterations_) {
    // Check for cancellation periodically
    if (iterations % 1000 == 0 && cancel_checker()) {
      RCLCPP_INFO(logger_, "Planning cancelled");
      return nav_msgs::msg::Path();
    }

    Node * current = open_set.top();
    open_set.pop();

    size_t current_key = getKey(current->x, current->y);

    // Skip if already processed
    if (closed_set.count(current_key)) {
      continue;
    }
    closed_set[current_key] = true;

    // Check if we reached the goal
    double dist_to_goal = std::hypot(
      static_cast<double>(current->x) - static_cast<double>(goal_x),
      static_cast<double>(current->y) - static_cast<double>(goal_y));

    if (dist_to_goal * costmap_->getResolution() <= tolerance_) {
      goal_node = current;
      break;
    }

    // Expand neighbors
    auto neighbors = getNeighbors(current->x, current->y);
    for (const auto & [nx, ny] : neighbors) {
      size_t neighbor_key = getKey(nx, ny);

      // Skip if already in closed set
      if (closed_set.count(neighbor_key)) {
        continue;
      }

      // Calculate tentative g cost
      double move_cost = (nx != current->x && ny != current->y) ? 1.414 : 1.0;
      double cell_cost = costmap_->getCost(nx, ny) * cost_factor_ / 252.0;
      double tentative_g = current->g_cost + move_cost + cell_cost;

      // Get or create neighbor node
      Node * neighbor;
      if (all_nodes.count(neighbor_key)) {
        neighbor = all_nodes[neighbor_key].get();
        if (tentative_g >= neighbor->g_cost) {
          continue;  // Not a better path
        }
      } else {
        auto new_node = std::make_unique<Node>(nx, ny);
        neighbor = new_node.get();
        all_nodes[neighbor_key] = std::move(new_node);
      }

      // Update neighbor
      neighbor->g_cost = tentative_g;
      neighbor->h_cost = calculateHeuristic(nx, ny, goal_x, goal_y);
      neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
      neighbor->parent = current;

      open_set.push(neighbor);
    }

    iterations++;
  }

  // Check if we found a path
  if (!goal_node) {
    RCLCPP_WARN(
      logger_, "Failed to find path after %d iterations (max: %d)",
      iterations, max_iterations_);
    throw nav2_core::PlannerException("Could not find valid path to goal");
  }

  RCLCPP_INFO(logger_, "Found path after %d iterations", iterations);

  // Reconstruct path
  auto cell_path = reconstructPath(goal_node);

  // Convert to world coordinates
  for (size_t i = 0; i < cell_path.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    double wx, wy;
    costmap_->mapToWorld(cell_path[i].first, cell_path[i].second, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;

    // Calculate orientation based on path direction
    if (i < cell_path.size() - 1) {
      double next_wx, next_wy;
      costmap_->mapToWorld(cell_path[i + 1].first, cell_path[i + 1].second, next_wx, next_wy);
      double yaw = std::atan2(next_wy - wy, next_wx - wx);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      // Use goal orientation for final pose
      if (use_final_approach_orientation_) {
        pose.pose.orientation = goal.pose.orientation;
      } else {
        // Use direction to goal
        pose.pose.orientation = path.poses.back().pose.orientation;
      }
    }

    path.poses.push_back(pose);
  }

  // Ensure final pose matches goal orientation
  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }

  RCLCPP_INFO(
    logger_, "Created path with %zu poses from (%.2f, %.2f) to (%.2f, %.2f)",
    path.poses.size(),
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  return path;
}

double AStarPlanner::calculateHeuristic(
  unsigned int x1, unsigned int y1,
  unsigned int x2, unsigned int y2)
{
  // Euclidean distance heuristic
  double dx = static_cast<double>(x2) - static_cast<double>(x1);
  double dy = static_cast<double>(y2) - static_cast<double>(y1);
  return std::hypot(dx, dy);
}

bool AStarPlanner::isTraversable(unsigned int x, unsigned int y)
{
  unsigned char cost = costmap_->getCost(x, y);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return false;
  }

  if (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }

  return true;
}

std::vector<std::pair<unsigned int, unsigned int>> AStarPlanner::getNeighbors(
  unsigned int x, unsigned int y)
{
  std::vector<std::pair<unsigned int, unsigned int>> neighbors;
  neighbors.reserve(8);

  // 8-connected grid
  const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();

  for (int i = 0; i < 8; ++i) {
    int nx = static_cast<int>(x) + dx[i];
    int ny = static_cast<int>(y) + dy[i];

    // Bounds check
    if (nx < 0 || ny < 0 ||
      static_cast<unsigned int>(nx) >= size_x ||
      static_cast<unsigned int>(ny) >= size_y)
    {
      continue;
    }

    unsigned int ux = static_cast<unsigned int>(nx);
    unsigned int uy = static_cast<unsigned int>(ny);

    // Traversability check
    if (isTraversable(ux, uy)) {
      neighbors.emplace_back(ux, uy);
    }
  }

  return neighbors;
}

std::vector<std::pair<unsigned int, unsigned int>> AStarPlanner::reconstructPath(Node * goal_node)
{
  std::vector<std::pair<unsigned int, unsigned int>> path;

  Node * current = goal_node;
  while (current != nullptr) {
    path.emplace_back(current->x, current->y);
    current = current->parent;
  }

  // Reverse to get start-to-goal order
  std::reverse(path.begin(), path.end());

  return path;
}

}  // namespace example_planner

// Register the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example_planner::AStarPlanner, nav2_core::GlobalPlanner)
