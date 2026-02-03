// Copyright 2024 Nav2 Contributors
// Licensed under the Apache License, Version 2.0

#ifndef EXAMPLE_PLANNER__A_STAR_PLANNER_HPP_
#define EXAMPLE_PLANNER__A_STAR_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace example_planner
{

/**
 * @struct Node
 * @brief Represents a node in the A* search graph
 */
struct Node
{
  unsigned int x;
  unsigned int y;
  double g_cost;  // Cost from start
  double h_cost;  // Heuristic cost to goal
  double f_cost;  // Total cost (g + h)
  Node * parent;

  Node(unsigned int x_, unsigned int y_)
  : x(x_), y(y_), g_cost(0.0), h_cost(0.0), f_cost(0.0), parent(nullptr) {}

  bool operator>(const Node & other) const
  {
    return f_cost > other.f_cost;
  }
};

/**
 * @class AStarPlanner
 * @brief A* global path planner implementation for Nav2
 *
 * This planner implements the A* algorithm for finding optimal paths
 * on a 2D costmap. It supports 8-connected grid search and uses
 * Euclidean distance as the heuristic.
 */
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief Default constructor
   */
  AStarPlanner() = default;

  /**
   * @brief Virtual destructor
   */
  ~AStarPlanner() override = default;

  /**
   * @brief Configure the planner
   * @param parent Lifecycle node pointer
   * @param name Plugin name
   * @param tf TF buffer pointer
   * @param costmap_ros Costmap ROS wrapper pointer
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup resources
   */
  void cleanup() override;

  /**
   * @brief Activate the planner
   */
  void activate() override;

  /**
   * @brief Deactivate the planner
   */
  void deactivate() override;

  /**
   * @brief Create a path from start to goal
   * @param start Starting pose
   * @param goal Goal pose
   * @param cancel_checker Function to check for cancellation
   * @return Path from start to goal
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  /**
   * @brief Calculate heuristic cost (Euclidean distance)
   */
  double calculateHeuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

  /**
   * @brief Check if a cell is traversable
   */
  bool isTraversable(unsigned int x, unsigned int y);

  /**
   * @brief Get neighbors of a cell
   */
  std::vector<std::pair<unsigned int, unsigned int>> getNeighbors(unsigned int x, unsigned int y);

  /**
   * @brief Reconstruct path from goal to start
   */
  std::vector<std::pair<unsigned int, unsigned int>> reconstructPath(Node * goal_node);

  /**
   * @brief Convert cell index to unique key
   */
  inline size_t getKey(unsigned int x, unsigned int y)
  {
    return static_cast<size_t>(y) * costmap_->getSizeInCellsX() + x;
  }

  // Node, TF, and costmap
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Plugin name and logger
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // Frame IDs
  std::string global_frame_;

  // Parameters
  bool allow_unknown_;
  double tolerance_;
  bool use_final_approach_orientation_;
  int max_iterations_;
  double cost_factor_;
};

}  // namespace example_planner

#endif  // EXAMPLE_PLANNER__A_STAR_PLANNER_HPP_
