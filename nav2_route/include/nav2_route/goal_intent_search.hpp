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

#ifndef NAV2_ROUTE__GOAL_INTENT_SEARCH_HPP_
#define NAV2_ROUTE__GOAL_INTENT_SEARCH_HPP_

#include <limits>
#include <string>
#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>

#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "nav2_route/types.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_route
{

namespace GoalIntentSearch
{

/**
 * @class Find which position is nearest to a given point on a costmap using breadth-first search
 */
class BreadthFirstSearch
{
public:
  /**
   * @brief Constructor
   * @param costmap Costmap to check
   */
  explicit BreadthFirstSearch(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap)
  : costmap_(costmap)
  {
  }

  /**
   * @brief Search for the closest node to the given reference node
   * @param reference_node The reference node to search from
   * @param candidate_nodes The candidate nodes to search for
   * @param max_iterations The maximum number of iterations to perform
   * @return True if a candidate node is found, false otherwise
   */
  bool search(
    const geometry_msgs::msg::PoseStamped & reference_node,
    const std::vector<geometry_msgs::msg::PoseStamped> & candidate_nodes,
    const int max_iterations = std::numeric_limits<int>::max())
  {
    closest_node_idx_ = 0u;

    // Convert target to costmap space
    unsigned int goal_x, goal_y;
    if (!costmap_->worldToMap(reference_node.pose.position.x, reference_node.pose.position.y,
          goal_x, goal_y))
    {
      return false;
    }
    unsigned int goal_id = costmap_->getIndex(goal_x, goal_y);

    // Convert candidates to costmap space
    std::vector<unsigned int> candidate_ids;
    candidate_ids.reserve(candidate_nodes.size());
    for (const auto & node : candidate_nodes) {
      unsigned int node_x, node_y;
      if (!costmap_->worldToMap(node.pose.position.x, node.pose.position.y, node_x, node_y)) {
        // Off the costmap, consider this candidate invalid
        continue;
      }
      candidate_ids.push_back(costmap_->getIndex(node_x, node_y));
    }

    if (candidate_ids.empty()) {
      return false;
    }

    // Main Breadth-First Search
    std::queue<unsigned int> search_queue;
    std::unordered_set<unsigned int> visited;
    visited.insert(goal_id);
    search_queue.push(goal_id);
    int iterations = 0;
    while (!search_queue.empty() && iterations < max_iterations) {
      unsigned int current_id = search_queue.front();
      search_queue.pop();
      iterations++;

      // Check if the current node is a candidate
      auto iter = std::find(candidate_ids.begin(), candidate_ids.end(), current_id);
      if (iter != candidate_ids.end()) {
        closest_node_idx_ = std::distance(candidate_ids.begin(), iter);
        return true;
      }

      // Get neighbors of the current node
      std::vector<unsigned int> neighbors = getNeighbors(current_id);

      // Add unvisited neighbors to the queue
      for (const auto & neighbor : neighbors) {
        if (visited.find(neighbor) == visited.end()) {
          visited.insert(neighbor);
          search_queue.push(neighbor);
        }
      }
    }

    // No solution found with full expansion or iterations exceeded
    return false;
  }

  /**
   * @brief Get the neighbors of a given node
   * @param current_id The current node id
   * @return A vector of neighbor node ids
   */
  std::vector<unsigned int> getNeighbors(const unsigned int current_id)
  {
    int size_x = static_cast<int>(costmap_->getSizeInCellsX());
    const std::vector<int> offsets = {
      -1, 1, -size_x, size_x,
      -size_x - 1, -size_x + 1,
      size_x - 1, size_x + 1};
    int costmap_size = static_cast<int>(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());

    unsigned int p_mx, p_my;
    costmap_->indexToCells(current_id, p_mx, p_my);

    std::vector<unsigned int> neighbors;
    for (const auto & offset : offsets) {
      // If out of bounds of costmap
      int signed_neighbor_id = static_cast<int>(current_id) + offset;
      if (signed_neighbor_id < 0 || signed_neighbor_id >= costmap_size) {
        continue;
      }
      unsigned int neighbor_id = static_cast<unsigned int>(signed_neighbor_id);

      // If wrapping around the costmap
      unsigned int n_mx, n_my;
      costmap_->indexToCells(neighbor_id, n_mx, n_my);
      if (std::fabs(static_cast<float>(p_mx) - static_cast<float>(n_mx)) > 1 ||
        std::fabs(static_cast<float>(p_my) - static_cast<float>(n_my)) > 1)
      {
        continue;
      }

      // Add if not in collision
      if (!inCollision(neighbor_id)) {
        neighbors.push_back(neighbor_id);
      }
    }

    return neighbors;
  }

  /**
   * @brief Check if a node is in collision with the costmap
   * @param id The node id to check
   * @return True if the node is in collision, false otherwise
   */
  bool inCollision(const unsigned int id)
  {
    // Check if the node is in collision
    float cost = static_cast<float>(costmap_->getCost(id));
    return cost >= 253.0f && cost != 255.0f;
  }

  /**
   * @brief Get the output closest node in candidate indices
   * @return closest_node The closest candidate node index to the given point by search
   */
  unsigned int getClosestNodeIdx()
  {
    return closest_node_idx_;
  }

 /**
  * @brief Destructor
  */
  ~BreadthFirstSearch() = default;

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  unsigned int closest_node_idx_{0u};
};

/**
 * @class Find if a line segment is in collision with the costmap or not
 */
class LoSCollisionChecker
{
public:
  /**
  * @brief Constructor
  * @param costmap Costmap to check
  */
  explicit LoSCollisionChecker(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap)
  : costmap_(costmap)
  {
  }

  /**
  * @brief Destructor
  */
  ~LoSCollisionChecker() = default;

  /**
  * @brief Find the line segment in cosmap frame
  * @param start Start point
  * @param end End point
  * @return True if the line segment is on the costmap, false otherwise
  */
  bool worldToMap(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end)
  {
    if (!costmap_->worldToMap(start.x, start.y, x0_, y0_) ||
      !costmap_->worldToMap(end.x, end.y, x1_, y1_))
    {
      return false;
    }
    return true;
  }

  /**
  * @brief Check if the line segment is in collision with the costmap
  * @return True if the line segment is in collision, false otherwise
  */
  bool isInCollision()
  {
    nav2_util::LineIterator iter(x0_, y0_, x1_, y1_);
    for (; iter.isValid(); iter.advance()) {
      float cost = static_cast<float>(costmap_->getCost(iter.getX(), iter.getY()));
      if (cost >= 253.0f && cost != 255.0 /*unknown*/) {
        return true;
      }
    }
    return false;
  }

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  unsigned int x0_, x1_, y0_, y1_;
};

}  // namespace GoalIntentSearch

}  // namespace nav2_route

#endif  // NAV2_ROUTE__GOAL_INTENT_SEARCH_HPP_
