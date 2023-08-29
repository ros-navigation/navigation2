// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2023 Joshua Wallace
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

#include "nav2_route/breadth_first_search.hpp"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/line_iterator.hpp"

namespace nav2_route
{

void BreadthFirstSearch::initialize(nav2_costmap_2d::Costmap2D * costmap, int max_iterations)
{
  costmap_ = costmap;

  int x_size = static_cast<int>(costmap_->getSizeInCellsX());
  int y_size = static_cast<int>(costmap_->getSizeInCellsY());

  max_index_ = x_size * y_size;
  x_size_ = x_size;

  neighbors_grid_offsets_ = {-1, +1,
    -x_size, +x_size,
    -x_size - 1, -x_size + 1,
    +x_size - 1, +x_size + 1};

  max_iterations_ = max_iterations;
}

BreadthFirstSearch::NodePtr BreadthFirstSearch::addToGraph(const unsigned int index)
{
  auto iter = graph_.find(index);
  if (iter != graph_.end()) {
    return &(iter->second);
  }

  return &(graph_.emplace(index, SimpleNode(index)).first->second);
}

void BreadthFirstSearch::setStart(unsigned int mx, unsigned int my)
{
  start_ = addToGraph(costmap_->getIndex(mx, my));
}

void BreadthFirstSearch::setGoals(std::vector<nav2_costmap_2d::MapLocation> & goals)
{
  goals_.clear();
  for (const auto & goal : goals) {
    goals_.push_back(addToGraph(costmap_->getIndex(goal.x, goal.y)));
  }
}

void BreadthFirstSearch::search(unsigned int & goal)
{
  std::queue<NodePtr> queue;

  start_->explored = true;
  queue.push(start_);

  int iteration = 0;
  while (!queue.empty()) {
    auto & current = queue.front();
    queue.pop();

    if (iteration > max_iterations_) {
      throw nav2_core::PlannerTimedOut("Exceeded maximum iterations");
    }


    // Check goals
    for (unsigned int index = 0; index < goals_.size(); ++index) {
      if (current->index == goals_[index]->index) {
        goal = index;
        return;
      }
    }

    NodeVector neighbors;
    getNeighbors(current->index, neighbors);

    for (const auto neighbor : neighbors) {
      if (!neighbor->explored) {
        neighbor->explored = true;
        queue.push(neighbor);
      }
    }
  }

  throw nav2_core::NoValidPathCouldBeFound("No valid path found");
}

void BreadthFirstSearch::getNeighbors(unsigned int parent_index, NodeVector & neighbors)
{
  unsigned int p_mx, p_my;
  costmap_->indexToCells(parent_index, p_mx, p_my);

  unsigned int neighbor_index;
  for (const int & neighbors_grid_offset : neighbors_grid_offsets_) {
    // Check if index is negative
    int index = parent_index + neighbors_grid_offset;
    if (index < 0) {
      continue;
    }

    neighbor_index = static_cast<unsigned int>(index);

    unsigned int n_mx, n_my;
    costmap_->indexToCells(neighbor_index, n_mx, n_my);

    // Check for wrap around conditions
    if (std::fabs(static_cast<float>(p_mx) - static_cast<float>(n_mx)) > 1 ||
      std::fabs(static_cast<float>(p_my) - static_cast<float>(n_my)) > 1)
    {
      continue;
    }

    // Check for out of bounds
    if (neighbor_index >= max_index_) {
      continue;
    }

    if (inCollision(neighbor_index)) {
      continue;
    }

    neighbors.push_back(addToGraph(neighbor_index));
  }
}

bool BreadthFirstSearch::isNodeVisible()
{
  unsigned int s_mx, s_my, g_mx, g_my;
  costmap_->indexToCells(start_->index, s_mx, s_my);
  costmap_->indexToCells(goals_.front()->index, g_mx, g_my);

  for (nav2_util::LineIterator line(s_mx, s_my, g_mx, g_my); line.isValid(); line.advance()) {
    double cost = costmap_->getCost(line.getX(), line.getX());
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
  }
  graph_.clear();
  return true;
}

bool BreadthFirstSearch::inCollision(unsigned int index)
{
  unsigned char cost = costmap_->getCost(index);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return true;
  }
  return false;
}

void BreadthFirstSearch::clearGraph()
{
  graph_.clear();
}

std::unordered_map<unsigned int, SimpleNode> * BreadthFirstSearch::getGraph()
{
  return &graph_;
}

}  // namespace nav2_route
