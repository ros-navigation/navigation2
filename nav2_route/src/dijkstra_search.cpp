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

#include "nav2_route/dijkstra_search.hpp"

#include <queue>
#include <unordered_map>
#include <vector>
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/line_iterator.hpp"

namespace nav2_route
{

void DijkstraSearch::initialize(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, int max_iterations)
{
  costmap_ = costmap;

  int x_size = static_cast<int>(costmap_->getSizeInCellsX());
  int y_size = static_cast<int>(costmap_->getSizeInCellsY());

  max_index_ = x_size * y_size;
  x_size_ = x_size;

  neighbors_grid_offsets_ = {-1, +1, 
                    -x_size, x_size};
  diagonals_ = {-x_size - 1, -x_size + 1,
                +x_size -1, +x_size +1};

  neighbors_grid_offsets_.insert(
    neighbors_grid_offsets_.end(), diagonals_.begin(), diagonals_.end());

  max_iterations_ = max_iterations;
}

DijkstraSearch::NodePtr DijkstraSearch::addToGraph(const unsigned int index)
{
  auto iter = graph_.find(index);
  if (iter != graph_.end()) {
    return &(iter->second);
  }

  return &(graph_.emplace(index, SimpleNode(index)).first->second);
}

void DijkstraSearch::setStart(unsigned int mx, unsigned int my)
{
  start_ = addToGraph(costmap_->getIndex(mx, my));
}

void DijkstraSearch::setGoals(std::vector<nav2_costmap_2d::MapLocation> & goals)
{
  goals_.clear();
  for (const auto & goal : goals) {
    goals_.push_back(addToGraph(costmap_->getIndex(goal.x, goal.y)));
  }
}

void DijkstraSearch::search(unsigned int & goal)
{
  NodeQueue queue;
  start_->cost = 0.0f;
  queue.push(std::make_pair(start_->cost, start_));

  int iteration = 0;
  while (!queue.empty()) {
    
    auto current = queue.top().second;
    queue.pop();

    if (iteration > max_iterations_) {
      throw nav2_core::PlannerTimedOut("Exceeded maximum iterations");
    }

    for (unsigned int i = 0; i < goals_.size(); ++i) {
      if (current->index == goals_[i]->index) {
        goal = i;
        return;
      }
    }

    NodeVector neighbors;
    getNeighbors(current->index, neighbors);

    for (const auto neighbor : neighbors) {
      if (!neighbor->visited) {
        float updated_cost = current->cost + calculateCost(current->index, neighbor->index);
        if (updated_cost < neighbor->cost) {
          neighbor->cost = updated_cost;
          queue.push(std::make_pair(neighbor->cost, neighbor));
        }
      }
    }
    current->visited = true;
  }

  throw nav2_core::NoValidPathCouldBeFound("No valid path found");
}

void DijkstraSearch::getNeighbors(unsigned int parent_index, NodeVector & neighbors)
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

bool DijkstraSearch::isFirstGoalVisible()
{
  unsigned int s_mx, s_my, g_mx, g_my;
  costmap_->indexToCells(start_->index, s_mx, s_my);
  costmap_->indexToCells(goals_.front()->index, g_mx, g_my);

  for (nav2_util::LineIterator line(s_mx, s_my, g_mx, g_my); line.isValid(); line.advance()) {
    double cost = costmap_->getCost(line.getX(), line.getY());
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
  }
  graph_.clear();
  return true;
}

bool DijkstraSearch::inCollision(unsigned int index)
{
  unsigned char cost = costmap_->getCost(index);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return true;
  }
  return false;
}


float DijkstraSearch::calculateCost(unsigned int current_index, unsigned int neighbor_index)
{
  auto diff = static_cast<int>(neighbor_index) - static_cast<int>(current_index);
  for (const auto & offset : diagonals_) {
    if (diff == offset) {
      return 14.0f;
    }
  }
  return 10.0f;
}
  
void DijkstraSearch::clearGraph()
{
  graph_.clear();
}

std::unordered_map<unsigned int, DijkstraSearch::SimpleNode> * DijkstraSearch::getGraph()
{
  return &graph_;
}

}  // namespace nav2_route
