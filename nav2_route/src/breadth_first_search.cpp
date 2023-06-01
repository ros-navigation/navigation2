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

namespace nav2_route
{

void BreadthFirstSearch::setCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  std::cout << "Set costmap " << std::endl;
  costmap_ = costmap;

  int x_size = static_cast<int>(costmap_->getSizeInCellsX());
  int y_size = static_cast<int>(costmap_->getSizeInCellsY());

  max_index_ = x_size * y_size;
  x_size_ = x_size;

  neighbors_grid_offsets_ = {-1, +1,
    -x_size, +x_size,
    -x_size - 1, -x_size + 1,
    +x_size - 1, +x_size + 1};
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

void BreadthFirstSearch::setGoals(std::vector<unsigned int> mxs, std::vector<unsigned int> mys)
{
  goals_.clear();
  for (unsigned int i = 0; i < (mxs.size() && mys.size()); ++i) {
    goals_.emplace_back(addToGraph(costmap_->getIndex(mxs[i], mys[i])));
  }
}

bool BreadthFirstSearch::search(unsigned int & closest_goal_index)
{
  std::queue<NodePtr> queue;

  start_->explored = true;
  queue.push(start_);

  while (!queue.empty()) {
    auto & current = queue.front();
    queue.pop();

    // Check goals
    for (const auto & goal : goals_) {
      if (current->index == goal->index) {
        closest_goal_index = current->index;
        graph_.clear();
        return true;
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

  graph_.clear();
  return false;
}

void BreadthFirstSearch::getNeighbors(unsigned int parent_index, NodeVector & neighbors)
{
  unsigned int p_mx, p_my; 
  costmap_->indexToCells(parent_index, p_mx, p_my);

  unsigned int neigbhor_index;
  for (const int & neighbors_grid_offset : neighbors_grid_offsets_) {
    // Check if index is negative
    int index = parent_index + neighbors_grid_offset;
    if(index < 0){
      continue;
    }

    neigbhor_index = static_cast<unsigned int>(index);

    unsigned int n_mx, n_my; 
    costmap_->indexToCells(neigbhor_index, n_mx, n_my);

    // Check for wrap around conditions
    if (std::fabs(static_cast<float>(p_mx) - static_cast<float>(n_mx)) > 1 || 
        std::fabs(static_cast<float>(p_my) - static_cast<float>(n_my)) > 1) {
      continue;
    }

    // Check for out of bounds
    if (neigbhor_index >= max_index_) {
      continue;
    }

    if (inCollision(neigbhor_index)) {
      continue;
    }

    auto neighbor = addToGraph(neigbhor_index);
    neighbors.push_back(neighbor);
  }
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

}  // namespace nav2_route
