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

  neighbors_grid_offsets = {-1, +1,
    -x_size, +x_size,
    -x_size - 1, -x_size + 1,
    +x_size - 1, +x_size + 1};

  std::cout << "offset size " << neighbors_grid_offsets.size() << std::endl;
}

BreadthFirstSearch::NodePtr BreadthFirstSearch::addToGraph(const unsigned int index)
{
  auto iter = graph_.find(index);
  if (iter != graph_.end()) {
    return &(iter->second);
  }

  return &(graph_.emplace(index, Node{index, false}).first->second);
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

bool BreadthFirstSearch::search(Coordinates & closest_goal)
{
  // clear the graph
  std::unordered_map<unsigned int, Node> empty_graph;
  std::swap(graph_, empty_graph);

  std::queue<NodePtr> queue;

  start_->visited = true;
  queue.push(start_);

  std::cout << "Start index " << start_->index << std::endl;
  std::cout << "Goal index " << goals_[0]->index << std::endl;

  int iter = 0;
  while (!queue.empty()) {
    iter += 1;
    std::cout << "Iteration " << iter << std::endl;
    std::cout << "Queue Size: " << queue.size() << std::endl;

    auto & current = queue.front();

    std::cout << "grab front" << std::endl;
    std::cout << std::endl;

    Coordinates current_coord = getCoords(current->index);
    std::cout << "CURRENT NODE: " << current_coord.x << " " << current_coord.y << std::endl;

    queue.pop();

    // Check goals
    for (const auto & goal : goals_) {
      std::cout << "Goal index " << goal->index << std::endl;
      std::cout << "current index " << current->index << std::endl;
      if (current->index == goal->index) {
        std::cout << "Found goal" << std::endl;
        closest_goal = getCoords(current->index);
        return true;
      }
    }
    std::cout << "Checked goals" << std::endl;

    NodeVector neighbors;
    getNeighbors(current->index, neighbors);
    std::cout << "Got neigbors" << std::endl;

    for (const auto neighbor : neighbors) {
      if (!neighbor->visited) {
        neighbor->visited = true;
        queue.push(neighbor);

        std::cout << "Added node: " << neighbor->index << std::endl;
      }
    }
  }
  return false;
}

bool BreadthFirstSearch::getNeighbors(unsigned int current, NodeVector & neighbors)
{
  const Coordinates p_coord = getCoords(current);

  unsigned int index;
  Coordinates c_coord{0, 0};

  std::cout << "grid offset size " << neighbors_grid_offsets.size() << std::endl;
  for (const int & neighbors_grid_offset : neighbors_grid_offsets) {
    std::cout << "Getting neighbors" << std::endl;
    index = current + neighbors_grid_offset;

    c_coord = getCoords(index);

    // Check for wrap around conditions
    if (std::fabs(p_coord.x - c_coord.x) > 1 || std::fabs(p_coord.y - c_coord.y) > 1) {
      std::cout << "Index wraps" << std::endl;
      continue;
    }

    // Check for out of bounds
    if (index >= max_index_) {
      std::cout << "Max index hit" << std::endl;
      continue;
    }

    if (inCollision(index)) {
      std::cout << "In collision" << std::endl;
      continue;
    }

    std::cout << "Adding neighbor to vector" << std::endl;
    auto neighbor = addToGraph(index);
    neighbors.push_back(neighbor);
  }
  return false;
}

bool BreadthFirstSearch::inCollision(unsigned int index)
{
  unsigned char cost = costmap_->getCost(index);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    std::cout << "In collision" << std::endl;
    return true;
  }
  return false;
}

}  // namespace nav2_route
