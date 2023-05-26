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

namespace nav2_route
{

void BreadthFirstSearch::initMotionModel(int x_size, int y_size)
{
  max_index_ = x_size * y_size;
  x_size_ = x_size;

  neighbors_grid_offsets = {-1, +1,
                            -x_size, +x_size,
                            -x_size - 1, -x_size + 1,
                            +x_size - 1, +x_size + 1};

  std::cout << "offset size " << neighbors_grid_offsets.size() << std::endl;
}

void BreadthFirstSearch::setCostmap(nav2_costmap_2d::Costmap2D *costmap)
{
  std::cout << "Set costmap " << std::endl;
  costmap_ = costmap;

  unsigned int x_size = costmap_->getSizeInCellsX();
  unsigned int y_size = costmap_->getSizeInCellsY();

  initMotionModel(static_cast<int>(x_size), static_cast<int>(y_size));
}

void BreadthFirstSearch::setStart(unsigned int mx, unsigned int my) {
  start_ = getIndex(mx, my, x_size_);
  states_.insert(std::make_pair(start_, State()));
}

void BreadthFirstSearch::setGoals(std::vector<unsigned int> mxs, std::vector<unsigned int> mys)
{
  goals_.clear();
  for(unsigned int i = 0; i < (mxs.size() && mys.size()); ++i) {
    unsigned int index = getIndex(mxs[i], mys[i], x_size_);
    goals_.push_back(index);
    states_.insert(std::make_pair(index, State()));
  }
}

bool BreadthFirstSearch::search(Coordinates & closest_goal)
{
  std::cout << "Start " << std::endl;
  states_.clear();

  // clear the queue
  std::queue<unsigned int> empty_queue;
  queue_.swap(empty_queue);

  //Add start to queue
  queue_.push(start_);

  std::cout << "Added start " << std::endl;

  while (!queue_.empty()) {

    std::cout << "Queue Size: " << queue_.size() << std::endl;

    unsigned int current = queue_.front();
    std::cout << "grab front" << std::endl;

    Coordinates current_coord = getCoords(current);
    std::cout << "Current Node: " << current_coord.x << " " << current_coord.y << std::endl;

    queue_.pop();
    states_[current].visited = true;
    states_[current].queued = false;

    // Check goals
    for(const auto &goal : goals_) {
      if (current == goal) {
        std::cout << "Found goal" << std::endl;
        closest_goal = getCoords(current);
        return true;
      }
    }
    std::cout << "Checked goals" << std::endl;

    std::vector<unsigned int> neighbors;
    getNeighbors(current, neighbors);
    std::cout << "Got neigbors" << std::endl;

    for(const auto neighbor : neighbors)
    {
      if(!states_[neighbor].queued && !states_[neighbor].visited) {
        states_[neighbor].queued = true;
        queue_.push(neighbor);

        std::cout << "Added neighbor: " << neighbor << std::endl;
      }
    }
 }
  return false;
}

bool BreadthFirstSearch::getNeighbors(unsigned int current, std::vector<unsigned int> & neighbors)
{
  const Coordinates p_coord = getCoords(current);

  unsigned int index;
  Coordinates c_coord{0, 0};

  std::cout << "grid offset size " << neighbors_grid_offsets.size() << std::endl;
  for (const int & neighbors_grid_offset : neighbors_grid_offsets)
  {
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

    if(inCollision(index)) {
      std::cout << "In collision" << std::endl;
      continue;
    }

    neighbors.push_back(index);

    auto it = states_.find(index);
    if (it == states_.end()) {
      states_.insert(std::make_pair(index, State()));
    }
  }
  return false;
}

bool BreadthFirstSearch::inCollision(unsigned int index) {
  unsigned char cost = costmap_->getCost(index);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    std::cout << "In collision" << std::endl;
    return true;
  }
  return false;
}

} // namespace nav2_route