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


#include <cmath>
#include <iostream>

#include "nav2_route/node.hpp"

namespace nav2_route
{

// Defining static members to be shared
std::vector<int> Node::neighbors_grid_offsets;
unsigned int Node::max_index_;
unsigned int Node::x_size_;

Node::Node(const unsigned int index)
: index_(index)
{}

Node::~Node()
{
  parent = nullptr;
}

void Node::initMotionModel(int x_size, int y_size)
{
  max_index_ = x_size * y_size;
  x_size_ = x_size;

  neighbors_grid_offsets = {-1, +1,
    -x_size, +x_size,
    -x_size - 1, -x_size + 1,
    +x_size - 1, +x_size + 1};
}

void Node::getNeighbors(
  NodeGetter & node_getter, 
  CollisionChecker* collision_checker, 
  const bool & traverse_unknown, 
  NodeVector & neighbors)
{
  unsigned int index;
  NodePtr neighbor; 
  int node_i = static_cast<int>(getIndex());
  const Coordinates p_coord = getCoords(node_i);
  Coordinates c_coord;

  for(const int & neighbors_grid_offset : neighbors_grid_offsets)
  {
    index = node_i + neighbors_grid_offset;
    std::cout << "Index" << index << std::endl;
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

    node_getter(index, neighbor);

    if(neighbor->isNodeValid(collision_checker, traverse_unknown) ) {
      neighbors.push_back(neighbor);
    }
  }
}

bool Node::isNodeValid(CollisionChecker * collision_checker, const bool &traverse_unknown)
{
  return !collision_checker->inCollision(getIndex(), traverse_unknown);
}

bool Node::backtracePath(nav2_route::Node::CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(Node::getCoords(current_node->getIndex()));
  }

  // add the start pose
  path.push_back(Node::getCoords(current_node->getIndex()));

  return true;
}

} 
