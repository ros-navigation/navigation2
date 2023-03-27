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


#include "nav2_route/node.hpp"

namespace nav2_route
{

// Defining static members to be shared
std::vector<int> Node::neighbors_grid_offsets;


Node::Node(const unsigned int index)
: parent(nullptr),
  index_(index),
  visited_(false),
  queued_(false)
{}

Node::~Node()
{
  parent = nullptr;
}

void Node::initMotionModel(int x_size)
{
  neighbors_grid_offsets = {-1, +1,
    -x_size, +x_size,
    -x_size - 1, -x_size + 1,
    +x_size - 1, +x_size + 1};
}

bool Node::isNodeValid()
{
  return true;
}

void Node::getNeighbors(nav2_route::Node::NodeVector &)
{}

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

}  // namespace nav2_route
