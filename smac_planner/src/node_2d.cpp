// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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
// limitations under the License. Reserved.

#include "smac_planner/node_2d.hpp"

#include <vector>
#include <limits>

namespace smac_planner
{

// defining static member for all instance to share
std::vector<int> Node2D::_neighbors_grid_offsets;
double Node2D::neutral_cost = 50.0;

Node2D::Node2D(unsigned char & cost_in, const unsigned int index)
: parent(nullptr),
  _cell_cost(static_cast<float>(cost_in)),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _is_queued(false)
{
}

Node2D::~Node2D()
{
  parent = nullptr;
}

void Node2D::reset(const unsigned char & cost)
{
  parent = nullptr;
  _cell_cost = static_cast<float>(cost);
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _is_queued = false;
}

bool Node2D::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker /*collision_checker*/)
{
  // NOTE(stevemacenski): Right now, we do not check if the node has wrapped around
  // the regular grid (e.g. your node is on the edge of the costmap and i+1
  // goes to the other side). This check would add compute time and my assertion is
  // that if you do wrap around, the heuristic will be so high it'll be added far
  // in the queue that it will never be called if a valid path exists.
  // This is intentionally un-included to increase speed, but be aware. If this causes
  // trouble, please file a ticket and we can address it then.

  auto & cost = this->getCost();

  // occupied node
  if (cost == OCCUPIED || cost == INSCRIBED) {
    return false;
  }

  // unknown node
  if (cost == UNKNOWN && !traverse_unknown) {
    return false;
  }

  return true;
}

float Node2D::getTraversalCost(const NodePtr & child)
{
  // cost to travel will be the cost of the cell's code

  // neutral_cost is neutral cost for cost just to travel anywhere (50)
  // 0.8 is a scale factor to remap costs [0, 252] evenly from [50, 252]
  return Node2D::neutral_cost + 0.8 * child->getCost();
}

float Node2D::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coordinates)
{
  return hypotf(
    goal_coordinates.x - node_coords.x,
    goal_coordinates.y - node_coords.y) * Node2D::neutral_cost;
}

void Node2D::initNeighborhood(
  const unsigned int & x_size_uint,
  const MotionModel & neighborhood)
{
  int x_size = static_cast<int>(x_size_uint);
  switch (neighborhood) {
    case MotionModel::UNKNOWN:
      throw std::runtime_error("Unknown neighborhood type selected.");
    case MotionModel::VON_NEUMANN:
      _neighbors_grid_offsets = {-1, +1, -x_size, +x_size};
      break;
    case MotionModel::MOORE:
      _neighbors_grid_offsets = {-1, +1, -x_size, +x_size, -x_size - 1,
        -x_size + 1, +x_size - 1, +x_size + 1};
      break;
    default:
      throw std::runtime_error(
              "Invalid neighborhood type selected. "
              "Von-Neumann and Moore are valid for Node2D.");
  }
}

void Node2D::getNeighbors(
  NodePtr & node,
  std::function<bool(const unsigned int &, smac_planner::Node2D * &)> & NeighborGetter,
  GridCollisionChecker collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  // NOTE(stevemacenski): Irritatingly, the order here matters. If you start in free
  // space and then expand 8-connected, the first set of neighbors will be all cost
  // _neutral_cost. Then its expansion will all be 2 * _neutral_cost but now multiple
  // nodes are touching that node so the last cell to update the back pointer wins.
  // Thusly, the ordering ends with the cardinal directions for both sets such that
  // behavior is consistent in large free spaces between them.
  // 100  50   0
  // 100  50  50
  // 100 100 100   where lower-middle '100' is visited with same cost by both bottom '50' nodes
  // Therefore, it is valuable to have some low-potential across the entire map
  // rather than a small inflation around the obstacles
  int index;
  NodePtr neighbor;
  int node_i = node->getIndex();

  for (unsigned int i = 0; i != _neighbors_grid_offsets.size(); ++i) {
    index = node_i + _neighbors_grid_offsets[i];
    if (NeighborGetter(index, neighbor)) {
      if (neighbor->isNodeValid(traverse_unknown, collision_checker) && !neighbor->wasVisited()) {
        neighbors.push_back(neighbor);
      }
    }
  }
}

}  // namespace smac_planner
