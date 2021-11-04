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

#include "nav2_smac_planner/node_2d.hpp"

#include <vector>
#include <limits>

namespace nav2_smac_planner
{

// defining static member for all instance to share
std::vector<int> Node2D::_neighbors_grid_offsets;
float Node2D::cost_travel_multiplier = 2.0;

Node2D::Node2D(const unsigned int index)
: parent(nullptr),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
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

void Node2D::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _is_queued = false;
}

bool Node2D::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  if (collision_checker->inCollision(this->getIndex(), traverse_unknown)) {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float Node2D::getTraversalCost(const NodePtr & child)
{
  float normalized_cost = child->getCost() / 252.0;
  const Coordinates A = getCoords(child->getIndex());
  const Coordinates B = getCoords(this->getIndex());
  const float & dx = A.x - B.x;
  const float & dy = A.y - B.y;
  static float sqrt_2 = sqrt(2);

  // If a diagonal move, travel cost is sqrt(2) not 1.0.
  if ((dx * dx + dy * dy) > 1.05) {
    return sqrt_2 + cost_travel_multiplier * normalized_cost;
  }

  return 1.0 + cost_travel_multiplier * normalized_cost;
}

float Node2D::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coordinates,
  const nav2_costmap_2d::Costmap2D * /*costmap*/)
{
  // Using Moore distance as it more accurately represents the distances
  // even a Van Neumann neighborhood robot can navigate.
  return fabs(goal_coordinates.x - node_coords.x) +
         fabs(goal_coordinates.y - node_coords.y);
}

void Node2D::initMotionModel(
  const MotionModel & neighborhood,
  unsigned int & x_size_uint,
  unsigned int & /*size_y*/,
  unsigned int & /*num_angle_quantization*/,
  SearchInfo & search_info)
{
  int x_size = static_cast<int>(x_size_uint);
  cost_travel_multiplier = search_info.cost_penalty;
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
  std::function<bool(const unsigned int &, nav2_smac_planner::Node2D * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  // NOTE(stevemacenski): Irritatingly, the order here matters. If you start in free
  // space and then expand 8-connected, the first set of neighbors will be all cost
  // 1.0. Then its expansion will all be 2 * 1.0 but now multiple
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
  int node_i = this->getIndex();
  const Coordinates parent = getCoords(this->getIndex());
  Coordinates child;

  for (unsigned int i = 0; i != _neighbors_grid_offsets.size(); ++i) {
    index = node_i + _neighbors_grid_offsets[i];

    // Check for wrap around conditions
    child = getCoords(index);
    if (fabs(parent.x - child.x) > 1 || fabs(parent.y - child.y) > 1) {
      continue;
    }

    if (NeighborGetter(index, neighbor)) {
      if (neighbor->isNodeValid(traverse_unknown, collision_checker) && !neighbor->wasVisited()) {
        neighbors.push_back(neighbor);
      }
    }
  }
}

}  // namespace nav2_smac_planner
