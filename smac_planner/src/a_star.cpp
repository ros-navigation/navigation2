// Copyright (c) 2020, Samsung Research America
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

#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <limits>

#include "smac_planner/a_star.hpp"

namespace smac_planner
{

template<typename NodeT>
AStarAlgorithm<NodeT>::AStarAlgorithm(const Neighborhood & neighborhood)
: _travel_cost_scale(0.0),
  _neutral_cost(0.0),
  _traverse_unknown(true),
  _max_iterations(0),
  _x_size(0),
  _y_size(0),
  _goal_coordinates(Coordinates()),
  _start(nullptr),
  _goal(nullptr),
  _graph(nullptr),
  _queue(nullptr),
  _neighborhood(neighborhood)
{
}

template<typename NodeT>
AStarAlgorithm<NodeT>::~AStarAlgorithm()
{
  _graph.reset();
  _queue.reset();
  _start = nullptr;
  _goal = nullptr;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::initialize(
  const float & travel_cost_scale,
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations)
{
  if (_graph) {
    _graph.reset();
  }

  if (_queue) {
    _queue.reset();
  }

  _graph = std::make_unique<Graph>();
  _queue = std::make_unique<NodeQueue>();
  _travel_cost_scale = travel_cost_scale;
  _neutral_cost = 253.0 * (1.0 - _travel_cost_scale);
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setCosts(
  const unsigned int & x,
  const unsigned int & y,
  unsigned char * & costs)
{
  if (getSizeX() != x || getSizeY() != y) {
    _x_size = x;
    _y_size = y;
    int x_size_int = static_cast<int>(_x_size);
    initNeighborhoods(x_size_int, _neighborhood);
    _graph->clear();
    _graph->reserve(x * y);
    for (unsigned int i = 0; i != x * y; i++) {
      _graph->emplace_back(costs[i], i);
    }
  } else {
    for (unsigned int i = 0; i != x * y; i++) {
      // Optimization: operator[] is used over at() for performance (no bound checking)
      _graph->operator[](i).reset(costs[i], i);
    }
  }
}

template <typename NodeT>
void AStarAlgorithm<NodeT>::initNeighborhoods(
  const int & x_size,
  const Neighborhood & neighborhood)
{
  switch (neighborhood) {
    case Neighborhood::UNKNOWN:
      throw std::runtime_error("Unknown neighborhood type selected.");
    case Neighborhood::VON_NEUMANN:
      _neighbors_grid_offsets = {-1, +1, -x_size, +x_size};
      break;
    case Neighborhood::MOORE:
      _neighbors_grid_offsets = {-1, +1, -x_size, +x_size, -x_size - 1,
                                 -x_size + 1, +x_size - 1, +x_size + 1};
      break;
    default:
      throw std::runtime_error("Invalid neighborhood type selected.");
  }
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setStart(const unsigned int & value)
{
  _start = & _graph->operator[](value);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setGoal(const unsigned int & value)
{
  _goal = & _graph->operator[](value);
  _goal_coordinates = getCoords(_goal->getIndex());
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::areInputsValid()
{
  // Check if initialization was called
  if (!_graph) {
    throw std::runtime_error("Failed to compute path, initialization not called.");
  }

  // Check if graph was filled in
  if (_graph->empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!_start || !_goal) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  if (getToleranceHeuristic() < 0.001 && !_goal->isNodeValid(_traverse_unknown)) {
    throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
  }

  // Check if starting point is valid
  if (!_start->isNodeValid(_traverse_unknown)) {
    throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
  }

  return true;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::createPath(IndexPath & path, int & iterations, const float & tolerance)
{  
  if (!areInputsValid()) {
    return false;
  }

  _tolerance = _neutral_cost * tolerance;
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
  clearQueue();

  // 0) Add starting point to the open set
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0);

  // Optimization: preallocate all variables
  NodePtr current_node;
  float g_cost;
  NodeVector neighbors;
  int approach_iterations = 0;
  typename NodeVector::iterator neighbor_iterator;

  while (iterations < getMaxIterations() && !_queue->empty()) {

    // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    current_node = getNode();

    // We allow for nodes to be queued multiple times in case
    // shorter paths result in it, but we can visit only once
    if (current_node->wasVisited()) {
      continue;
    }

    iterations++;

    // 2) Mark Nbest as visited
    current_node->visited();

    // 3) Check if we're at the goal, backtrace if required
    if (isGoal(current_node)) {
      return backtracePath(current_node, path);
    }
    else if (_best_heuristic_node.first < getToleranceHeuristic()) {
      // Optimization: Let us find when in tolerance and refine within reason
      approach_iterations++;
      if (approach_iterations > getOnApproachMaxIterations() ||
        iterations + 1 == getMaxIterations())
      {
        NodePtr node = & _graph->operator[](_best_heuristic_node.second);
        return backtracePath(node, path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    getNeighbors(current_node, neighbors);

    for (neighbor_iterator = neighbors.begin();
      neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      NodePtr & neighbor = * neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      g_cost = current_node->getAccumulatedCost() +
        getTraversalCost(current_node, neighbor);

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;

        // 4.3) If not in queue or visited, add it
        if (!neighbor->wasVisited()) {
          neighbor->queued();
          addNode(g_cost + getHeuristicCost(neighbor->getIndex()), neighbor);
        }
      }
    }
  }

  return false;
}

// Specialized method getNeighbors for 2D nodes
template <>
void AStarAlgorithm<Node>::getNeighbors(NodePtr & node, NodeVector & neighbors)
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

  for(unsigned int i = 0; i != _neighbors_grid_offsets.size(); ++i) {
    index = node_i + _neighbors_grid_offsets[i];
    if (index > 0)
    {
      neighbor = & _graph->operator[](index);
      if (neighbor->isNodeValid(_traverse_unknown))
      {
        neighbors.push_back(neighbor);
      }
    }
  }
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::isGoal(NodePtr & node)
{
  return node == getGoal();
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::backtracePath(NodePtr & node, IndexPath & path)
{
  if (!node->parent) {
    return false;
  }

  NodePtr current_node = node;

  while (current_node->parent) {
    path.push_back(current_node->getIndex());
    current_node = current_node->parent;
  }

  return path.size() > 1;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getStart()
{
  return _start;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getGoal()
{
  return _goal;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::getNode()
{
  NodePtr node = _queue->top().second;
  _queue->pop();
  return node;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::addNode(const float cost, NodePtr & node)
{
  _queue->emplace(cost, node);
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getTraversalCost(
  NodePtr & current_node,
  NodePtr & new_node)
{
  float & node_cost = new_node->getCost();

  // rescale cost quadratically, makes search more convex
  // Higher the scale, the less cost for lengthwise expansion
  return _neutral_cost + _travel_cost_scale * node_cost * node_cost;
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getHeuristicCost(const unsigned int & node)
{
  Coordinates node_coords = getCoords(node);
  float heuristic = hypotf(
    _goal_coordinates.first - node_coords.first,
    _goal_coordinates.second - node_coords.second) * _neutral_cost;
  
  // If we're far from goal, we want to ensure we can speed it along
  if (heuristic > getToleranceHeuristic()) {
    heuristic *= _neutral_cost;
  }

  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node};
  }

  return heuristic;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearQueue()
{
  while (!_queue->empty()) {
    _queue->pop();
  }
}

template<typename NodeT>
Coordinates AStarAlgorithm<NodeT>::getCoords(const unsigned int & index)
{
  return Coordinates(
    static_cast<float>(index % getSizeX()),
    static_cast<float>(index / getSizeX()));
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getMaxIterations()
{
  return _max_iterations;
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getOnApproachMaxIterations()
{
  return _max_on_approach_iterations;
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getToleranceHeuristic()
{
  return _tolerance;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeX()
{
  return _x_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeY()
{
  return _y_size;
}

// Instantiate AStartAlgorithm for the supported template type parameters
// This is needed to prevent "undefined symbol" errors at runtime.
template class AStarAlgorithm<Node>;

}  // namespace smac_planner
