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
#include <vector>
#include <unordered_map>
#include <memory>
#include <queue>
#include <algorithm>
#include <chrono>
#include <limits>

#include "smac_planner/a_star.hpp"

namespace smac_planner
{

// for sampling on 3D: planner is given a pointer to an environmental rep
//   and one of the implementations is getCost(current pose) so it can do traveribility
//   then no copying over values to the algorithm.
//   and interface should be defined in nav2_core.
//   and include some getCost(current, next) for traversibility. would just be cost in 2D

// Feature complete
// - neutral cost / costmap / minimum cost per step?
// - optimization points (also, reduce copies, different data structures for graph: vector may be faster. What about queue?, inline methods?)

// template all of AStarAlgorithm by NodeT. and then when we instantiate in plugin we give it. graph takes template arguement
  // different plugins (A*, hybrid A*, etc) are different front ends (maybe plgun-plugins for 1? that seems too much though) but share this implementation.
  // need to make API for this implementation to also take in nodes and edges to share in graph setCosts() a different alternative version, change this one to setGridCosts()

// once we start sampling, need to set (and check validity of) footprint.

AStarAlgorithm::AStarAlgorithm(const Neighborhood & neighborhood)
: travel_cost_(0.0),
  traverse_unknown_(true),
  max_iterations_(0),
  x_size_(0),
  y_size_(0),
  goal_coordinates_(Coordinates()),
  start_(nullptr),
  goal_(nullptr),
  graph_(nullptr),
  queue_(nullptr),
  neighborhood_(neighborhood)
{
}

AStarAlgorithm::~AStarAlgorithm()
{
  graph_.reset();
  queue_.reset();
  start_ = nullptr;
  goal_ = nullptr;
}

void AStarAlgorithm::initialize(
  const float & travel_cost,
  const bool & allow_unknown,
  int & max_iterations,
  const bool & revisit_neighbors,
  const int & max_on_approach_iterations)
{
  if (graph_) {
    graph_.reset();
  }

  if (queue_) {
    queue_.reset();
  }

  graph_ = std::make_unique<Graph>();
  queue_ = std::make_unique<NodeQueue>();
  travel_cost_ = travel_cost;
  traverse_unknown_ = allow_unknown;
  max_iterations_ = max_iterations;
  revisit_neighbors_ = revisit_neighbors;
  max_on_approach_iterations_ = max_on_approach_iterations;
}

void AStarAlgorithm::setCosts(
  const unsigned int & x,
  const unsigned int & y,
  unsigned char * costs)
{
  if (getSizeX() != x || getSizeY() != y) {
    x_size_ = x;
    y_size_ = y;
    graph_->clear();
    graph_->reserve(x * y);
    for (unsigned int i = 0; i != x * y; i++) {
      graph_->insert({i, Node(costs[i], i)});  // OPTIMIZATION: benchmark different methods here. emplace, reserve, resize, insert.
    }
  } else {
    for (unsigned int i = 0; i != x * y; i++) {
      graph_->at(i).reset(costs[i], i);  // OPTIMIZATION: benchmark if this actually saves me anything. Any way we can avoid the copy?
    }
  }
}

void AStarAlgorithm::setStart(const unsigned int & value)
{
  auto it = graph_->find(value);  // OPTIMIZATION: see if at with exception handles or faster
  if (it != graph_->end()) {
    start_ = &(it->second);
  } else {
    start_ = nullptr;
  }
}

void AStarAlgorithm::setGoal(const unsigned int & value)
{
  auto it = graph_->find(value);  // OPTIMIZATION: see if at with exception handles or faster
  if (it != graph_->end()) {
    goal_ = &(it->second);
    goal_coordinates_ = getCoords(goal_->getIndex());
  } else {
    goal_ = nullptr;
    goal_coordinates_ = Coordinates();
  }
}

bool AStarAlgorithm::areInputsValid()
{
  // Check if graph was filled in
  if (graph_->size() == 0) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!start_ || !goal_) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  Graph::iterator it;
  if (getTolerance() < 0.001 && !isCellValid(goal_->getIndex(), it)) {
    throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
  }

  // Check if starting point is valid
  if (!isCellValid(getStart()->getIndex(), it)) {
    throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
  }

  return true;
}

bool AStarAlgorithm::createPath(IndexPath & path, const float & tolerance)
{
  if (!areInputsValid()) {
    return false;
  }

  tolerance_ = tolerance;
  best_heuristic_node_ = {std::numeric_limits<float>::max(), 0};
  clearQueue();

  // 0) Add starting point to the open set
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0);

  // Optimization: preallocate all variables
  Node * current_node;
  float g_cost = 0.0;
  float priority = 0.0;
  NodeVector neighbors;
  int iterations = 0;
  int approach_iterations = 0;
  NodeVector::iterator neighbor_iterator;

  while (iterations < getMaxIterations() && !queue_->empty()) {
    iterations++;

    // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    current_node = getNode();

    // 2) Mark Nbest as visited
    current_node->visited();

    // 3) Check if we're at the goal, backtrace if required
    if (isGoal(current_node)) {
      return backtracePath(current_node, path);
    }
    else if (best_heuristic_node_.first < tolerance_) {
      // Optimization: Let us find when in tolerance and refine within reason
      approach_iterations++;
      if (approach_iterations > getOnApproachMaxIterations() || iterations + 1 == getMaxIterations()) {
        Node * node = & graph_->at(best_heuristic_node_.second);
        return backtracePath(node, path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    getNeighbors(current_node->getIndex(), neighbors);

    for (NodeVector::iterator neighbor_iterator = neighbors.begin();
        neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      Node * & neighbor = (*neighbor_iterator);

      if (neighbor->wasVisited() && !revisit_neighbors_) {
        // NOTE(stevemacenski): if we expanded it, we dont need to add to search
        // it again, because if heuristic is admissible and consistant, it cannot be
        // adding option in case someone messes up or has a special situation.
        continue;
      }

      // 4.1) Compute the cost to go to this cell
      g_cost = current_node->getAccumulatedCost() +
        getTraversalCost(current_node->getIndex(), neighbor->getIndex());

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->last_node = current_node;
        priority = g_cost + getHeuristicCost(neighbor->getIndex());

        // 4.3) If not in queue or visited, add it
        if (!neighbor->isQueued() && !neighbor->wasVisited()) {
          neighbor->queued();
          addNode(priority, neighbor);
        }
      }
    }
  }

  return false;
}

bool AStarAlgorithm::isGoal(Node * & node)
{
  return node == getGoal();
}

bool AStarAlgorithm::backtracePath(Node * & node, IndexPath & path)
{
  if (!node->last_node) {
    return false;
  }

  Node * current_node = node;

  while (current_node->last_node) {
    path.push_back(current_node->getIndex());
    current_node = current_node->last_node;
  }

  if (path.size() > 1) {
    return true;
  }

  return false;
}

Node * & AStarAlgorithm::getStart()
{
  return start_;
}

Node * & AStarAlgorithm::getGoal()
{
  return goal_;
}

Node * AStarAlgorithm::getNode()
{
  Node * node = queue_->top().second;
  queue_->pop();
  return node;
}

void AStarAlgorithm::addNode(const float cost, Node * & node)
{
  queue_->emplace(cost, node);  //  OPTIMIZATION: make sure this is faster than just inserting
}

float & AStarAlgorithm::getCellCost(const unsigned int & cell)
{
  return graph_->at(cell).getCost();  // TODO use cell costs in this
}  // OPTIMIZATION: try to access data structure only 1 tie for each cell. We already have it from search.

float & AStarAlgorithm::getTraversalCost(
  const unsigned int & /*lastCell*/,
  const unsigned int & /*cell*/)
{
  // Currently using a regular 2D grid, all traversal costs are the same
  return travel_cost_;  // TODO why does make thing this 10 vs 1 make it no longer plan??
}

float AStarAlgorithm::getHeuristicCost(const unsigned int & cell)
{
  Coordinates cell_coords = getCoords(cell);
  float heuristic = hypotf(
    goal_coordinates_.first - cell_coords.first,
    goal_coordinates_.second - cell_coords.second);  // * static_cast<float>(COST_NEUTRAL);

  if (heuristic < best_heuristic_node_.first) {
    best_heuristic_node_ = {heuristic, cell};
  }

  return heuristic;
}

void AStarAlgorithm::getNeighbors(const unsigned int & cell, NodeVector & neighbors)
{
  int size_x = static_cast<int>(getSizeX());  // OPTIMIZATION: have these as class variables when size changes to reduce cost
  int cell_i = static_cast<int>(cell);
  const std::vector<int> van_neumann_neighborhood = {cell_i + 1, cell_i - 1,
    cell_i - size_x, cell_i + size_x};
  const std::vector<int> moore_neighborhood = {
    cell_i + 1, cell_i - 1, cell_i - size_x, cell_i + size_x, cell_i - size_x - 1,
    cell_i - size_x + 1, cell_i + size_x - 1, cell_i + size_x + 1};

  switch (neighborhood_) {
    case Neighborhood::UNKNOWN:
      throw std::runtime_error("Unkown neighborhood type selected.");
    case Neighborhood::VAN_NEUMANN:
      getValidCells(van_neumann_neighborhood, neighbors);
      break;
    case Neighborhood::MOORE:
      getValidCells(moore_neighborhood, neighbors);
      break;
    default:
      throw std::runtime_error("Invalid neighborhood type selected.");
  }
}

void AStarAlgorithm::getValidCells(
  const std::vector<int> & lookup_table,
  NodeVector & neighbors)
{
  Graph::iterator cell_iterator;

  for (unsigned int i = 0; i != lookup_table.size(); i++) {
    if (lookup_table[i] > 0 && isCellValid(
        static_cast<unsigned int>(lookup_table[i]), cell_iterator))
    {
      neighbors.push_back(&cell_iterator->second);
    }
  }
}

bool AStarAlgorithm::isCellValid(const unsigned int & i, Graph::iterator & cell_it)
{
  cell_it = graph_->find(i);  // OPTIMIZATION: is 'at(i)' faster?

  // out of range
  if (cell_it == graph_->end()) {
    return false;
  }

  // NOTE(stevemacenski): Right now, we do not check if the cell has wrapped around
  // the regular grid (e.g. your cell is on the edge of the costmap and i+1 
  // goes to the other side). This check would add compute time and my assertion is
  // that if you do wrap around, the heuristic will be so high it'll be added far
  // in the queue that it will never be called if a valid path exists.
  // This is intentionally un-included to increase speed, but be aware. If this causes
  // trouble, please file a ticket and we can address it then.

  // occupied cell
  auto & cost = cell_it->second.getCost();
  if (cost == OCCUPIED || cost == INSCRIBED) {
    return false;
  }

  // unknown cell
  if (cost == UNKNOWN && !traverse_unknown_) {
    return false;
  }

  return true;
}

void AStarAlgorithm::clearQueue()
{
  while (!queue_->empty()) {
    queue_->pop();
  }
}

Coordinates AStarAlgorithm::getCoords(const unsigned int & index)
{
  return Coordinates(
    static_cast<float>(index % getSizeX()),
    static_cast<float>(index / getSizeX()));
}


int & AStarAlgorithm::getMaxIterations()
{
  return max_iterations_;
}

int & AStarAlgorithm::getOnApproachMaxIterations()
{
  return max_on_approach_iterations_;
}

float & AStarAlgorithm::getTolerance()
{
  return tolerance_;
}


unsigned int & AStarAlgorithm::getSizeX()
{
  return x_size_;
}

unsigned int & AStarAlgorithm::getSizeY()
{
  return y_size_;
}

}  // namespace smac_planner
