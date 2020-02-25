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

#include <vector>
#include "nav2_smac_planner/a_star.hpp"

namespace smac_planner
{


AStarAlgorithm::AStarAlgorithm()
: _travel_cost(10.0), _traverse_unknown(true), _max_iterations(10000)
{

}

AStarAlgorithm::~AStarAlgorithm()
{
  _cost_grid.reset();
}

void AStarAlgorithm::initialize(
  const double & travel_cost,
  const bool & allow_unknown,
  const double & max_iterations)
{
  _travelCost = travelCost;
  _traverseUnknown = allowUnknown;
  _max_iterations = max_iterations;
}

bool AStarAlgorithm::createPath()
{
  if (/*costmap size 0, no goal, start, or footprint*/) {
    return false;
  }

  if (/*goal or start off map or on unkonwn and not enabled unknown*/) {
    return false;
  }

  queue_->push(0.0, & getStartNode()); // cost and starting node

  // Open and Closed sets created when grid is populated with a costmap
  // Add starting point to the open set
  getStartNode().inOpenSet(true);

  uint iterations = 0;
  while (iterations < getMaxIterations()) {
    iterations++;

    // 1) Pick Nbest from O s.t. min(f(nBest)) 
    auto * node = getNextNode();

    // 2) Remove Nbest from O, into C
    node->isOpenSet(false);
    node->isClosedSet(true);

    // 3) Check if we're at the goal
    if (isNodeGoal(node)) {
      break;
    }

    // 4) Expand neighbors of Nbest not in C
    auto neighbors = getNeighbors(node);
    for (uint i = 0; i != neighbors) {
      if (neighbors[i]->isOpenSet())
      {
        //   4.1) if Neighor in O: if lower cost, update back ptr
      } else {
        //   4.2) If not: add to O 
      }
    }
  }

  // 5) traceback path
  if (iterations < getMaxIterations() && backTrace(/*???*/)) {
    return true;
  }

  return false;
}

void AStarAlgorithm::setCosts()
{
  if (/*sizes different*/) {
    _cost_grid->resize(x, y);
  }

  for (/*each*/) {
    _cost_grid->AddNode(i, cost);
  }
}

void AStarAlgorithm::setStart()
{

}

Pose & AStarAlgorithm::getStart()
{
  return _start;
}

void AStarAlgorithm::setGoal()
{

}

Pose & AStarAlgorithm::getGoal()
{
  return _goal;
}

double AStarAlgorithm::getCost(cont uint & cell)
{
  // total cost of costs below
}

double & AStarAlgorithm::getCellCost(const uint & cell)
{
  return _cost_grid->getCost(cell);
}

double AStarAlgorithm::getTraversalCost(const uint & lastCell, const uint & cell)
{
  return _travel_cost;
}

uint & getMaxIterations()
{
  return _max_iterations;
}

double AStarAlgorithm::getHeuristicCost(const uint & cell)
{
  // distance metric
}

std::vector<cell> AStarAlgorithm::getNeighbors(const uint & cell)
{
  // 8 connected, or otherwise (if valid)
}

}  // namespace smac_planner
