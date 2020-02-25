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

#ifndef SMAC_PLANNER__A_STAR_HPP_
#define SMAC_PLANNER__A_STAR_HPP_

#include <vector>
#include <iostream>

#include "smac_planner/conversion_utils.hpp"
#include "smac_planner/pose.hpp"

namespace smac_planner
{

// https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
// https://en.wikipedia.org/wiki/A*_search_algorithm
// https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py

// add doxogen
// add tests

class AStarAlgorithm
{
public:
  AStarAlgorithm();
  ~AStarAlgorithm();

  void initialize(
    const double & travel_cost,
    const bool & allow_unknown,
    const double & max_iterations);

  bool createPath();

  void setCosts();

  void setGoal();

  void setStart();

private:
  Pose & getStart();

  Pose & getGoal();

  double getCost(cont uint cell);

  double & getCellCost(const uint & cell);

  double getTraversalCost(const uint & lastCell, const uint & cell);

  double getHeuristicCost(const uint & cell);

  int & getMaxIterations();

  std::vector<cell> getNeighbors(const uint & cell);

  double _travel_cost;
  bool _traverse_unknown;
  uint _max_iterations;
  Pose _goal, _start;

  std::unique_ptr<CostGrid> _cost_grid;
  std::unique_ptr<NodeQueue> _queue;
}

}  // namespace smac_planner

#endif  // SMAC_PLANNER__A_STAR_HPP_
