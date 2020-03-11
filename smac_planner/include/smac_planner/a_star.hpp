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
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "smac_planner/node.hpp"
#include "smac_planner/types.hpp"

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
  explicit AStarAlgorithm(const Neighborhood & neighborhood);
  ~AStarAlgorithm();

  void initialize(
    const float & travel_cost,
    const bool & allow_unknown,
    const float & max_iterations);

  bool createPath(IndexPath & path);

  void setCosts(
    const unsigned int & x,
    const unsigned int & y,
    unsigned char * costs);

  void setGoal(const unsigned int & value);

  void setStart(const unsigned int & value);

  bool backtracePath(Node * node, IndexPath & path);

private:
  Node * & getStart();

  Node * & getGoal();

  Node * getNode();

  void addNode(const float cost, Node * & node);

  bool isGoal(const Node * node);

  bool isCellValid(const unsigned int & i, Graph::iterator & cell_it);

  NodeVector getValidCells(
    const std::vector<int> & lookup_table,
    const unsigned int & cell);

  float & getCellCost(const unsigned int & cell);

  float & getTraversalCost(const unsigned int & lastCell, const unsigned int & cell);

  float getHeuristicCost(const unsigned int & cell);

  NodeVector getNeighbors(const unsigned int & cell);

  bool areInputsValid();

  unsigned int & getMaxIterations() {return max_iterations_;}
  unsigned int & getSizeX() {return x_size_;}
  unsigned int & getSizeY() {return y_size_;}
  std::pair<unsigned int, unsigned int> getCoords(const unsigned int index)
  {
    const unsigned int x = index % getSizeX();
    const unsigned int y = index / getSizeX();
    return std::pair<unsigned int, unsigned int>(x, y);
  }

  float travel_cost_;
  bool traverse_unknown_;
  unsigned int max_iterations_;
  unsigned int x_size_;
  unsigned int y_size_;

  Coordinates goal_coordinates_;
  Node * start_;
  Node * goal_;

  std::unique_ptr<Graph> graph_;
  std::unique_ptr<NodeQueue> queue_;

  Neighborhood neighborhood_;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__A_STAR_HPP_
