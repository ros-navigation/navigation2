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

#ifndef NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_
#define NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>

#include "ompl/base/StateSpace.h"

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"

namespace nav2_smac_planner
{
// TODO update readme param list

// TODO add Lattice new planner plugin / xml

// TODO add fields to SearchInfo for Lattice
		// num prims
		// range of angular inputs valid since turning rad isn't the right one anymore.
	  // add to motion model diff/omni + update in string conversion functions in constants.hpp

// TODO test coverage

// Must forward declare
class NodeLattice;
class NodeHybrid;

/**
 * @struct nav2_smac_planner::LatticeMotionTable
 * @brief A table of motion primitives and related functions
 */
struct LatticeMotionTable
{
  /**
   * @brief A constructor for nav2_smac_planner::LatticeMotionTable
   */
  LatticeMotionTable() {}

  /**
   * @brief Initializing state lattice planner's motion model
   * @param size_x_in Size of costmap in X
   * @param size_y_in Size of costmap in Y
   * @param angle_quantization_in Size of costmap in bin sizes
   * @param search_info Parameters for searching
   */
  void initMotionModel(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  /**
   * @brief Get projections of motion models
   * @param node Ptr to NodeLattice
   * @return A set of motion poses
   */
  MotionPoses getProjections(const NodeLattice * node);

  MotionPoses projections;
  unsigned int size_x;
  unsigned int num_angle_quantization;
  float num_angle_quantization_float;
  float min_turning_radius;
  float bin_size;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  ompl::base::StateSpacePtr state_space;
  std::vector<std::vector<double>> delta_xs;
  std::vector<std::vector<double>> delta_ys;
};

/**
 * @class nav2_smac_planner::NodeLattice
 * @brief NodeLattice implementation for graph, Hybrid-A*
 */
class NodeLattice
{
public:
  typedef NodeLattice * NodePtr;
  typedef std::unique_ptr<std::vector<NodeLattice>> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef NodeHybrid::Coordinates Coordinates;
  typedef NodeHybrid::CoordinateVector CoordinateVector;

  /**
   * @brief A constructor for nav2_smac_planner::NodeLattice
   * @param index The index of this node for self-reference
   */
  explicit NodeLattice(const unsigned int index);

  /**
   * @brief A destructor for nav2_smac_planner::NodeLattice
   */
  ~NodeLattice();

  /**
   * @brief operator== for comparisons
   * @param NodeLattice right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const NodeLattice & rhs)
  {
    return this->_index == rhs._index;
  }

  /**
   * @brief setting continuous coordinate search poses (in partial-cells)
   * @param Pose pose
   */
  inline void setPose(const Coordinates & pose_in)
  {
    pose = pose_in;
  }

  /**
   * @brief Reset method for new search
   */
  void reset();

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  inline float & getAccumulatedCost()
  {
    return _accumulated_cost;
  }

  /**
   * @brief Sets the accumulated cost at this node
   * @param reference to accumulated cost
   */
  inline void setAccumulatedCost(const float & cost_in)
  {
    _accumulated_cost = cost_in;
  }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  inline float & getCost()
  {
    return _cell_cost;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @param If cell was visited
   */
  inline bool & wasVisited()
  {
    return _was_visited;
  }

  /**
   * @brief Sets if cell has been visited in search
   */
  inline void visited()
  {
    _was_visited = true;
    _is_queued = false;
  }

  /**
   * @brief Gets if cell is currently queued in search
   * @param If cell was queued
   */
  inline bool & isQueued()
  {
    return _is_queued;
  }

  /**
   * @brief Sets if cell is currently queued in search
   */
  inline void queued()
  {
    _is_queued = true;
  }

  /**
   * @brief Gets cell index
   * @return Reference to cell index
   */
  inline unsigned int & getIndex()
  {
    return _index;
  }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @return whether this node is valid and collision free
   */
  bool isNodeValid(const bool & traverse_unknown, GridCollisionChecker & collision_checker);

  /**
   * @brief Get traversal cost of parent node to child node
   * @param child Node pointer to child
   * @return traversal cost
   */
  float getTraversalCost(const NodePtr & child);

  /**
   * @brief Get index at coordinates
   * @param x X coordinate of point
   * @param y Y coordinate of point
   * @param angle Theta coordinate of point
   * @return Index
   */
  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle)
  {
    return NodeHybrid::getIndex(
    	x, y, angle, motion_table.size_x,
      motion_table.num_angle_quantization);
  }

  /**
   * @brief Get coordinates at index
   * @param index Index of point
   * @param width Width of costmap
   * @param angle_quantization Theta size of costmap
   * @return Coordinates
   */
  static inline Coordinates getCoords(
    const unsigned int & index,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    return NodeHybrid::Coordinates(
      (index / angle_quantization) % width,    // x
      index / (angle_quantization * width),    // y
      index % angle_quantization);    // theta
  }

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  static float getHeuristicCost(
    const Coordinates & node_coords,
    const Coordinates & goal_coordinates);

  /**
   * @brief Initialize motion models
   * @param motion_model Motion model enum to use
   * @param size_x Size of X of graph
   * @param size_y Size of y of graph
   * @param angle_quantization Size of theta bins of graph
   * @param search_info Search info to use
   */
  static void initMotionModel(
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & angle_quantization,
    SearchInfo & search_info);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param node Pointer to the node we are currently exploring in A*
   * @param validity_checker Functor for state validity checking
   * @param neighbors Vector of neighbors to be filled
   */
  static void getNeighbors(
    const NodePtr & node,
    std::function<bool(const unsigned int &, nav2_smac_planner::NodeLattice * &)> & validity_checker,
    GridCollisionChecker & collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  NodeLattice * parent;
  Coordinates pose;
  static double neutral_cost;  // TODO, these might not be the same length?
  static LatticeMotionTable motion_table;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  bool _is_queued;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_
