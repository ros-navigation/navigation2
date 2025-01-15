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

#ifndef NAV2_SMAC_PLANNER__NODE_2D_HPP_
#define NAV2_SMAC_PLANNER__NODE_2D_HPP_

#include <math.h>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <limits>
#include <utility>
#include <functional>

#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::Node2D
 * @brief Node2D implementation for graph
 */
class Node2D
{
public:
  typedef Node2D * NodePtr;
  typedef std::unique_ptr<std::vector<Node2D>> Graph;
  typedef std::vector<NodePtr> NodeVector;

  /**
   * @class nav2_smac_planner::Node2D::Coordinates
   * @brief Node2D implementation of coordinate structure
   */
  struct Coordinates
  {
    Coordinates() {}
    Coordinates(const float & x_in, const float & y_in)
    : x(x_in), y(y_in)
    {}

    float x, y;
  };
  typedef std::vector<Coordinates> CoordinateVector;

  /**
   * @brief A constructor for nav2_smac_planner::Node2D
   * @param index The index of this node for self-reference
   */
  explicit Node2D(const uint64_t index);

  /**
   * @brief A destructor for nav2_smac_planner::Node2D
   */
  ~Node2D();

  /**
   * @brief operator== for comparisons
   * @param Node2D right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const Node2D & rhs)
  {
    return this->_index == rhs._index;
  }

  /**
   * @brief Reset method for new search
   */
  void reset();
  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  inline float getAccumulatedCost()
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
  inline float getCost()
  {
    return _cell_cost;
  }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  inline void setCost(const float & cost)
  {
    _cell_cost = cost;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @param If cell was visited
   */
  inline bool wasVisited()
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
  inline uint64_t getIndex()
  {
    return _index;
  }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @param collision_checker Pointer to collision checker object
   * @return whether this node is valid and collision free
   */
  bool isNodeValid(const bool & traverse_unknown, GridCollisionChecker * collision_checker);

  /**
   * @brief get traversal cost from this node to child node
   * @param child Node pointer to this node's child
   * @return traversal cost
   */
  float getTraversalCost(const NodePtr & child);

  /**
   * @brief Get index
   * @param x x coordinate of point to get index of
   * @param y y coordinate of point to get index of
   * @param width width of costmap
   * @return index
   */
  static inline uint64_t getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & width)
  {
    return static_cast<uint64_t>(x) + static_cast<uint64_t>(y) *
           static_cast<uint64_t>(width);
  }

  /**
   * @brief Get index
   * @param Index Index of point
   * @param width width of costmap
   * @param angles angle bins to use (must be 1 or throws exception)
   * @return coordinates of point
   */
  static inline Coordinates getCoords(
    const uint64_t & index, const unsigned int & width, const unsigned int & angles)
  {
    if (angles != 1) {
      throw std::runtime_error("Node type Node2D does not have a valid angle quantization.");
    }

    return Coordinates(index % width, index / width);
  }

  /**
   * @brief Get index
   * @param Index Index of point
   * @return coordinates of point
   */
  static inline Coordinates getCoords(const uint64_t & index)
  {
    const unsigned int & size_x = _neighbors_grid_offsets[3];
    return Coordinates(index % size_x, index / size_x);
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
   * @brief Initialize the neighborhood to be used in A*
   * We support 4-connect (VON_NEUMANN) and 8-connect (MOORE)
   * @param neighborhood The desired neighborhood type
   * @param x_size_uint The total x size to find neighbors
   * @param y_size The total y size to find neighbors
   * @param num_angle_quantization Number of quantizations, must be 0
   * @param search_info Search parameters, unused by 2D node
   */
  static void initMotionModel(
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & num_angle_quantization,
    SearchInfo & search_info);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param validity_checker Functor for state validity checking
   * @param collision_checker Collision checker to use
   * @param traverse_unknown If unknown costs are valid to traverse
   * @param neighbors Vector of neighbors to be filled
   */
  void getNeighbors(
    std::function<bool(const uint64_t &,
    nav2_smac_planner::Node2D * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  bool backtracePath(CoordinateVector & path);

  Node2D * parent;
  static float cost_travel_multiplier;
  static std::vector<int> _neighbors_grid_offsets;

private:
  float _cell_cost;
  float _accumulated_cost;
  uint64_t _index;
  bool _was_visited;
  bool _is_queued;
  bool _in_collision{false};
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_2D_HPP_
