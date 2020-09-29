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

#ifndef SMAC_PLANNER__NODE_SE2_HPP_
#define SMAC_PLANNER__NODE_SE2_HPP_

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>

#include <ompl/base/StateSpace.h>

#include "smac_planner/constants.hpp"
#include "smac_planner/types.hpp"
#include "smac_planner/collision_checker.hpp"

namespace smac_planner
{

// Need seperate pose struct for motion table operations
struct MotionPose
{
  MotionPose() {}
  MotionPose(const float & x, const float & y, const float & theta)
  : _x(x), _y(y), _theta(theta)
  {}

  float _x;
  float _y;
  float _theta;
};

typedef std::vector<MotionPose> MotionPoses;

// Must forward declare
class NodeSE2;

struct MotionTable
{
  MotionTable() {}

  void initDubin(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);
  void initReedsShepp(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  MotionPoses getProjections(const NodeSE2 * node);
  MotionPose getProjection(const NodeSE2 * node, const unsigned int & motion_index);

  MotionPoses projections;
  unsigned int size_x;
  unsigned int num_angle_quantization;
  float num_angle_quantization_float;
  float bin_size;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  ompl::base::StateSpacePtr state_space;
};

/**
 * @class smac_planner::NodeSE2
 * @brief NodeSE2 implementation for graph
 */
class NodeSE2
{
public:
  typedef NodeSE2 * NodePtr;
  typedef std::unique_ptr<std::vector<NodeSE2>> Graph;
  typedef std::vector<NodePtr> NodeVector;
  /**
   * @class smac_planner::NodeSE2::Coordinates
   * @brief NodeSE2 implementation of coordinate structure
   */
  struct Coordinates
  {
    Coordinates() {}
    Coordinates(const float & x_in, const float & y_in, const float & theta_in)
    : x(x_in), y(y_in), theta(theta_in)
    {}

    float x, y, theta;
  };

  typedef std::vector<Coordinates> CoordinateVector;

  /**
   * @brief A constructor for smac_planner::NodeSE2
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  explicit NodeSE2(const unsigned int index);

  /**
   * @brief A destructor for smac_planner::NodeSE2
   */
  ~NodeSE2();

  /**
   * @brief operator== for comparisons
   * @param NodeSE2 right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const NodeSE2 & rhs)
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
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
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
  inline void setAccumulatedCost(const float cost_in)
  {
    _accumulated_cost = cost_in;
  }

  /**
   * @brief Sets the motion primitive index used to achieve node in search
   * @param reference to motion primitive idx
   */
  inline void setMotionPrimitiveIndex(const unsigned int & idx)
  {
    _motion_primitive_index = idx;
  }

  /**
   * @brief Gets the motion primitive index used to achieve node in search
   * @return reference to motion primitive idx
   */
  inline unsigned int & getMotionPrimitiveIndex()
  {
    return _motion_primitive_index;
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
  bool isNodeValid(const bool & traverse_unknown, GridCollisionChecker collision_checker);

  float getTraversalCost(const NodePtr & child);

  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle,
    const unsigned int & width, const unsigned int angle_quantization)
  {
    return angle + x * angle_quantization + y * width * angle_quantization;
  }

  static inline Coordinates getCoords(
    const unsigned int & index,
    const unsigned int & width, const unsigned int angle_quantization)
  {
    return Coordinates(
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

  static void initMotionModel(
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & angle_quantization,
    SearchInfo & search_info);

  static void computeWavefrontHeuristic(
    nav2_costmap_2d::Costmap2D * & costmap,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param node Pointer to the node we are currently exploring in A*
   * @param validity_checker Functor for state validity checking
   * @param neighbors Vector of neighbors to be filled
   */
  static void getNeighbors(
    const NodePtr & node,
    std::function<bool(const unsigned int &, smac_planner::NodeSE2 * &)> & validity_checker,
    GridCollisionChecker collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  NodeSE2 * parent;
  Coordinates pose;
  static double neutral_cost;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  bool _is_queued;
  unsigned int _motion_primitive_index;
  static MotionTable _motion_model;
  static std::vector<unsigned int> _wavefront_heuristic;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NODE_SE2_HPP_
