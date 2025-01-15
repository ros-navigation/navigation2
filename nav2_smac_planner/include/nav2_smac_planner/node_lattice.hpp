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
#include <string>

#include "nlohmann/json.hpp"
#include "ompl/base/StateSpace.h"
#include "angles/angles.h"

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/utils.hpp"

namespace nav2_smac_planner
{

// forward declare
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
   * @param search_info Parameters for searching
   */
  void initMotionModel(
    unsigned int & size_x_in,
    SearchInfo & search_info);

  /**
   * @brief Get projections of motion models
   * @param node Ptr to NodeLattice
   * @param Reference direction change index
   * @return A set of motion poses
   */
  MotionPrimitivePtrs getMotionPrimitives(
    const NodeLattice * node,
    unsigned int & direction_change_index);

  /**
   * @brief Get file metadata needed
   * @param lattice_filepath Filepath to the lattice file
   * @return A set of metadata containing the number of angular bins
   * and the global coordinates minimum turning radius of the primitives
   * for use in analytic expansion and heuristic calculation.
   */
  static LatticeMetadata getLatticeMetadata(const std::string & lattice_filepath);

  /**
   * @brief Get the angular bin to use from a raw orientation
   * @param theta Angle in radians
   * @return bin index of closest angle to request
   */
  unsigned int getClosestAngularBin(const double & theta);

  /**
   * @brief Get the raw orientation from an angular bin
   * @param bin_idx Index of the bin
   * @return Raw orientation in radians
   */
  float & getAngleFromBin(const unsigned int & bin_idx);

  /**
   * @brief Get the angular bin to use from a raw orientation
   * @param theta Angle in radians
   * @return bin index of closest angle to request
   */
  double getAngle(const double & theta);

  unsigned int size_x;
  unsigned int num_angle_quantization;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  float travel_distance_reward;
  float rotation_penalty;
  float min_turning_radius;
  bool allow_reverse_expansion;
  std::vector<std::vector<MotionPrimitive>> motion_primitives;
  ompl::base::StateSpacePtr state_space;
  std::vector<TrigValues> trig_values;
  std::string current_lattice_filepath;
  LatticeMetadata lattice_metadata;
  MotionModel motion_model = MotionModel::UNKNOWN;
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
  explicit NodeLattice(const uint64_t index);

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
   * @brief Sets the motion primitive used to achieve node in search
   * @param pointer to motion primitive
   */
  inline void setMotionPrimitive(MotionPrimitive * prim)
  {
    _motion_primitive = prim;
  }

  /**
   * @brief Gets the motion primitive used to achieve node in search
   * @return pointer to motion primitive
   */
  inline MotionPrimitive * & getMotionPrimitive()
  {
    return _motion_primitive;
  }

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
   * @brief Sets that this primitive is moving in reverse
   */
  inline void backwards(bool back = true)
  {
    _backwards = back;
  }

  /**
   * @brief Gets if this primitive is moving in reverse
   * @return backwards If moving in reverse
   */
  inline bool isBackward()
  {
    return _backwards;
  }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @param collision_checker Collision checker object to aid in validity checking
   * @param primitive Optional argument if needing to check over a primitive
   * not only a terminal pose
   * @param is_backwards Optional argument if needed to check if prim expansion is
   * in reverse
   * @return whether this node is valid and collision free
   */
  bool isNodeValid(
    const bool & traverse_unknown,
    GridCollisionChecker * collision_checker,
    MotionPrimitive * primitive = nullptr,
    bool is_backwards = false);

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
  static inline uint64_t getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle)
  {
    // Hybrid-A* and State Lattice share a coordinate system
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
    const uint64_t & index,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    // Hybrid-A* and State Lattice share a coordinate system
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
   * @brief Compute the SE2 distance heuristic
   * @param lookup_table_dim Size, in costmap pixels, of the
   * each lookup table dimension to populate
   * @param motion_model Motion model to use for state space
   * @param dim_3_size Number of quantization bins for caching
   * @param search_info Info containing minimum radius to use
   */
  static void precomputeDistanceHeuristic(
    const float & lookup_table_dim,
    const MotionModel & motion_model,
    const unsigned int & dim_3_size,
    const SearchInfo & search_info);

  /**
   * @brief Compute the wavefront heuristic
   * @param costmap Costmap to use
   * @param goal_coords Coordinates to start heuristic expansion at
   */
  static void resetObstacleHeuristic(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y)
  {
    // State Lattice and Hybrid-A* share this heuristics
    NodeHybrid::resetObstacleHeuristic(costmap_ros, start_x, start_y, goal_x, goal_y);
  }

  /**
   * @brief Compute the Obstacle heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @return heuristic Heuristic value
   */
  static float getObstacleHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const double & cost_penalty)
  {
    return NodeHybrid::getObstacleHeuristic(node_coords, goal_coords, cost_penalty);
  }

  /**
   * @brief Compute the Distance heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @param obstacle_heuristic Value of the obstacle heuristic to compute
   * additional motion heuristics if required
   * @return heuristic Heuristic value
   */
  static float getDistanceHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const float & obstacle_heuristic);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param validity_checker Functor for state validity checking
   * @param collision_checker Collision checker to use
   * @param traverse_unknown If unknown costs are valid to traverse
   * @param neighbors Vector of neighbors to be filled
   */
  void getNeighbors(
    std::function<bool(const uint64_t &,
    nav2_smac_planner::NodeLattice * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  bool backtracePath(CoordinateVector & path);

  /**
   * \brief add node to the path
   * \param current_node
   */
  void addNodeToPath(NodePtr current_node, CoordinateVector & path);

  NodeLattice * parent;
  Coordinates pose;
  static LatticeMotionTable motion_table;
  // Dubin / Reeds-Shepp lookup and size for dereferencing
  static LookupTable dist_heuristic_lookup_table;
  static float size_lookup;

private:
  float _cell_cost;
  float _accumulated_cost;
  uint64_t _index;
  bool _was_visited;
  MotionPrimitive * _motion_primitive;
  bool _backwards;
  bool _is_node_valid{false};
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_
