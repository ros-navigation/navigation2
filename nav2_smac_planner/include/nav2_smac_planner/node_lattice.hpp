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
// https://www.ri.cmu.edu/pub_files/pub4/pivtoraiko_mihail_2007_1/pivtoraiko_mihail_2007_1.pdf
		// controls should be the most basic primitives and remove any that can be made up (or mostly represented) as a linear combination of other primitives.
			// fig 4 on Page 12
			// if a primitive passes near another node, use that node instead
		// lattice edges should end where others can pick up or lattice correctly into to reduce branching effects
			// TODO how do I use that if visiting the same bin/cell from leaving and entering of a continuous lattice grid?
		// controls should radiate from the central node
			// by radiating + decomposing smaller primitives that are "close" sets the length of the primitives
		// not all angular bins will have the same number of primitives
		// params: cell resolution, angle discretizations, max turning specifications
		// end of page 18 talks about delaying collision checking for speed ups!!
			// use minimum possible cost until it is expanded in the search for use, then collision check
			// so only check collisions on expansion, not on arrival like we do right now
			// then compare with the next best in the open set and reinsert if required
			// add a "has been checked" so that if requeued or looked at from another source, it doesn't recompute.
		// distance traveled used as cost -- Reeds-shepp are better than L2 norms, but still not great
			// instead they use an obstacle-free lookup using the planner itself to compute costs
			// sounds a whole lot like the Hybrid-A* method I simplified to wavefront
			// they recommend precomputing by running the planner for the minimum spanning set of primitives and looking up actual costs later
			// the lookup table can't be infinitely sized, so use the L2 norm for distant queries off the lookup table
			// window size for lookup set by a ratio of estimated costs using L2
		// they did experiments with 16 bins and 12 primitives per orientation (on average)
		// longer controls = less expansions to reach goal, but a longer total path length (suboptimal)
		// More controls = more memory and compute, but better chances of a shorter route 
		// not much slower than A* when using the perfect hueristic + longer primitives than just closest neighbors

// TODO continuous coordinates not required here?!?! End in bins exactly, might change how we do search to be more 2d-A*-y
		// must be to be a lattice pattern
	  // but also want to make use of the analytic expansion

// TODO depending on primitive lengths, collision check intermediary points, not just the end points.
		// do the non-checking-until-expanded thing--make sure to watch out for the fragile isNodeValid setting of _cost_cell and then later getting for getTraversalCost

// TODO smoother improvements / replacement
// TODO on approach not working if unreachable / can't get to approach on destination invalid?

// matt
// SBPL-or-other methods for state lattice robot-centric
	// is it even a lattice? Or approximation?
	// week or two to figure out if their method works or go back to this other method
	// ME: look into this method from SBPL and others like it to see what makes sense + understand it

// TODO test coverage
// TODO update docs for new plugin name Hybrid + Lattice (plugins page, configuration page) and add new params for lattice/description of algo
  // add all the optimizations added (cached heuristics all, leveraging symmetry in the H-space to lower mem footprint, precompute primitives and rotations, precompute footprint rotations, collision check only when required)
// TODO update any compute times / map sizes as given in docs/readmes
// TODO possibly inscribed proper use
// TODO param default updttes

// forward declare
class NodeLattice;
class NodeHybrid;

typedef std::pair<unsigned int, double> LatticeMetadata;

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
    SearchInfo & search_info);

  /**
   * @brief Get projections of motion models
   * @param node Ptr to NodeLattice
   * @return A set of motion poses
   */
  MotionPoses getProjections(const NodeLattice * node);

  /**
   * @brief Get file metadata needed
   * @param lattice_filepath Filepath to the lattice file
   * @return A set of metadata containing the number of angular bins
   * and the global coordinates minimum turning radius of the primitives
   * for use in analytic expansion and heuristic calculation.
   */
  static LatticeMetadata getLatticeMetadata(const std::string & lattice_filepath);

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
  std::vector<TrigValues> trig_values;
  std::string current_lattice_filepath;
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
   * @brief Compute the wavefront heuristic
   * @param costmap Costmap to use to compute heuristic
   * @param start_x Coordinate of Start X
   * @param start_y Coordinate of Start Y
   * @param goal_x Coordinate of Goal X
   * @param goal_y Coordinate of Goal Y
   */
  static void precomputeWavefrontHeuristic(
    nav2_costmap_2d::Costmap2D * & costmap,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y);

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
  static double neutral_cost;
  static LatticeMotionTable motion_table;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  bool _is_queued;
  unsigned int _motion_primitive_index;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_
