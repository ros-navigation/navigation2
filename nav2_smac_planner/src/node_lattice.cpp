// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#include <math.h>
#include <chrono>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <limits>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_smac_planner/node_lattice.hpp"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// defining static member for all instance to share
LatticeMotionTable NodeLattice::motion_table;
double NodeLattice::neutral_cost = sqrt(2);

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.
void LatticeMotionTable::initMotionModel(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;

  if (num_angle_quantization_in == num_angle_quantization &&
      min_turning_radius == search_info.minimum_turning_radius)
  {
    return;
  }

  // TODO Matt read in file, precompute based on orientation bins for lookup at runtime
  // file is `search_info.lattice_filepath`, to be read in from plugin and provided here.

  // TODO Matt create a state_space with the max turning rad primitive within the file (or another -- mid?)
  // to use for analytic expansions and heuristic generation. Potentially make both an extreme and a passive one?
}

MotionPoses LatticeMotionTable::getProjections(const NodeLattice * node)
{
  return MotionPoses();  // TODO Matt lookup at run time the primitives to use at node
}

LatticeMetadata LatticeMotionTable::getLatticeMetadata(const std::string & lattice_filepath)
{
  // TODO Matt from this file extract and return the number of angle bins and turning radius in global coordinates, respectively.
  // world coordinates meaning meters, not cells
  return {0 /*num bins*/, 0 /*turning rad*/};
}

NodeLattice::NodeLattice(const unsigned int index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _is_queued(false)
{
}

NodeLattice::~NodeLattice()
{
  parent = nullptr;
}

void NodeLattice::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _is_queued = false;
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
}

bool NodeLattice::isNodeValid(const bool & traverse_unknown, GridCollisionChecker & collision_checker)
{
  if (collision_checker.inCollision(
      this->pose.x, this->pose.y, this->pose.theta * motion_table.bin_size, traverse_unknown))
  {
    return false;
  }

  _cell_cost = collision_checker.getCost();
  return true;
}

float NodeLattice::getTraversalCost(const NodePtr & child)
{
  return 0.0;  // TODO Josh: cost of different angles, changing, nonstraight, backwards, distance long
  // should this use neutral_cost? TODO if not, set to 1 for no impact in A*
  // can use getMotionPrimitiveIndex() to get the ID of the index of the primitive the child/this belongs to for use
  // feel free to make new params, let me know and I'll add tothe SearchInfo struct for inputs and add to the plugin TODO
}

float NodeLattice::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords)
{
  return 0.0;  // TODO Josh: wavefront, ompl, or more optimal heuristic for wavefront search.
  // should this use neutral_cost? TODO if not, set to 1 for no impact A*
  // feel free to make new params, let me know and I'll add tothe SearchInfo struct for inputs and add to the plugin TODO
}

void NodeLattice::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & size_y,
  unsigned int & num_angle_quantization,
  SearchInfo & search_info)
{

  if (motion_model != MotionModel::STATE_LATTICE) {
    throw std::runtime_error(
      "Invalid motion model for Lattice node. Please select"
      " STATE_LATTICE and provide a valid lattice file.");
  }

  motion_table.initMotionModel(size_x, size_y, num_angle_quantization, search_info);
}

void NodeLattice::getNeighbors(
  const NodePtr & node,
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeLattice * &)> & NeighborGetter,
  GridCollisionChecker & collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  unsigned int index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = motion_table.getProjections(node);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeLattice::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta));

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt other lattices being expanded
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
        neighbor->setMotionPrimitiveIndex(i);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

}  // namespace nav2_smac_planner
