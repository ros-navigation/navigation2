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
#include <string>
#include <fstream>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "nav2_smac_planner/node_lattice.hpp"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// defining static member for all instance to share
LatticeMotionTable NodeLattice::motion_table;

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.
void LatticeMotionTable::initMotionModel(
  unsigned int & size_x_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;

  if (current_lattice_filepath == search_info.lattice_filepath) {
    return;
  }

  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  current_lattice_filepath = search_info.lattice_filepath;

  std::ifstream latticeFile(current_lattice_filepath);

  if( !latticeFile.is_open() )
  {
    throw std::runtime_error("Could not open lattice file");
  }

  nlohmann::json j;
  latticeFile >> j;

  fromJsonToMetaData(j["latticeMetadata"], latticeMetadata);

  float prevStartAngle = 0; 
  std::vector<MotionPrimitive> primitives; 
  for(auto const& primative : j["primitives"] )
  {
    MotionPrimitive newPrimitive; 
    fromJsonToMotionPrimitive(primative, newPrimitive); 

    if(prevStartAngle != newPrimitive.start_angle )
    {
      motionPrimitives.emplace_back(primitives);
      primitives.clear(); 
      prevStartAngle = newPrimitive.start_angle;
    }
    primitives.emplace_back(newPrimitive);
  }
  motionPrimitives.emplace_back(primitives); 


  // TODO(Matt) read in file, precompute based on orientation bins for lookup at runtime
  // file is `search_info.lattice_filepath`, to be read in from plugin and provided here.

  // TODO(Matt) create a state_space with the max turning rad primitive within the file
  // (or another -- mid?)
  // to use for analytic expansions and heuristic generation. Potentially make both an
  // extreme and a passive one?

  // TODO(Matt) populate num_angle_quantization, size_x, min_turning_radius, trig_values,
  // all of the member variables of LatticeMotionTable

  num_angle_quantization = latticeMetadata.number_of_headings;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = latticeMetadata.min_turning_radius;

  //Find the largest turning radius that is not equal to zero
  float largestTurningRadius = std::numeric_limits<float>::max();
  for(auto const& motionPrimitivesAtAngle : motionPrimitives)
  {
    for(auto const& motionPrimitive : motionPrimitivesAtAngle)
    {
      if( motionPrimitive.radius < largestTurningRadius && motionPrimitive.radius != 0 )
      {
        largestTurningRadius = motionPrimitive.radius;
      }
    }
  }

  //NOTE: Note sure what type of state space this should be 
  state_space = std::make_unique<ompl::base::DubinsStateSpace>( largestTurningRadius );

  for(auto const& motionPrimitivesAtAngle : motionPrimitives)
  {
    for(auto const& motionPrimitive : motionPrimitivesAtAngle)
    {
      //TODO: angles in json file need to be converted to radians
      primitive_start_angles.emplace_back( motionPrimitive.start_angle );
    }
  }

  //TODO: angles in json file need to be converted to radians 
  for(auto const &start_angle : primitive_start_angles)
  {
    trig_values.emplace_back( cos(start_angle), sin(start_angle) );
  }
}

MotionPoses LatticeMotionTable::getProjections(const NodeLattice * node)
{
  return MotionPoses();  // TODO(Matt) lookup at run time the primitives to use at node
}

LatticeMetadata LatticeMotionTable::getLatticeMetadata(const std::string & lattice_filepath)
{
  // TODO(Matt) from this file extract and return the number of angle bins and
  // turning radius in global coordinates, respectively.
  // world coordinates meaning meters, not cells
  return LatticeMetadata();
}

void fromJsonToMetaData(const nlohmann::json &j, LatticeMetadata &latticeMetadata)
{
  j.at("turningRadius").get_to(latticeMetadata.min_turning_radius);
  j.at("stepDistance").get_to(latticeMetadata.step_distance);
  j.at("gridSeparation").get_to(latticeMetadata.grid_separation);
  j.at("maxLength").get_to(latticeMetadata.max_length);
  j.at("numberOfHeadings").get_to(latticeMetadata.number_of_headings);
  j.at("outputFile").get_to(latticeMetadata.output_file);
  j.at("headingAngles").get_to(latticeMetadata.heading_angles);
  j.at("numberOfTrajectories").get_to(latticeMetadata.number_of_trajectories);
}

void fromJsonToPose(const nlohmann::json &j, MotionPose &pose)
{
  pose._x = j[0]; 
  pose._y = j[1]; 
  pose._theta = j[2];
}

void fromJsonToMotionPrimitive(const nlohmann::json &j, MotionPrimitive &motionPrimitive)
{
  j.at("trajectoryId").get_to(motionPrimitive.trajectory_id);
  j.at("startAngle").get_to(motionPrimitive.start_angle);
  j.at("endAngle").get_to(motionPrimitive.end_angle);   
  j.at("radius").get_to(motionPrimitive.radius);
  j.at("trajectoryLength").get_to(motionPrimitive.trajectory_length);
  j.at("arcLength").get_to(motionPrimitive.arc_length);
  j.at("straightLength").get_to(motionPrimitive.straight_length);

  for(auto& jsonPose : j["poses"])
  {
      MotionPose pose;
      fromJsonToPose(jsonPose, pose);
      motionPrimitive.poses.push_back(pose);
  }
}


NodeLattice::NodeLattice(const unsigned int index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false)
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
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
}

bool NodeLattice::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  // TODO(steve) if primitive longer than 1.5 cells, then we need to split into 1 cell
  // increments and collision check across them
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, this->pose.theta * motion_table.bin_size, traverse_unknown))
  {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float NodeLattice::getTraversalCost(const NodePtr & child)
{
  return 0.0;  // TODO(josh): cost of different angles, changing, nonstraight, backwards, distance
}

float NodeLattice::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  // get obstacle heuristic value
  const float obstacle_heuristic = getObstacleHeuristic(node_coords, goal_coords);
  const float distance_heuristic =
    getDistanceHeuristic(node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, distance_heuristic);
}

float NodeLattice::getDistanceHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & obstacle_heuristic)
{
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // Due to the rounding involved in exact cell increments for caching,
  // this is not an exact replica of a live heuristic, but has bounded error.
  // (Usually less than 1 cell length)

  // This angle is negative since we are de-rotating the current node
  // by the goal angle; cos(-th) = cos(th) & sin(-th) = -sin(th)
  const TrigValues & trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  } else if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(
    round(dx * cos_th - dy * sin_th),
    round(dx * sin_th + dy * cos_th),
    round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table is contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  float motion_heuristic = 0.0;
  const int floored_size = floor(NodeHybrid::size_lookup / 2.0);
  const int ceiling_size = ceil(NodeHybrid::size_lookup / 2.0);
  const float mirrored_relative_y = abs(node_coords_relative.y);
  if (abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {
    // Need to mirror angle if Y coordinate was mirrored
    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index =
      x_pos * ceiling_size * motion_table.num_angle_quantization +
      y_pos * motion_table.num_angle_quantization +
      theta_pos;
    motion_heuristic = NodeHybrid::dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic == 0.0) {
    static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    to[2] = goal_coords.theta * motion_table.num_angle_quantization;
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = node_coords.theta * motion_table.num_angle_quantization;
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

void NodeLattice::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & /*size_y*/,
  unsigned int & /*num_angle_quantization*/,
  SearchInfo & search_info)
{
  if (motion_model != MotionModel::STATE_LATTICE) {
    throw std::runtime_error(
            "Invalid motion model for Lattice node. Please select"
            " STATE_LATTICE and provide a valid lattice file.");
  }

  motion_table.initMotionModel(size_x, search_info);
}

void NodeLattice::getNeighbors(
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeLattice * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  unsigned int index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = motion_table.getProjections(this);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeLattice::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta));

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // For State Lattice, the poses are exact bin increments and the pose
      // can be derived from the index alone.
      // However, we store them as if they were continuous so that it may be
      // leveraged by the analytic expansion tool to accelerate goal approaches,
      // collision checking, and backtracing (even if not strictly necessary).
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
        neighbor->setMotionPrimitiveIndex(i);
        neighbors.push_back(neighbor);
      }
    }
  }
}

}  // namespace nav2_smac_planner
