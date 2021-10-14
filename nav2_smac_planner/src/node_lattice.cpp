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
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_smac_planner/node_lattice.hpp"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// defining static member for all instance to share
LatticeMotionTable NodeLattice::motion_table;
float NodeLattice::size_lookup = 25;
LookupTable NodeLattice::dist_heuristic_lookup_table;

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
  allow_reverse_expansion = search_info.allow_reverse_expansion;

  // Get the metadata about this minimum control set
  lattice_metadata = getLatticeMetadata(current_lattice_filepath);
  std::ifstream latticeFile(current_lattice_filepath);
  if(!latticeFile.is_open()) {
    throw std::runtime_error("Could not open lattice file");
  }
  nlohmann::json json;
  latticeFile >> json;
  num_angle_quantization = lattice_metadata.number_of_headings;

  if (!state_space) {
    if (!allow_reverse_expansion) {
      state_space = std::make_unique<ompl::base::DubinsStateSpace>(
        lattice_metadata.min_turning_radius);
    } else {
      state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
        lattice_metadata.min_turning_radius);
    }
  }

  // Populate the motion primitives at each heading angle
  float prev_start_angle = 0.0;
  std::vector<MotionPrimitive> primitives; 
  nlohmann::json json_primitives = json["primitives"];
  for(unsigned int i = 0; i < json_primitives.size(); ++i) {
    MotionPrimitive new_primitive; 
    fromJsonToMotionPrimitive(json_primitives[i], new_primitive); 

    if(prev_start_angle != new_primitive.start_angle) {
      motion_primitives.push_back(primitives);
      primitives.clear(); 
      prev_start_angle = new_primitive.start_angle;
    }
    primitives.push_back(new_primitive); 
  }
  motion_primitives.push_back(primitives);

  // Populate useful precomputed values to be leveraged
  trig_values.reserve(lattice_metadata.number_of_headings);
  for(unsigned int i = 0; i < lattice_metadata.heading_angles.size(); ++i) {
    trig_values.emplace_back(
      cos(lattice_metadata.heading_angles[i]),
      sin(lattice_metadata.heading_angles[i]));
  }
}

MotionPrimitives LatticeMotionTable::getMotionPrimitives(const NodeLattice * node)
{
  MotionPrimitives & prims_at_heading = motion_primitives[node->pose.theta];
  MotionPrimitives primitive_projection_list = prims_at_heading; 

  if (allow_reverse_expansion) {
    // Find normalized heading bin of the reverse expansion
    double reserve_heading = node->pose.theta - (num_angle_quantization / 2);
    if (reserve_heading < 0) {
      reserve_heading += num_angle_quantization;
    }
    if (reserve_heading > num_angle_quantization) {
      reserve_heading -= num_angle_quantization;
    }

    prims_at_heading = motion_primitives[reserve_heading];
    primitive_projection_list.insert(
      primitive_projection_list.end(),
      prims_at_heading.begin(),
      prims_at_heading.end());
  }

  return primitive_projection_list;
}

LatticeMetadata LatticeMotionTable::getLatticeMetadata(const std::string & lattice_filepath)
{
  std::ifstream lattice_file(lattice_filepath);
  if(!lattice_file.is_open()) {
    throw std::runtime_error("Could not open lattice file!");
  }

  nlohmann::json j;
  lattice_file >> j;
  LatticeMetadata metadata;
  fromJsonToMetaData(j["latticeMetadata"], metadata);
  return metadata;
}

unsigned int LatticeMotionTable::getClosestAngularBin(const double & theta)
{
  float min_dist = std::numeric_limits<float>::max();
  unsigned int closest_idx = 0;
  float dist = 0.0;
  for (unsigned int i = 0; i != lattice_metadata.heading_angles.size(); i++) {
    dist = fabs(theta - lattice_metadata.heading_angles[i]);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

float & LatticeMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  return lattice_metadata.heading_angles[bin_idx];
}

NodeLattice::NodeLattice(const unsigned int index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive(nullptr),
  _backwards(false)
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
  _motion_primitive = nullptr;
  _backwards = false;
}

bool NodeLattice::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker,
  MotionPrimitive * motion_primitive)
{
  // Check primitive end pose
  // Convert grid quantization of primitives to radians, then collision checker quantization
  static const double bin_size = 2.0 * M_PI / collision_checker->getPrecomputedAngles().size();
  const double & angle = motion_table.getAngleFromBin(this->pose.theta) / bin_size;
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, angle /*bin in collision checker*/, traverse_unknown))
  {
    return false;
  }

  // Set the cost of a node to the highest cost across the primitive
  float max_cell_cost = collision_checker->getCost();

  // If valid motion primitives are set, check intermediary poses > 1 cell apart
  if (motion_primitive) {
    const float & grid_resolution = motion_table.lattice_metadata.grid_resolution;
    const float & resolution_diag_sq = 2.0 * grid_resolution * grid_resolution;
    MotionPose last_pose(1e9, 1e9, 1e9), pose_dist(0.0, 0.0, 0.0);

    // Back out the initial node starting point to move motion primitive relative to
    MotionPose initial_pose, prim_pose;
    initial_pose._x = this->pose.x - (motion_primitive->poses.back()._x / grid_resolution);
    initial_pose._y = this->pose.y - (motion_primitive->poses.back()._y / grid_resolution);
    initial_pose._theta = motion_table.getAngleFromBin(motion_primitive->start_angle); /*rad*/

    for (auto it = motion_primitive->poses.begin(); it != motion_primitive->poses.end(); ++it) {
      // poses are in metric coordinates from (0, 0), not grid space yet
      pose_dist = *it - last_pose;
      // Avoid square roots by (hypot(x, y) > res) == (x*x+y*y > diag*diag)
      if (pose_dist._x * pose_dist._x + pose_dist._y * pose_dist._y > resolution_diag_sq) {
        last_pose = *it;
        // Convert primitive pose into grid space if it should be checked
        prim_pose._x = initial_pose._x + (it->_x / grid_resolution);
        prim_pose._y = initial_pose._y + (it->_y / grid_resolution);
        prim_pose._theta = initial_pose._theta + it->_theta; /*rad*/
        if (prim_pose._theta < 0.0) {
          prim_pose._theta += 2.0 * M_PI;
        }
        if (prim_pose._theta > 2.0 * M_PI) {
          prim_pose._theta -= 2.0 * M_PI;
        }
        if (collision_checker->inCollision(
            prim_pose._x,
            prim_pose._y,
            prim_pose._theta / bin_size /*bin in collision checker*/,
            traverse_unknown))
        {
          return false;
        }
        max_cell_cost = std::max(max_cell_cost, collision_checker->getCost());
      }
    }
  }

  _cell_cost = max_cell_cost;
  return true;
}

float NodeLattice::getTraversalCost(const NodePtr & child)
{
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known collision cost!");
  }

  // this is the first node
  MotionPrimitive * prim = this->getMotionPrimitive();
  MotionPrimitive * transition_prim = child->getMotionPrimitive();
  if (prim == nullptr) {
    return transition_prim->trajectory_length;
  }

  // Note(stevemacenski): `travel_cost_raw` at one point contained a term:
  // `+ motion_table.cost_penalty * normalized_cost;`
  // It has been removed, but we may want to readdress this point and determine
  // the technically and theoretically correctness of that choice. I feel technically speaking
  // that term has merit, but it doesn't seem to impact performance or path quality.
  // W/o it lowers the travel cost, which would drive the heuristics up proportionally where I
  // would expect it to plan much faster in all cases, but I only see it in some cases. Since
  // this term would weight against moving to high cost zones, I would expect to see more smooth
  // central motion, but I only see it in some cases, possibly because the obstacle heuristic is
  // already driving the robot away from high cost areas; implicitly handling this. However,
  // then I would expect that not adding it here would make it unbalanced enough that path quality
  // would suffer, which I did not see in my limited experimentation, possibly due to the smoother.
  float travel_cost = 0.0;
  float travel_cost_raw = transition_prim->trajectory_length;

  if (transition_prim->arc_length < 0.001) {
    // New motion is a straight motion, no additional costs to be applied
    travel_cost = travel_cost_raw;
  } else {
    if (prim->left == transition_prim->left) {
      // Turning motion but keeps in same general direction: encourages to commit to actions
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } else {
      // Turning motion and velocity directions: penalizes wiggling. 
      travel_cost = travel_cost_raw *
        (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  // If backwards flag is set, this primitive is moving in reverse
  if (this->isBackward()) {
    // reverse direction
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
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
  if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(
    round(dx * cos_th - dy * sin_th),
    round(dx * sin_th + dy * cos_th),
    round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table is contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  float motion_heuristic = 0.0;
  const int floored_size = floor(size_lookup / 2.0);
  const int ceiling_size = ceil(size_lookup / 2.0);
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
    motion_heuristic = dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic == 0.0) {
    static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    to[2] = motion_table.getAngleFromBin(goal_coords.theta);
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = motion_table.getAngleFromBin(node_coords.theta);
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

void NodeLattice::precomputeDistanceHeuristic(
  const float & lookup_table_dim,
  const MotionModel & motion_model,
  const unsigned int & dim_3_size,
  const SearchInfo & search_info)
{
  // Dubin or Reeds-Shepp shortest distances
  if (!search_info.allow_reverse_expansion) {
    motion_table.state_space = std::make_unique<ompl::base::DubinsStateSpace>(
      search_info.minimum_turning_radius);
  } else {
    motion_table.state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
      search_info.minimum_turning_radius);
  }

  ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Deu to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  dist_heuristic_lookup_table.resize(size_lookup * ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = motion_table.getAngleFromBin(heading);
        motion_heuristic = motion_table.state_space->distance(from(), to());
        dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeLattice::getNeighbors(
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeLattice * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  unsigned int index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords, motion_projection;
  MotionPrimitives motion_primitives = motion_table.getMotionPrimitives(this);
  const float & grid_resolution = motion_table.lattice_metadata.grid_resolution;

  unsigned int direction_change_idx = 1e9;
  for (unsigned int i = 0; i != motion_primitives.size(); i++) {
    if (motion_primitives[0].start_angle != motion_primitives[i].start_angle) {
      direction_change_idx = i;
      break;
    }
  }

  for (unsigned int i = 0; i != motion_primitives.size(); i++) {
    const MotionPose & end_pose = motion_primitives[i].poses.back();
    motion_projection.x = this->pose.x + (end_pose._x / grid_resolution);
    motion_projection.y = this->pose.y + (end_pose._y / grid_resolution);
    motion_projection.theta = motion_primitives[i].end_angle /*this is the ending angular bin*/;
    index = NodeLattice::getIndex(
      static_cast<unsigned int>(motion_projection.x),
      static_cast<unsigned int>(motion_projection.y),
      static_cast<unsigned int>(motion_projection.theta));

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projection.x,
          motion_projection.y,
          motion_projection.theta));
      // Using a special isNodeValid API here, giving the motion primitive to use to
      // validity check the transition of the current node to the new node over
      if (neighbor->isNodeValid(traverse_unknown, collision_checker, &motion_primitives[i])) {
        neighbor->setMotionPrimitive(&motion_primitives[i]);
        // Marking if this search was obtained in the reverse direction
        if ((!this->isBackward() && i >= direction_change_idx) || 
            (this->isBackward() && i <= direction_change_idx))
        {
          neighbor->backwards();
        }
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

}  // namespace nav2_smac_planner
