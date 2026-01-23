// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
// Copyright (c) 2023, Open Navigation LLC
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
#include <utility>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_smac_planner/node_hybrid.hpp"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.

// http://planning.cs.uiuc.edu/planning/node821.html
// Model for ackermann style vehicle with minimum radius restriction
void HybridMotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  downsample_obstacle_heuristic = search_info.downsample_obstacle_heuristic;
  use_quadratic_cost_penalty = search_info.use_quadratic_cost_penalty;

  // if nothing changed, no need to re-compute primitives
  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::DUBIN)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::DUBIN;

  // angle must meet 3 requirements:
  // 1) be increment of quantized bin size
  // 2) chord length must be greater than sqrt(2) to leave current cell
  // 3) maximum curvature must be respected, represented by minimum turning angle
  // Thusly:
  // On circle of radius minimum turning angle, we need select motion primitives
  // with chord length > sqrt(2) and be an increment of our bin size
  //
  // chord >= sqrt(2) >= 2 * R * sin (angle / 2); where angle / N = quantized bin size
  // Thusly: angle <= 2.0 * asin(sqrt(2) / (2 * R))
  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  // Now make sure angle is an increment of the quantized bin size
  // And since its based on the minimum chord, we need to make sure its always larger
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    // Search dimensions are clean multiples of quantization - this prevents
    // paths with loops in them
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  // find deflections
  // If we make a right triangle out of the chord in circle of radius
  // min turning angle, we can see that delta X = R * sin (angle)
  const float delta_x = min_turning_radius * sin(angle);
  // Using that same right triangle, we can see that the complement
  // to delta Y is R * cos (angle). If we subtract R, we get the actual value
  const float delta_y = min_turning_radius - (min_turning_radius * cos(angle));
  const float delta_dist = hypotf(delta_x, delta_y);

  projections.clear();
  projections.reserve(3);
  projections.emplace_back(delta_dist, 0.0, 0.0, TurnDirection::FORWARD);  // Forward
  projections.emplace_back(delta_x, delta_y, increments, TurnDirection::LEFT);  // Left
  projections.emplace_back(delta_x, -delta_y, -increments, TurnDirection::RIGHT);  // Right

  if (search_info.allow_primitive_interpolation && increments > 1.0f) {
    // Create primitives that are +/- N to fill in search space to use all set angular quantizations
    // Allows us to create N many primitives so that each search iteration can expand into any angle
    // bin possible with the minimum turning radius constraint, not just the most extreme turns.
    projections.reserve(3 + (2 * (increments - 1)));
    for (unsigned int i = 1; i < static_cast<unsigned int>(increments); i++) {
      const float angle_n = static_cast<float>(i) * bin_size;
      const float turning_rad_n = delta_dist / (2.0f * sin(angle_n / 2.0f));
      const float delta_x_n = turning_rad_n * sin(angle_n);
      const float delta_y_n = turning_rad_n - (turning_rad_n * cos(angle_n));
      projections.emplace_back(
        delta_x_n, delta_y_n, static_cast<float>(i), TurnDirection::LEFT);  // Left
      projections.emplace_back(
        delta_x_n, -delta_y_n, -static_cast<float>(i), TurnDirection::RIGHT);  // Right
    }
  }

  // Create the correct OMPL state space
  state_space = std::make_shared<ompl::base::DubinsStateSpace>(min_turning_radius);

  // Precompute projection deltas
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }

  // Precompute travel costs for each motion primitive
  travel_costs.resize(projections.size());
  for (unsigned int i = 0; i != projections.size(); i++) {
    const TurnDirection turn_dir = projections[i]._turn_dir;
    if (turn_dir != TurnDirection::FORWARD && turn_dir != TurnDirection::REVERSE) {
      // Turning, so length is the arc length
      const float arc_angle = projections[i]._theta * bin_size;
      const float turning_rad = delta_dist / (2.0f * sin(arc_angle / 2.0f));
      travel_costs[i] = turning_rad * arc_angle;
    } else {
      travel_costs[i] = delta_dist;
    }
  }
}

// http://planning.cs.uiuc.edu/planning/node822.html
// Same as Dubin model but now reverse is valid
// See notes in Dubin for explanation
void HybridMotionTable::initReedsShepp(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  downsample_obstacle_heuristic = search_info.downsample_obstacle_heuristic;
  use_quadratic_cost_penalty = search_info.use_quadratic_cost_penalty;

  // if nothing changed, no need to re-compute primitives
  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::REEDS_SHEPP)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::REEDS_SHEPP;

  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  const float delta_x = min_turning_radius * sin(angle);
  const float delta_y = min_turning_radius - (min_turning_radius * cos(angle));
  const float delta_dist = hypotf(delta_x, delta_y);

  projections.clear();
  projections.reserve(6);
  projections.emplace_back(delta_dist, 0.0, 0.0, TurnDirection::FORWARD);  // Forward
  projections.emplace_back(
    delta_x, delta_y, increments, TurnDirection::LEFT);  // Forward + Left
  projections.emplace_back(
    delta_x, -delta_y, -increments, TurnDirection::RIGHT);  // Forward + Right
  projections.emplace_back(-delta_dist, 0.0, 0.0, TurnDirection::REVERSE);  // Backward
  projections.emplace_back(
    -delta_x, delta_y, -increments, TurnDirection::REV_LEFT);  // Backward + Left
  projections.emplace_back(
    -delta_x, -delta_y, increments, TurnDirection::REV_RIGHT);  // Backward + Right

  if (search_info.allow_primitive_interpolation && increments > 1.0f) {
    // Create primitives that are +/- N to fill in search space to use all set angular quantizations
    // Allows us to create N many primitives so that each search iteration can expand into any angle
    // bin possible with the minimum turning radius constraint, not just the most extreme turns.
    projections.reserve(6 + (4 * (increments - 1)));
    for (unsigned int i = 1; i < static_cast<unsigned int>(increments); i++) {
      const float angle_n = static_cast<float>(i) * bin_size;
      const float turning_rad_n = delta_dist / (2.0f * sin(angle_n / 2.0f));
      const float delta_x_n = turning_rad_n * sin(angle_n);
      const float delta_y_n = turning_rad_n - (turning_rad_n * cos(angle_n));
      projections.emplace_back(
        delta_x_n, delta_y_n, static_cast<float>(i), TurnDirection::LEFT);  // Forward + Left
      projections.emplace_back(
        delta_x_n, -delta_y_n, -static_cast<float>(i), TurnDirection::RIGHT);  // Forward + Right
      projections.emplace_back(
        -delta_x_n, delta_y_n, -static_cast<float>(i),
        TurnDirection::REV_LEFT);  // Backward + Left
      projections.emplace_back(
        -delta_x_n, -delta_y_n, static_cast<float>(i),
        TurnDirection::REV_RIGHT);  // Backward + Right
    }
  }

  // Create the correct OMPL state space
  state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turning_radius);

  // Precompute projection deltas
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }

  // Precompute travel costs for each motion primitive
  travel_costs.resize(projections.size());
  for (unsigned int i = 0; i != projections.size(); i++) {
    const TurnDirection turn_dir = projections[i]._turn_dir;
    if (turn_dir != TurnDirection::FORWARD && turn_dir != TurnDirection::REVERSE) {
      // Turning, so length is the arc length
      const float arc_angle = projections[i]._theta * bin_size;
      const float turning_rad = delta_dist / (2.0f * sin(arc_angle / 2.0f));
      travel_costs[i] = turning_rad * arc_angle;
    } else {
      travel_costs[i] = delta_dist;
    }
  }
}

MotionPoses HybridMotionTable::getProjections(const NodeHybrid * node)
{
  MotionPoses projection_list;
  projection_list.reserve(projections.size());

  for (unsigned int i = 0; i != projections.size(); i++) {
    const MotionPose & proj_motion_model = projections[i];

    // normalize theta, I know its overkill, but I've been burned before...
    const float & node_heading = node->pose.theta;
    float new_heading = node_heading + proj_motion_model._theta;

    if (new_heading < 0.0) {
      new_heading += num_angle_quantization_float;
    }

    if (new_heading >= num_angle_quantization_float) {
      new_heading -= num_angle_quantization_float;
    }

    projection_list.emplace_back(
      delta_xs[i][node_heading] + node->pose.x,
      delta_ys[i][node_heading] + node->pose.y,
      new_heading, proj_motion_model._turn_dir);
  }

  return projection_list;
}

unsigned int HybridMotionTable::getClosestAngularBin(const double & theta)
{
  auto bin = static_cast<unsigned int>(round(static_cast<float>(theta) / bin_size));
  return bin < num_angle_quantization ? bin : 0u;
}

float HybridMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  return bin_idx * bin_size;
}

double HybridMotionTable::getAngle(const double & theta)
{
  return theta / bin_size;
}

NodeHybrid::NodeHybrid(const uint64_t index, NodeContext * ctx)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive_index(std::numeric_limits<unsigned int>::max()),
  _is_node_valid(false),
  _ctx(ctx)
{
}

NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

void NodeHybrid::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _motion_primitive_index = std::numeric_limits<unsigned int>::max();
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
  _is_node_valid = false;
}

bool NodeHybrid::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  // Already found, we can return the result
  if (!std::isnan(_cell_cost)) {
    return _is_node_valid;
  }

  _is_node_valid = !collision_checker->inCollision(
    this->pose.x, this->pose.y, this->pose.theta /*bin number*/, traverse_unknown);
  _cell_cost = collision_checker->getCost();
  return _is_node_valid;
}

float NodeHybrid::getTraversalCost(const NodePtr & child)
{
  const float normalized_cost = child->getCost() / 252.0f;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known SE2 collision cost!");
  }

  const TurnDirection & child_turn_dir = child->getTurnDirection();
  float travel_cost_raw = _ctx->motion_table.travel_costs[child->getMotionPrimitiveIndex()];
  float travel_cost = 0.0;

  if (_ctx->motion_table.use_quadratic_cost_penalty) {
    travel_cost_raw *=
      (_ctx->motion_table.travel_distance_reward +
      (_ctx->motion_table.cost_penalty * normalized_cost * normalized_cost));
  } else {
    travel_cost_raw *=
      (_ctx->motion_table.travel_distance_reward + _ctx->motion_table.cost_penalty *
      normalized_cost);
  }

  if (child_turn_dir == TurnDirection::FORWARD || child_turn_dir == TurnDirection::REVERSE ||
    getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max())
  {
    // New motion is a straight motion, no additional costs to be applied
    travel_cost = travel_cost_raw;
  } else {
    if (getTurnDirection() == child_turn_dir) {
      // Turning motion but keeps in same direction: encourages to commit to turning if starting it
      travel_cost = travel_cost_raw * _ctx->motion_table.non_straight_penalty;
    } else {
      // Turning motion and changing direction: penalizes wiggling
      travel_cost = travel_cost_raw *
        (_ctx->motion_table.non_straight_penalty + _ctx->motion_table.change_penalty);
    }
  }

  if (child_turn_dir == TurnDirection::REV_RIGHT ||
    child_turn_dir == TurnDirection::REV_LEFT ||
    child_turn_dir == TurnDirection::REVERSE)
  {
    // reverse direction
    travel_cost *= _ctx->motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeHybrid::getHeuristicCost(
  const Coordinates & node_coords,
  const CoordinateVector & goals_coords)
{
  // obstacle heuristic does not depend on goal heading
  const float obstacle_heuristic =
    _ctx->obstacle_heuristic->getObstacleHeuristic(node_coords, _ctx->motion_table.cost_penalty,
      _ctx->motion_table.use_quadratic_cost_penalty,
      _ctx->motion_table.downsample_obstacle_heuristic);
  float distance_heuristic = std::numeric_limits<float>::max();
  for (unsigned int i = 0; i < goals_coords.size(); i++) {
    distance_heuristic = std::min(
      distance_heuristic,
      _ctx->distance_heuristic->getDistanceHeuristic(node_coords, goals_coords[i],
        obstacle_heuristic, _ctx->motion_table));
  }
  return std::max(obstacle_heuristic, distance_heuristic);
}

void NodeHybrid::getNeighbors(
  std::function<bool(const uint64_t &,
  nav2_smac_planner::NodeHybrid * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  uint64_t index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = _ctx->motion_table.getProjections(this);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeHybrid::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta),
      _ctx->motion_table.size_x, _ctx->motion_table.num_angle_quantization);

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
        neighbor->setMotionPrimitiveIndex(i, motion_projections[i]._turn_dir);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

bool NodeHybrid::backtracePath(CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(current_node->pose);
    // Convert angle to radians
    path.back().theta = _ctx->motion_table.getAngleFromBin(path.back().theta);
    current_node = current_node->parent;
  }

  // add the start pose
  path.push_back(current_node->pose);
  // Convert angle to radians
  path.back().theta = _ctx->motion_table.getAngleFromBin(path.back().theta);

  return true;
}

}  // namespace nav2_smac_planner
