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

#include "smac_planner/smac_planner_2d.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "costmap_2d/footprint_collision_checker.hpp"
#include "smac_planner/collision_checker.hpp"

// #define BENCHMARK_TESTING

namespace smac_planner
{
using namespace std::chrono;  // NOLINT

SmacPlanner2D::SmacPlanner2D()
: _a_star(nullptr), _smoother(nullptr), _costmap(nullptr), _costmap_downsampler(nullptr)
{
}

SmacPlanner2D::~SmacPlanner2D()
{
  ROS_INFO_STREAM(_logger, "Destroying plugin %s of type SmacPlanner2D", _name.c_str());
}

void SmacPlanner2D::initialize(
  const ros::NodeHandle & parent, const std::string & name, TFListenerPtr tf,
  nav_core2::Costmap::Ptr costmap)
{
  auto costmap_ros = costmap->getCostmap2DROS();
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  ros::NodeHandle nodeHandle(parent, name);

  // General planner params
  nodeHandle.param("tolerance", _tolerance, 0.125);

  nodeHandle.param("downsample_costmap", _downsample_costmap, true);
  nodeHandle.param("downsampling_factor", _downsampling_factor, 1);

  auto allow_unknown{true};
  nodeHandle.param("allow_unknown", allow_unknown, true);

  auto max_iterations{-1};
  nodeHandle.param("max_iterations", max_iterations, -1);

  auto max_on_approach_iterations{1000};
  nodeHandle.param("max_on_approach_iterations", max_on_approach_iterations, 1000);

  auto smooth_path{false};
  nodeHandle.param("smooth_path", smooth_path, false);

  auto minimum_turning_radius{0.2};
  nodeHandle.param("minimum_turning_radius", minimum_turning_radius, 0.2);

  nodeHandle.param("max_planning_time_ms", _max_planning_time, 1000.0);

  auto motion_model_for_search{"MOORE"};
  nodeHandle.param("motion_model_for_search", motion_model_for_search, std::string("MOORE"));

  MotionModel motion_model = fromString(motion_model_for_search);
  if (motion_model == MotionModel::UNKNOWN) {
    ROS_WARN_STREAM(
      "Unable to get MotionModel search type. Given '" <<
        motion_model_for_search.c_str() <<
        "', "
        "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.");
  }

  if (max_on_approach_iterations <= 0) {
    ROS_INFO_STREAM(
      "On approach iteration selected as <= 0, " <<
        "disabling tolerance and on approach iterations.");
    max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (max_iterations <= 0) {
    ROS_INFO_STREAM(
      "maximum iteration selected as <= 0, " <<
        "disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  using GridCollisionCheckerT = GridCollisionChecker<
    costmap_2d::FootprintCollisionChecker<costmap_2d::Costmap2D *>, costmap_2d::Costmap2D,
    costmap_2d::Footprint>;

  _a_star = std::make_unique<
    AStarAlgorithm<Node2D<GridCollisionCheckerT>, GridCollisionCheckerT, costmap_2d::Costmap2D>
    costmap_2d::Footprint>(motion_model, SearchInfo());
  _a_star->initialize(allow_unknown, max_iterations, max_on_approach_iterations);

  if (smooth_path) {
    _smoother = std::make_unique<Smoother<costmap_2d::Costmap2D>>();
    _optimizer_params.get(node.get(), name);
    _smoother_params.get(node.get(), name);
    _smoother_params.max_curvature = 1.0f / minimum_turning_radius;
    _smoother->initialize(_optimizer_params);
  }

  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<CostmapDownsampler<costmap_2d::Costmap2D>>();
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::Path>("unsmoothed_plan", 1);

  ROS_INFO_STREAM(
    "Configured plugin " << _name.c_str() <<
      " of type SmacPlanner2D with "
      "tolerance " <<
      _tolerance << ", maximum iterations " << max_iterations <<
      ", "
      "max on approach iterations " <<
      max_on_approach_iterations << ", and " <<
    (allow_unknown ? "allowing unknown traversal" :
    "not allowing unknown traversal") <<
      " Using motion model: " << toString(motion_model).c_str() << ".", );
}

void SmacPlanner2D::activate()
{
  ROS_INFO_STREAM("Activating plugin " << _name.c_str() << " of type SmacPlanner2D");
  _raw_plan_publisher->on_activate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
}

void SmacPlanner2D::deactivate()
{
  ROS_INFO_STREAM("Deactivating plugin " << _name.c_str() << " of type SmacPlanner2D");
  _raw_plan_publisher->on_deactivate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
}

void SmacPlanner2D::cleanup()
{
  ROS_INFO_STREAM("Cleaning up plugin " << _name.c_str() << " of type SmacPlanner2D");
  _a_star.reset();
  _smoother.reset();
  _costmap_downsampler->on_cleanup();
  _costmap_downsampler.reset();
  _raw_plan_publisher.reset();
}

nav_2d_msgs::Path2D SmacPlanner2D::makePlan(
  const nav_2d_msgs::Pose2DStamped & start, const nav_2d_msgs::Pose2DStamped & goal)
{
  steady_clock::time_point a = steady_clock::now();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
  }

  // Set Costmap
  _a_star->createGraph(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), 1, costmap);

  // Set starting point
  unsigned int mx, my;
  costmap->worldToMap(start.pose.x, start.pose.y, mx, my);
  _a_star->setStart(mx, my, 0);

  // Set goal point
  costmap->worldToMap(goal.pose.x, goal.pose.y, mx, my);
  _a_star->setGoal(mx, my, 0);

  // Setup message
  nav_msgs::Path plan;
  plan.header.stamp = ros::Time::now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  Node2D::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    ROS_WARN_STREAM(_name.c_str() << ": failed to create plan, " << error.c_str() << ".");
    return plan;
  }

  // Convert to world coordinates and downsample path for smoothing if necesssary
  // We're going to downsample by 4x to give terms room to move.
  const int downsample_ratio = 4;
  std::vector<Eigen::Vector2d> path_world;
  path_world.reserve(_smoother ? path.size() / downsample_ratio : path.size());
  plan.poses.reserve(_smoother ? path.size() / downsample_ratio : path.size());

  for (int i = path.size() - 1; i >= 0; --i) {
    if (_smoother && i % downsample_ratio != 0) {
      continue;
    }

    path_world.push_back(getWorldCoords(path[i].x, path[i].y, costmap));
    pose.pose.position.x = path_world.back().x();
    pose.pose.position.y = path_world.back().y();
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // If not smoothing or too short to smooth, return path
  if (!_smoother || path_world.size() < 4) {
#ifdef BENCHMARK_TESTING
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    std::cout << "It took " << time_span.count() * 1000 << " milliseconds with " <<
      num_iterations <<
      " iterations." << std::endl;
#endif
    return plan;
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());
  _smoother_params.max_time = std::min(time_remaining, _optimizer_params.max_time);

  // Smooth plan
  if (!_smoother->smooth(path_world, costmap, _smoother_params)) {
    ROS_WARN_STREAM(
      _name.c_str() <<
        ": failed to smooth plan, Ceres could not find a usable solution to optimize.");
    return plan;
  }

  removeHook(path_world);

  // populate final path
  for (uint i = 0; i != path_world.size(); i++) {
    pose.pose.position.x = path_world[i][0];
    pose.pose.position.y = path_world[i][1];
    plan.poses[i] = pose;
  }

  return plan;
}

void SmacPlanner2D::removeHook(std::vector<Eigen::Vector2d> & path)
{
  // Removes the end "hooking" since goal is locked in place
  Eigen::Vector2d interpolated_second_to_last_point;
  interpolated_second_to_last_point = (path.end()[-3] + path.end()[-1]) / 2.0;
  if (
    squaredDistance(path.end()[-2], path.end()[-1]) >
    squaredDistance(interpolated_second_to_last_point, path.end()[-1]))
  {
    path.end()[-2] = interpolated_second_to_last_point;
  }
}

Eigen::Vector2d SmacPlanner2D::getWorldCoords(
  const float & mx, const float & my, const costmap_2d::Costmap2D * costmap)
{
  float world_x = static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  float world_y = static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return Eigen::Vector2d(world_x, world_y);
}

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner2D, nav_core2::GlobalPlanner)
