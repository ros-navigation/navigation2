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

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "smac_planner/smac_planner.hpp"

#define BENCHMARK_TESTING

namespace smac_planner
{

using namespace std::chrono;  // NOLINT

SmacPlanner::SmacPlanner()
: _a_star(nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlanner::~SmacPlanner()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlanner",
    _name.c_str());
}

void SmacPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  bool allow_unknown;
  int max_iterations;
  int max_on_approach_iterations = std::numeric_limits<int>::max();
  int angle_quantizations;
  SearchInfo search_info;
  bool smooth_path;
  std::string motion_model_for_search;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.125));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
  node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(-1));
  node->get_parameter(name + ".max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".smooth_path", smooth_path);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.2));
  node->get_parameter(name + ".minimum_turning_radius", search_info.minimum_turning_radius);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", search_info.reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".change_penalty", search_info.change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.05));
  node->get_parameter(name + ".non_straight_penalty", search_info.non_straight_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(1.2));
  node->get_parameter(name + ".cost_penalty", search_info.cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".analytic_expansion_ratio", search_info.analytic_expansion_ratio);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time_ms", rclcpp::ParameterValue(5000.0));
  node->get_parameter(name + ".max_planning_time_ms", _max_planning_time);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  node->get_parameter(name + ".motion_model_for_search", motion_model_for_search);
  MotionModel motion_model = fromString(motion_model_for_search);
  if (motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
      motion_model_for_search.c_str());
  }

  if (max_on_approach_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (max_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  // convert to grid coordinates
  const double minimum_turning_radius_global_coords = search_info.minimum_turning_radius;
  search_info.minimum_turning_radius =
    search_info.minimum_turning_radius / (_costmap->getResolution() * _downsampling_factor);

  _a_star = std::make_unique<AStarAlgorithm<NodeSE2>>(motion_model, search_info);
  _a_star->initialize(
    allow_unknown,
    max_iterations,
    max_on_approach_iterations);
  _a_star->setFootprint(costmap_ros->getRobotFootprint(), costmap_ros->getUseRadius());

  if (smooth_path) {
    _smoother = std::make_unique<Smoother>();
    _optimizer_params.get(node.get(), name);
    _smoother_params.get(node.get(), name);
    _smoother_params.max_curvature = 1.0f / minimum_turning_radius_global_coords;
    _smoother->initialize(_optimizer_params);
  }

  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlanner with "
    "tolerance %.2f, maximum iterations %i, "
    "max on approach iterations %i, and %s. Using motion model: %s.",
    _name.c_str(), _tolerance, max_iterations, max_on_approach_iterations,
    allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    toString(motion_model).c_str());
}

void SmacPlanner::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlanner",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
}

void SmacPlanner::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlanner",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
}

void SmacPlanner::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlanner",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  _costmap_downsampler->on_cleanup();
  _costmap_downsampler.reset();
  _raw_plan_publisher.reset();
}

nav_msgs::msg::Path SmacPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
  }

  // Set Costmap
  _a_star->createGraph(
    costmap->getSizeInCellsX(),
    costmap->getSizeInCellsY(),
    _angle_quantizations,
    costmap);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setStart(mx, my, orientation_bin_id);

  // Set goal point, in A* bin search coordinates
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setGoal(mx, my, orientation_bin_id);

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  NodeSE2::CoordinateVector path;
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
    RCLCPP_WARN(
      _logger,
      "%s: failed to create plan, %s.",
      _name.c_str(), error.c_str());
    return plan;
  }

  // Convert to world coordinates and downsample path for smoothing if necesssary
  // We're going to downsample by 4x to give terms room to move.
  const int downsample_ratio = 4;
  std::vector<Eigen::Vector2d> path_world;
  path_world.reserve(path.size());
  plan.poses.reserve(path.size());

  for (int i = path.size() - 1; i >= 0; --i) {
    path_world.push_back(getWorldCoords(path[i].x, path[i].y, costmap));
    pose.pose.position.x = path_world.back().x();
    pose.pose.position.y = path_world.back().y();
    pose.pose.orientation = getWorldOrientation(path[i].theta);
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
    std::cout << "It took " << time_span.count() * 1000 <<
      " milliseconds with " << num_iterations << " iterations." << std::endl;
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
    RCLCPP_WARN(
      _logger,
      "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      _name.c_str());
    return plan;
  }

  removeHook(path_world);

  // populate final path
  // TODO(stevemacenski): set orientation to tangent of path
  for (unsigned int i = 0; i != path_world.size(); i++) {
    pose.pose.position.x = path_world[i][0];
    pose.pose.position.y = path_world[i][1];
    plan.poses[i] = pose;
  }

  return plan;
}

void SmacPlanner::removeHook(std::vector<Eigen::Vector2d> & path)
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

Eigen::Vector2d SmacPlanner::getWorldCoords(
  const float & mx, const float & my, const nav2_costmap_2d::Costmap2D * costmap)
{
  // mx, my are in continuous grid coordinates, must convert to world coordinates
  double world_x =
    static_cast<double>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  double world_y =
    static_cast<double>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return Eigen::Vector2d(world_x, world_y);
}

geometry_msgs::msg::Quaternion SmacPlanner::getWorldOrientation(const float & theta)
{
  // theta is in continuous bin coordinates, must convert to world orientation
  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta * static_cast<double>(_angle_bin_size));
  return tf2::toMsg(q);
}

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner, nav2_core::GlobalPlanner)
