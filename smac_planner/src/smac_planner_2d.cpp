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
#include <limits>
#include <algorithm>

#include "smac_planner/smac_planner_2d.hpp"

#define BENCHMARK_TESTING

namespace smac_planner
{
using namespace std::chrono;  // NOLINT

SmacPlanner2D::SmacPlanner2D()
: _a_star(nullptr),
  _smoother(nullptr),
  _upsampler(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr),
  _node(nullptr)
{
}

SmacPlanner2D::~SmacPlanner2D()
{
  RCLCPP_INFO(
    _node->get_logger(), "Destroying plugin %s of type SmacPlanner2D",
    _name.c_str());
}

void SmacPlanner2D::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  bool allow_unknown;
  int max_iterations;
  int max_on_approach_iterations;
  bool smooth_path;
  bool upsample_path;
  std::string motion_model_for_search;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".tolerance", rclcpp::ParameterValue(0.125));
  _tolerance = static_cast<float>(_node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".downsample_costmap", rclcpp::ParameterValue(true));
  _node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  _node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  _node->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".max_iterations", rclcpp::ParameterValue(-1));
  _node->get_parameter(name + ".max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  _node->get_parameter(name + ".max_on_approach_iterations", max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".smooth_path", rclcpp::ParameterValue(true));
  _node->get_parameter(name + ".smooth_path", smooth_path);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".upsample_path", rclcpp::ParameterValue(false));
  _node->get_parameter(name + ".upsample_path", upsample_path);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".smoother.upsampling_ratio", rclcpp::ParameterValue(2));
  _node->get_parameter(name + ".smoother.upsampling_ratio", _upsampling_ratio);

  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".max_planning_time_ms", rclcpp::ParameterValue(1000.0));
  _node->get_parameter(name + ".max_planning_time_ms", _max_planning_time);

  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("MOORE")));
  _node->get_parameter(name + ".motion_model_for_search", motion_model_for_search);
  MotionModel motion_model = fromString(motion_model_for_search);
  if (motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _node->get_logger(),
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
      motion_model_for_search.c_str());
  }

  if (max_on_approach_iterations <= 0) {
    RCLCPP_INFO(
      _node->get_logger(), "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (max_iterations <= 0) {
    RCLCPP_INFO(
      _node->get_logger(), "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  if (_upsampling_ratio != 2 && _upsampling_ratio != 4) {
    RCLCPP_WARN(
      _node->get_logger(),
      "Upsample ratio set to %i, only 2 and 4 are valid. Defaulting to 2.", _upsampling_ratio);
    _upsampling_ratio = 2;
  }

  _a_star = std::make_unique<AStarAlgorithm<Node2D>>(motion_model, SearchInfo());
  _a_star->initialize(
    allow_unknown,
    max_iterations,
    max_on_approach_iterations);

  if (smooth_path) {
    _smoother = std::make_unique<Smoother>();
    _optimizer_params.get(_node.get(), name);
    _smoother_params.get(_node.get(), name);
    _smoother->initialize(_optimizer_params);

    if (upsample_path && _upsampling_ratio > 0) {
      _upsampler = std::make_unique<Upsampler>();
      _upsampler->initialize(_optimizer_params);
    }
  }

  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<CostmapDownsampler>(_node);
    _costmap_downsampler->initialize(_global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = _node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);
  _smoothed_plan_publisher = _node->create_publisher<nav_msgs::msg::Path>("smoothed_plan", 1);

  RCLCPP_INFO(
    _node->get_logger(), "Configured plugin %s of type SmacPlanner2D with "
    "tolerance %.2f, maximum iterations %i, "
    "max on approach iterations %i, and %s. Using motion model: %s.",
    _name.c_str(), _tolerance, max_iterations, max_on_approach_iterations,
    allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    toString(motion_model).c_str());
}

void SmacPlanner2D::activate()
{
  RCLCPP_INFO(
    _node->get_logger(), "Activating plugin %s of type SmacPlanner2D",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  _smoothed_plan_publisher->on_activate();
  if (_costmap_downsampler) {
    _costmap_downsampler->activatePublisher();
  }
}

void SmacPlanner2D::deactivate()
{
  RCLCPP_INFO(
    _node->get_logger(), "Deactivating plugin %s of type SmacPlanner2D",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  _smoothed_plan_publisher->on_deactivate();
  if (_costmap_downsampler) {
    _costmap_downsampler->deactivatePublisher();
  }
}

void SmacPlanner2D::cleanup()
{
  RCLCPP_INFO(
    _node->get_logger(), "Cleaning up plugin %s of type SmacPlanner2D",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  _upsampler.reset();
  _costmap_downsampler.reset();
  _raw_plan_publisher.reset();
  _smoothed_plan_publisher.reset();
}

nav_msgs::msg::Path SmacPlanner2D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Choose which costmap to use for the planning
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
  }

  // Set Costmap
  _a_star->createGraph(
    costmap->getSizeInCellsX(),
    costmap->getSizeInCellsY(),
    1,
    costmap);

  // Set starting point
  unsigned int mx, my;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  _a_star->setStart(mx, my, 0);

  // Set goal point
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  _a_star->setGoal(mx, my, 0);

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _node->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
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
    RCLCPP_WARN(
      _node->get_logger(),
      "%s: failed to create plan, %s.",
      _name.c_str(), error.c_str());
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
  if (_node->count_subscribers(_raw_plan_publisher->get_topic_name()) > 0) {
    _raw_plan_publisher->publish(plan);
  }

  if (!_smoother) {
#ifdef BENCHMARK_TESTING
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    std::cout << "It took " << time_span.count() * 1000 <<
      " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif
    return plan;
  }

  // if too small, return path
  if (path_world.size() < 4) {
    return plan;
  }

  // Find how much time we have left to do upsampling
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());
  _smoother_params.max_time = std::min(time_remaining, _optimizer_params.max_time);

  // Smooth plan
  if (!_smoother->smooth(path_world, costmap, _smoother_params)) {
    RCLCPP_WARN(
      _node->get_logger(),
      "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      _name.c_str());
    return plan;
  }

  removeHook(path_world);

  // Publish smoothed path for debug
  if (_node->count_subscribers(_smoothed_plan_publisher->get_topic_name()) > 0) {
    for (uint i = 0; i != path_world.size(); i++) {
      pose.pose.position.x = path_world[i][0];
      pose.pose.position.y = path_world[i][1];
      plan.poses[i] = pose;
    }
    _smoothed_plan_publisher->publish(plan);
  }

  // Find how much time we have left to do upsampling
  b = steady_clock::now();
  time_span = duration_cast<duration<double>>(b - a);
  time_remaining = _max_planning_time - static_cast<double>(time_span.count());
  _smoother_params.max_time = std::min(time_remaining, _optimizer_params.max_time);

  // Upsample path
  if (_upsampler) {
    if (!_upsampler->upsample(path_world, _smoother_params, _upsampling_ratio)) {
      RCLCPP_WARN(
        _node->get_logger(),
        "%s: failed to upsample plan, Ceres could not find a usable solution to optimize.",
        _name.c_str());
    } else {
      plan.poses.resize(path_world.size());
    }
  }

  for (uint i = 0; i != plan.poses.size(); i++) {
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
  const float & mx, const float & my, const nav2_costmap_2d::Costmap2D * costmap)
{
  float world_x =
    static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  float world_y =
    static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return Eigen::Vector2d(world_x, world_y);
}

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner2D, nav2_core::GlobalPlanner)
