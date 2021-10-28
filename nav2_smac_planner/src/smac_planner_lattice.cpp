// Copyright (c) 2021, Samsung Research America
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
#include "nav2_smac_planner/smac_planner_lattice.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT

SmacPlannerLattice::SmacPlannerLattice()
: _a_star(nullptr),
  _collision_checker(nullptr, 1),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerLattice::~SmacPlannerLattice()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerLattice",
    _name.c_str());
}

void SmacPlannerLattice::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerLattice", name.c_str());

  bool allow_unknown;
  int max_iterations;
  double lookup_table_size;
  SearchInfo search_info;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", max_iterations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lattice_filepath", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name + ".lattice_filepath", search_info.lattice_filepath);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".cache_obstacle_heuristic", search_info.cache_obstacle_heuristic);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", search_info.reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.15));
  node->get_parameter(name + ".change_penalty", search_info.change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.50));
  node->get_parameter(name + ".non_straight_penalty", search_info.non_straight_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(1.7));
  node->get_parameter(name + ".cost_penalty", search_info.cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  node->get_parameter(name + ".analytic_expansion_ratio", search_info.analytic_expansion_ratio);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".lookup_table_size", lookup_table_size);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_reverse_expansion", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".allow_reverse_expansion", search_info.allow_reverse_expansion);

  _metadata = LatticeMotionTable::getLatticeMetadata(search_info.lattice_filepath);
  search_info.minimum_turning_radius = _metadata.min_turning_radius;
  MotionModel motion_model = MotionModel::STATE_LATTICE;

  if (max_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  float lookup_table_dim =
    static_cast<float>(lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      lookup_table_dim);
    lookup_table_dim += 1.0;
  }

  // Initialize collision checker using 72 evenly sized bins instead of the lattice
  // heading angles. This is done so that we have precomputed angles every 5 degrees.
  // If we used the sparse lattice headings (usually 16), then when we attempt to collision
  // check for intermediary points of the primitives, we're forced to round to one of the 16
  // increments causing "wobbly" checks that could cause larger robots to virtually show collisions
  // in valid configurations. This approximation helps to bound orientation error for all checks
  // in exchange for slight inaccuracies in the collision headings in terminal search states.
  _collision_checker = GridCollisionChecker(_costmap, 72u);
  _collision_checker.setFootprint(
    costmap_ros->getRobotFootprint(),
    costmap_ros->getUseRadius(),
    findCircumscribedCost(costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeLattice>>(motion_model, search_info);
  _a_star->initialize(
    allow_unknown,
    max_iterations,
    std::numeric_limits<int>::max(),
    lookup_table_dim,
    _metadata.number_of_headings);

  // Initialize path smoother
  SmootherParams params;
  params.get(node, name);
  _smoother = std::make_unique<Smoother>(params);
  _smoother->initialize(_metadata.min_turning_radius);

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerLattice with "
    "maximum iterations %i, "
    "and %s. Using motion model: %s. State lattice file: %s.",
    _name.c_str(), max_iterations,
    allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    toString(motion_model).c_str(), search_info.lattice_filepath.c_str());
}

void SmacPlannerLattice::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
}

void SmacPlannerLattice::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
}

void SmacPlannerLattice::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  _costmap_downsampler->on_cleanup();
  _costmap_downsampler.reset();
  _raw_plan_publisher.reset();
}

nav_msgs::msg::Path SmacPlannerLattice::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  _a_star->setStart(
    mx, my,
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(start.pose.orientation)));

  // Set goal point, in A* bin search coordinates
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  _a_star->setGoal(
    mx, my,
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(goal.pose.orientation)));

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
  NodeLattice::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(path, num_iterations, 0 /*no tolerance*/)) {
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

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (num_iterations > 1 && plan.poses.size() > 6) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  return plan;
}

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerLattice, nav2_core::GlobalPlanner)
