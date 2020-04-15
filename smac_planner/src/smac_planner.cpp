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

// benefits list:
//  - for tolerance, only search once (max iterations on appraoch meeting tolerance), if set tol low and iterations low then can use to actually compute to a scale before target
//      e.g. compute for the next ~N meters only on way towards something 
//  - Against NavFns: we have inflation + dynamic processing: cached gradiant map not used. reusibility, cannot be built on for nonholonomic, no looping or weird artifacts
//  - common building blocks for use in all planners to maximize reliability, stress test, and reduce likelihood of bugs
//  - not searching then backtracing with grad descent for 2x go through
//  - lower memory (?) and faster (?)
//  - modern data structures & carefully optimized & generic for use in other planning problems
//  - generic smoother that has applications to anything
//  - smoother costmap aware (vs bezier, splines, b-splines, etc) 
//  - caching paths rather than recomputing needlessly if they're still good
//  - network planner & arbitrary nonholonomic including ackermann
//  - non-circular footprints, diff/omni/ackermann, covering all classes of ground robots. circl diff/omni A*, ackerman hybrid, arbitrary diff/omni A* if relatively small, hybrid is large
//  - dials for Astar quality (can be quick and dirty or slow and smooth) then dials for the optimizer to suit (quick once over, or really smooth out a jazzed path)
//  - disable max iterations / tolerance with 0 / -1
//  - max time for soft gaurentees on planning and smoothing times, time tracking
//  - Do low potential field in all areas -- this should be the new defacto-default (really should have been already but ppl ignore it). Footprint + inflation important
//  - describe why and when on the 4 vs 8 connected
//  - plots of pts that violate over iterations (curve, dist > thresh, smooth > dist, cost > thresh)
// - Need to boil down statements about why I did this, clear benefits, and drawbacks of current approaches / solutions
//  - Identified 3 math errors of Thrun

// astar timeout, max duration, optimizer gets rest or until its set maximum. Test time before/after A* but not in it, that would slow down. if over, send log warning like DWB

// if collision in smoothed path, anchor that point and then re-run until successful (helpful in narrow spaces).
// try vornoi from dynamic vornoi && if works, optimize it && put into vornoi layer in costmap 2d (how with struct / non char*?)

// NOTES 
// way to do collision checking on oriented footprint https://github.com/windelbouwman/move-base-ompl/blob/master/src/ompl_global_planner.cpp#L133 (but doesnt cache)

#include <string>
#include <memory>
#include <vector>
#include "Eigen/Core"
#include "smac_planner/smac_planner.hpp"

#define BENCHMARK_TESTING

namespace smac_planner
{
using namespace std::chrono;
using namespace std;

inline double squaredDistance(
  const Eigen::Vector2d & p1,
  const Eigen::Vector2d & p2)
{
  const double & dx = p1[0] - p2[0];
  const double & dy = p1[1] - p2[1];
  return dx * dx + dy * dy;
}

SmacPlanner::SmacPlanner()
: _a_star(nullptr),
  _smoother(nullptr),
  _upsampler(nullptr),
  _node(nullptr),
  _costmap(nullptr)
{
}

SmacPlanner::~SmacPlanner()
{
  RCLCPP_INFO(
    _node->get_logger(), "Destroying plugin %s of type SmacPlanner",
    _name.c_str());
}

void SmacPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  bool allow_unknown;
  int max_iterations;
  int max_on_approach_iterations;
  float travel_cost_scale;
  bool smooth_path;
  bool upsample_path;
  std::string neighborhood_for_search;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".tolerance", rclcpp::ParameterValue(0.125));
  _tolerance = static_cast<float>(_node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  _node->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".max_iterations", rclcpp::ParameterValue(-1)); /*TODO set reasoanble number, also, per request depending on length?*/
  _node->get_parameter(name + ".max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".travel_cost_scale", rclcpp::ParameterValue(0.8));
  _node->get_parameter(name + ".travel_cost_scale", travel_cost_scale);
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
    _node, name + ".smoother.sampling_ratio", rclcpp::ParameterValue(4));
  _node->get_parameter(name + ".smoother.sampling_ratio", _sampling_ratio);

  nav2_util::declare_parameter_if_not_declared(
    _node, name + ".neighborhood_for_search", rclcpp::ParameterValue(std::string("MOORE")));
  _node->get_parameter(name + ".neighborhood_for_search", neighborhood_for_search);
  Neighborhood neighborhood;
  if (neighborhood_for_search == std::string("MOORE")) {
    neighborhood = Neighborhood::MOORE;
  } else if (neighborhood_for_search == std::string("VAN_NEUMANN")) {
    neighborhood = Neighborhood::VAN_NEUMANN;
  } else {
    neighborhood = Neighborhood::MOORE;
    RCLCPP_WARN(_node->get_logger(),
      "Unable to get Neighborhood search type. Given '%s', "
      "valid options are MOORE and VAN_NEUMANN. Using MOORE as default",
      neighborhood_for_search.c_str());
  }

  if (max_on_approach_iterations <= 0) {
    RCLCPP_INFO(_node->get_logger(), "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (max_iterations <= 0) {
    RCLCPP_INFO(_node->get_logger(), "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  if (travel_cost_scale > 1.0 || travel_cost_scale < 0.0) {
    RCLCPP_FATAL(_node->get_logger(), "Travel cost scale must be between 0 and 1, exiting.");
    exit(-1);
  }

  _a_star = std::make_unique<AStarAlgorithm>(neighborhood);
  _a_star->initialize(
    travel_cost_scale,
    allow_unknown,
    max_iterations,
    max_on_approach_iterations);

  if (smooth_path) {
    _smoother = std::make_unique<Smoother>();
    _optimizer_params.get(_node.get(), name);  // Get optimizer params        // TODO per-run with time left over
    _smoother_params.get(_node.get(), name);  // Get weights
    _smoother->initialize(_optimizer_params);

    if (upsample_path) {
      _upsampler = std::make_unique<Upsampler>();
      _upsampler->initialize(_optimizer_params);
    }
  }

  _raw_plan_publisher = _node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);
  smoother_debug1_pub_= _node->create_publisher<nav_msgs::msg::Path>("debug1", 1);
  smoother_debug2_pub_= _node->create_publisher<nav_msgs::msg::Path>("debug2", 1);
  smoother_debug3_pub_= _node->create_publisher<nav_msgs::msg::Path>("debug3", 1);

  RCLCPP_INFO(
    _node->get_logger(), "Configured plugin %s of type SmacPlanner with "
    "travel cost %.2f, tolerance %.2f, maximum iterations %i, "
    "max on approach iterations %i, and %s. Using neighorhood: %s.",
    _name.c_str(), travel_cost_scale, _tolerance, max_iterations, max_on_approach_iterations,
    allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    toString(neighborhood).c_str());
}

void SmacPlanner::activate()
{
  RCLCPP_INFO(
    _node->get_logger(), "Activating plugin %s of type SmacPlanner",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  smoother_debug1_pub_->on_activate();
  smoother_debug2_pub_->on_activate();
  smoother_debug3_pub_->on_activate();
}

void SmacPlanner::deactivate()
{
  RCLCPP_INFO(
    _node->get_logger(), "Deactivating plugin %s of type SmacPlanner",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
}

void SmacPlanner::cleanup()
{
  RCLCPP_INFO(
    _node->get_logger(), "Cleaning up plugin %s of type SmacPlanner",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  _upsampler.reset();
  _raw_plan_publisher.reset(); 
}

nav_msgs::msg::Path SmacPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif

  // Set Costmap
  unsigned char * char_costmap = _costmap->getCharMap();
  _a_star->setCosts(
    _costmap->getSizeInCellsX(),
    _costmap->getSizeInCellsY(),
    char_costmap);

  // Set starting point
  unsigned int mx, my, index;
  _costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  index = _costmap->getIndex(mx, my);
  _a_star->setStart(index);

  // Set goal point
  _costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  index = _costmap->getIndex(mx, my);
  _a_star->setGoal(index);

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
  IndexPath path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
      path, num_iterations, _tolerance / static_cast<float>(_costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found.");
      } else {
        error = std::string("exceeded maximum iterations.");
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
  std::vector<Eigen::Vector2d> path_world;
  path_world.reserve(path.size() / _sampling_ratio);
  plan.poses.reserve(path.size() / _sampling_ratio);

  for (int i = path.size() - 1; i >= 0; --i) {
    if (_smoother && i % _sampling_ratio != 0) {
      continue;
    }
    unsigned int index_x, index_y;
    double world_x, world_y;
    _costmap->indexToCells(path[i], index_x, index_y);
    _costmap->mapToWorld(index_x, index_y, world_x, world_y);
    path_world.push_back(Eigen::Vector2d(world_x, world_y));
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_node->count_subscribers(_raw_plan_publisher->get_topic_name()) > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // Smooth plan
  if (_smoother && path_world.size() > 4) {
    MinimalCostmap mcmap(char_costmap, _costmap->getSizeInCellsX(),
      _costmap->getSizeInCellsY(), _costmap->getOriginX(), _costmap->getOriginY(),
      _costmap->getResolution());
    if (!_smoother->smooth(path_world, & mcmap, _smoother_params)) {
      RCLCPP_WARN(
        _node->get_logger(),
        "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
        _name.c_str());
      return plan;
    }

    removeHook(path_world);

///////////////////////////////// DEBUG/////////////////////////////////
    for (int i = 0; i != path_world.size(); i++) {
      pose.pose.position.x = path_world[i][0];
      pose.pose.position.y = path_world[i][1];
      plan.poses[i] = pose;
    }
    smoother_debug1_pub_->publish(plan);
///////////////////////////////// DEBUG/////////////////////////////////

    // Upsample path
    if (_upsampler) {
      if(!_upsampler->upsample(path_world, _smoother_params, _sampling_ratio))
      {
        RCLCPP_WARN(
          _node->get_logger(),
          "%s: failed to upsample plan, Ceres could not find a usable solution to optimize.",
          _name.c_str());
      } else {
        plan.poses.resize(path_world.size());
      }
    }
  } else {
    return plan;
  }

///////////////////////////////// DEBUG/////////////////////////////////

  // // move back below to do multiple runs testing
  // std::vector<Eigen::Vector2d> path_world_debug1 = path_world;
  // std::vector<Eigen::Vector2d> path_world_debug2 = path_world;


  // if (path_world_debug1.size() > 5) {
  //   SmootherParams params(15000.0 /*smooth*/, 0.0  /*cost*/, 300.0 /*dist*/, 30.0 /*curve*/, 4.0 /*max curve*/, 10.0 /*costmap factor*/);
  //   _smoother->smooth(path_world_debug1, & mcmap, params);
  // }
  //   if (path_world_debug1.size() > 3) {
  //     Eigen::Vector2d interpolated_second_to_last_point;
  //     interpolated_second_to_last_point = (path_world_debug1.end()[-3] + path_world_debug1.end()[-1]) / 2.0;

  //     if (
  //       squaredDistance(path_world_debug1.end()[-2], path_world_debug1.end()[-1]) >
  //       squaredDistance(interpolated_second_to_last_point, path_world_debug1.end()[-1])) {
  //       path_world_debug1.end()[-2] = interpolated_second_to_last_point;
  //     }
  //   }

  // for (int i = 0; i != path_world_debug1.size(); i++) {
  //   pose.pose.position.x = path_world_debug1[i][0];
  //   pose.pose.position.y = path_world_debug1[i][1];
  //   plan.poses[i] = pose;
  // }
  // smoother_debug1_pub_->publish(plan);

  // if (path_world_debug2.size() > 5) {
  //   SmootherParams params(15000.0 /*smooth*/, 0.007 /*cost*/, 50.0 /*dist*/, 30.0 /*curve*/, 4.0 /*max curve*/, 10.0 /*costmap factor*/);
  //   _smoother->smooth(path_world_debug2, & mcmap, params);
  // }
  //   if (path_world_debug2.size() > 3) {
  //     Eigen::Vector2d interpolated_second_to_last_point;
  //     interpolated_second_to_last_point = (path_world_debug2.end()[-3] + path_world_debug2.end()[-1]) / 2.0;

  //     if (squaredDistance(path_world_debug2.end()[-2], path_world_debug2.end()[-1]) > squaredDistance(interpolated_second_to_last_point, path_world_debug2.end()[-1])) {
  //       path_world_debug2.end()[-2] = interpolated_second_to_last_point;
  //     }
  //   }
  // for (int i = 0; i != path_world_debug2.size(); i++) {
  //   pose.pose.position.x = path_world_debug2[i][0];
  //   pose.pose.position.y = path_world_debug2[i][1];
  //   plan.poses[i] = pose;
  // }
  // smoother_debug2_pub_->publish(plan);

  // // second stage a second time
  // if (path_world_debug2.size() > 5) {
  //   SmootherParams params(15000.0 /*smooth*/, 0.007 /*cost*/, 50.0 /*dist*/, 30.0 /*curve*/, 4.0 /*max curve*/, 10.0 /*costmap factor*/);
  //   _smoother->smooth(path_world_debug2, & mcmap, params);
  // }

  // for (int i = 0; i != path_world_debug2.size(); i++) {
  //   pose.pose.position.x = path_world_debug2[i][0];
  //   pose.pose.position.y = path_world_debug2[i][1];
  //   plan.poses[i] = pose;
  // }

  // smoother_debug3_pub_->publish(plan);
//////////////////////////////// DEBUG/////////////////////////////////

  for (int i = 0; i != plan.poses.size(); i++) {
    pose.pose.position.x = path_world[i][0];
    pose.pose.position.y = path_world[i][1];
    plan.poses[i] = pose;
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double> >(b-a);
  cout << "It took " << time_span.count() <<
    " seconds with " << num_iterations << " iterations." <<  endl;
#endif

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

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner, nav2_core::GlobalPlanner)
