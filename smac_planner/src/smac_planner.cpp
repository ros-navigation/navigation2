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
//  - Against NavFns: we have inflation + dynamic processing: cached gradiant map not used. reusibility, cannot be built on for nonholonomic
//  - not searching then backtracing with grad descent for 2x go through
//  - lower memory (?) and faster (?)
//  - modern data structures & carefully optimized & generic for use in other planning problems
//  - generic smoother that has applications to anything
//  - caching paths rather than recomputing needlessly if they're still good
//  - network planner
//  - non-circular footprints, diff/omni/ackermann, covering all classes of ground robots. circl diff/omni A*, ackerman hybrid, arbitrary diff/omni A* if relatively small, hybrid is large
//  - dials for Astar quality (can be quick and dirty or slow and smooth) then dials for the optimizer to suit (quick once over, or really smooth out a jazzed path)
//  - disable max iterations / tolerance with 0 / -1
//  - Do low potential field in all areas -- this should be the new defacto-default (really should have been already but ppl ignore it). Footprint + inflation important
//  - describe why and when on the 4 vs 8 connected
//  - plots of pts that violate over iterations (curve, dist > thresh, smooth > dist, cost > thresh)

// TODOs
// total cost path caching
// astar timeout, max duration, optimizer gets rest or until its set maximum
// crashing

// NOTES 
// way to do collision checking on oriented footprint https://github.com/windelbouwman/move-base-ompl/blob/master/src/ompl_global_planner.cpp#L133 (but doesnt cache)
//   server  for smoothing alone
//  separate tuning server instance for refining for your needs
// if switching direction, discount all terms in hybrid in smoother

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
  double dx = p1[0] - p2[0];
  double dy = p1[1] - p2[1];
  return dx * dx + dy * dy;
}

SmacPlanner::SmacPlanner()
: a_star_(nullptr),
  smoother_(nullptr),
  node_(nullptr),
  costmap_(nullptr)
{
}

SmacPlanner::~SmacPlanner()
{
  RCLCPP_INFO(
    node_->get_logger(), "Destroying plugin %s of type SmacPlanner",
    name_.c_str());
}

void SmacPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  costmap_ = costmap_ros->getCostmap();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  bool allow_unknown;
  int max_iterations;
  int max_on_approach_iterations;
  float travel_cost_scale;
  bool smooth_path;
  std::string neighborhood_for_search;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".tolerance", rclcpp::ParameterValue(0.125));
  tolerance_ = static_cast<float>(node_->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node_->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".max_iterations", rclcpp::ParameterValue(-1)); /*TODO set reasoanble number, also, per request depending on length?*/
  node_->get_parameter(name + ".max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".travel_cost_scale", rclcpp::ParameterValue(0.8));
  node_->get_parameter(name + ".travel_cost_scale", travel_cost_scale);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node_->get_parameter(name + ".max_on_approach_iterations", max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".smooth_path", rclcpp::ParameterValue(true));
  node_->get_parameter(name + ".smooth_path", smooth_path);

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".neighborhood_for_search", rclcpp::ParameterValue(std::string("MOORE")));
  node_->get_parameter(name + ".neighborhood_for_search", neighborhood_for_search);
  Neighborhood neighborhood;
  if (neighborhood_for_search == std::string("MOORE")) {
    neighborhood = Neighborhood::MOORE;
  } else if (neighborhood_for_search == std::string("VAN_NEUMANN")) {
    neighborhood = Neighborhood::VAN_NEUMANN;
  } else {
    neighborhood = Neighborhood::MOORE;
    RCLCPP_WARN(node_->get_logger(),
      "Unable to get Neighborhood search type. Given '%s', "
      "valid options are MOORE and VAN_NEUMANN. Using MOORE as default",
      neighborhood_for_search.c_str());
  }

  a_star_ = std::make_unique<AStarAlgorithm>(neighborhood);
  a_star_->initialize(
    travel_cost_scale,
    allow_unknown,
    max_iterations,
    revisit_neighbors,
    max_on_approach_iterations);

  if (smooth_path) {
    smoother_ = std::make_unique<CGSmoother>();
    smoother_->initialize(debug_optimizer);
  }

  raw_plan_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  RCLCPP_INFO(
    node_->get_logger(), "Configured plugin %s of type SmacPlanner with "
    "travel cost %.2f, tolerance %.2f, maximum iterations %i, "
    "max on appraoch iterations %i, and %s. %s. Using neighorhood: %s.",
    name_.c_str(), travel_cost_scale, tolerance_, max_iterations, max_on_approach_iterations,
    revisit_neighbors ? "Sllowing to revisit neighbors" : "Not allowing revisit of neighbors",
    allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    toString(neighborhood).c_str());
}

void SmacPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type SmacPlanner",
    name_.c_str());
  raw_plan_publisher_->on_activate();    
}

void SmacPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type SmacPlanner",
    name_.c_str());
  raw_plan_publisher_->on_deactivate();
}

void SmacPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type SmacPlanner",
    name_.c_str());
  a_star_.reset();
  smoother_.reset();
  raw_plan_publisher_.reset(); 
}

nav_msgs::msg::Path SmacPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif

  // Set Costmap
  unsigned char * char_costmap = costmap_->getCharMap();
  a_star_->setCosts(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY(),
    char_costmap);

  // Set starting point
  unsigned int mx, my, index;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  index = costmap_->getIndex(mx, my);
  a_star_->setStart(index);

  // Set goal point
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  index = costmap_->getIndex(mx, my);
  a_star_->setGoal(index);

  nav_msgs::msg::Path plan;
  plan.header.stamp = node_->now();
  plan.header.frame_id = global_frame_;
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
    if (!a_star_->createPath(
      path, num_iterations, tolerance_ / static_cast<float>(costmap_->getResolution())))
    {
      if (num_iterations < a_star_->getMaxIterations()) {
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
      node_->get_logger(),
      "%s: failed to create plan, %s.",
      name_.c_str(), error.c_str());
    return plan;
  }

  // Downsample to for smoothing
  const int downsample_ratio = 2;
  std::vector<Eigen::Vector2d> path_world;
  path_world.reserve(path.size() / downsample_ratio);
  plan.poses.reserve(path.size() / downsample_ratio);

  for (int i = path.size() - 1; i >= 0; --i) {
    if (i % downsample_ratio == 0) {
      continue;
    }
    unsigned int index_x, index_y;
    double world_x, world_y;
    costmap_->indexToCells(path[i], index_x, index_y);
    costmap_->mapToWorld(index_x, index_y, world_x, world_y);
    path_world.push_back(Eigen::Vector2d(world_x, world_y));
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    plan.poses.push_back(pose);
  }

  if (node_->count_subscribers(raw_plan_publisher_->get_topic_name()) > 0) {
    raw_plan_publisher_->publish(plan);
  }

  // Smooth plan
  if (smoother_ && path_world.size() > 5) {
    MinimalCostmap mcmap(char_costmap, costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY(), costmap_->getOriginX(), costmap_->getOriginY(),
      costmap_->getResolution());
    if (!smoother_->smooth(path_world, & mcmap)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
        name_.c_str());
      return plan;
    }
  } else {
    return plan;
  }

  for (int i = 0; i != path_world.size(); i++) {
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

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner, nav2_core::GlobalPlanner)
