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

// INTO 3D / dynamics respecting
//   graph no longer regular
//   graph with quad tree? multiresolution?
//   footprint / collision cehcking quickly (plugin?)
//   plugin-based search criteria (4, 8, motion projection). or graph nodes has getNeighbors method with overridden implementation. the edges shuld be stored in teh graph but we dont here because its extra memory and we know the tructure to search. In general though, the graph should have that represented.
//     then its just the node that is the plugin interface? have implementations for getNeighbors & inCollision
//     I think that would make it so the same A* planner with different node implementations could support general A* and hybrid A* for circular and non-circular
//     and the graph could be non-regular or non-grid (input though could generalize?)

// INTO SPARSE
//   graph without grid structure
//   topographical
//   PRM
//   voronoi
//   freespace planner + sparse channel planner (lanes)
//   lane planners (find closest gateway to you and to goal, plan to closest gateway, use sparse graph serach to gateway near goal, exit to goal plan)
//     mixing topographical with free space planners
//     ROS plugin layer: look at map semantic labels to find the gateways, call the algorithm to do free place planning, call another alogrithm for non-grid network graph (?)
//     maybe a PRM thought experiment would help to see if I could get them both to work with 1 implementation but 2 different graph nodes on input, since PRM is a less structured road-network graph

// WHAT all should this do for release / tutorial
//  - A* better than NavFn

// WHAT all should this do for namesake?
//  - A* better than NavFn
//  - Plugins to support Hybrid sampling and collision checking
//  - either: lane/topographical support. Meaning to have "go to X" or have "use lanes" on a sparse given network (similar to PRM with narative and way of representing it in map file)
//      - if both: msg field for "use network", field for string of topographical name, param for metric on when to use network
//      - assumption in lanes that straight field projections are feasible for the robot type (ei can follow straight line graph without getting stuck)
//      - param to enable dynamic planning in network if required? Break out of network and do full freespace upon failure?
//        - when? how? is this going to far down the rabbit hole to use-case specific?
//        - should it try to free space navigate then to the next closest gateway and try again?
//      - still allows for collision avoidance controller to deviate to get around stuff
//      - topographical support for file format needed in both, so why not enable both? the "go to X" will require BT and navigation task changes too

// benefits list:
//  - for tolerance, only search once
//  - Against NavFns: we have inflation + dynamic processing: cached gradiant map not used
//  - not searching then backtracing with grad descent for 2x go through
//  - lower memory (?) and faster (?)
//  - modern data structures & carefully optimized & generic for use in other planning problems
//  - generic smoother that has applications to anything

// no costmap in planning stage (?)
//   In optimization as voronoi.
//   Talk: see problem, A* general & fast, optimization, sampling, sparse.

// hybrid A*, be more sparse in search, optimize, then CG upsample with validity checks (or spline fitting)
// maybe do for A* too? faster search if courser resolution, then fitting faster as smaller, then upsampling with light weigh geometry

#include <string>
#include <memory>
#include <vector>
#include "smac_planner/smac_planner.hpp"

// For getting times
#define BENCHMARK_TESTING

namespace smac_planner
{
using namespace std::chrono;
using namespace std;

SmacPlanner::SmacPlanner()
: a_star_(nullptr),
  smoother_(nullptr),
  tf_(nullptr),
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
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  bool allow_unknown;
  int max_iterations;
  int max_on_approach_iterations;
  float travel_cost_scale;
  bool revisit_neighbors;
  bool smooth_path;
  std::string neighborhood_for_search;

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".tolerance", rclcpp::ParameterValue(0.25));
  tolerance_ = static_cast<float>(node_->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node_->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".max_iterations", rclcpp::ParameterValue(2000)); /*TODO set reasoanble number, also, per request depending on length?*/
  node_->get_parameter(name + ".max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".travel_cost_scale", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name + ".travel_cost_scale", travel_cost_scale);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".revisit_neighbors", rclcpp::ParameterValue(true)); /* TODO do CPU testing on large maps, paths seem permissible and similar CPU in short */
  node_->get_parameter(name + ".revisit_neighbors", revisit_neighbors);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".max_on_approach_iterations", rclcpp::ParameterValue(200));
  node_->get_parameter(name + ".max_on_approach_iterations", max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".smooth_path", rclcpp::ParameterValue(true)); /*TODO default false*/
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
    smoother_->initialize(true /*debug logging*/);    
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
  raw_plan_publisher_.reset();
}

nav_msgs::msg::Path SmacPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif

  unsigned char * char_costmap = costmap_->getCharMap();
  a_star_->setCosts(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY(),
    char_costmap);

  unsigned int mx, my, index;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  index = costmap_->getIndex(mx, my);
  a_star_->setStart(index);

  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  index = costmap_->getIndex(mx, my);
  a_star_->setGoal(index);

  nav_msgs::msg::Path plan;
  plan.header.stamp = node_->now();
  plan.header.frame_id = global_frame_;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;

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
  } catch (const std::exception & e) {
    error = std::string("invalid use: %s", e.what());
  }

  if (!error.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "%s: failed to create plan, %s.",
      name_.c_str(), error.c_str());
    return plan;
  }

  std::vector<DoubleCoordinates> grid_coords(path.size());

  for (int i = path.size() - 1; i >= 0; --i) {
    unsigned int index_x, index_y;
    double world_x, world_y;
    costmap_->indexToCells(path[i], index_x, index_y);
    costmap_->mapToWorld(index_x, index_y, world_x, world_y);
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
    grid_coords.push_back(DoubleCoordinates(world_x, world_y));
  }

  raw_plan_publisher_->publish(plan);
  plan.poses.clear();

  if (smoother_) {
    if (!smoother_->smooth(grid_coords, char_costmap)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
        name_.c_str());
    }
  }

  plan.poses.clear();
  std::vector<DoubleCoordinates>::const_iterator it;
  for (it = grid_coords.begin(); it != grid_coords.end(); ++it) {
    pose.pose.position.x = it->first;
    pose.pose.position.y = it->second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double> >(b-a);
  cout << "It took " << time_span.count() << " seconds with " << num_iterations << " iterations." <<  endl;
#endif

  return plan;
}

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner, nav2_core::GlobalPlanner)
