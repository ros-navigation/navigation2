// Copyright (c) 2018 Intel Corporation
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
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "planner_tester.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::Costmap;
using nav2_util::TestCostmap;

namespace nav2_system_tests
{

PlannerTester::PlannerTester()
: Node("PlannerTester"), is_active_(false),
  map_set_(false), costmap_set_(false),
  using_fake_costmap_(true), trinary_costmap_(true),
  track_unknown_space_(false), lethal_threshold_(100), unknown_cost_value_(-1),
  testCostmapType_(TestCostmap::open_space), base_transform_(nullptr),
  map_publish_rate_(100s)
{
}

void PlannerTester::activate()
{
  if (is_active_) {
    throw std::runtime_error("Trying to activate while already active");
    return;
  }
  is_active_ = true;

  // Launch a thread to process the messages for this node
  spin_thread_ = std::make_unique<nav2_util::NodeThread>(this);

  // We start with a 10x10 grid with no obstacles
  costmap_ = std::make_unique<Costmap>(this);
  loadSimpleCostmap(TestCostmap::open_space);

  startRobotTransform();

  // The navfn wrapper
  auto state = rclcpp_lifecycle::State();
  planner_tester_ = std::make_shared<NavFnPlannerTester>();
  planner_tester_->declare_parameter(
    "GridBased.use_astar", rclcpp::ParameterValue(true));
  planner_tester_->set_parameter(
    rclcpp::Parameter(std::string("GridBased.use_astar"), rclcpp::ParameterValue(true)));
  planner_tester_->set_parameter(
    rclcpp::Parameter(std::string("expected_planner_frequency"), rclcpp::ParameterValue(-1.0)));
  planner_tester_->onConfigure(state);
  publishRobotTransform();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  rclcpp::Rate r(1);
  r.sleep();
  planner_tester_->onActivate(state);
}

void PlannerTester::deactivate()
{
  if (!is_active_) {
    throw std::runtime_error("Trying to deactivate while already inactive");
    return;
  }
  is_active_ = false;

  spin_thread_.reset();

  auto state = rclcpp_lifecycle::State();
  planner_tester_->onDeactivate(state);
  planner_tester_->onCleanup(state);

  map_timer_.reset();
  map_pub_.reset();
  map_.reset();
  tf_broadcaster_.reset();
}

PlannerTester::~PlannerTester()
{
  if (is_active_) {
    deactivate();
  }
}

void PlannerTester::startRobotTransform()
{
  // Provide the robot pose transform
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Set an initial pose
  geometry_msgs::msg::Point robot_position;
  robot_position.x = 1.0;
  robot_position.y = 1.0;
  updateRobotPosition(robot_position);

  // Publish the transform periodically
  transform_timer_ = create_wall_timer(
    100ms, std::bind(&PlannerTester::publishRobotTransform, this));
}

void PlannerTester::updateRobotPosition(const geometry_msgs::msg::Point & position)
{
  if (!base_transform_) {
    base_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    base_transform_->header.frame_id = "map";
    base_transform_->child_frame_id = "base_link";
  }
  std::cout << now().nanoseconds() << std::endl;

  base_transform_->header.stamp = now() + rclcpp::Duration(0.25s);
  base_transform_->transform.translation.x = position.x;
  base_transform_->transform.translation.y = position.y;
  base_transform_->transform.rotation.w = 1.0;

  publishRobotTransform();
}

void PlannerTester::publishRobotTransform()
{
  if (base_transform_) {
    tf_broadcaster_->sendTransform(*base_transform_);
  }
}

void PlannerTester::loadDefaultMap()
{
  // Specs for the default map
  double resolution = 1.0;
  bool negate = false;
  double occupancy_threshold = 0.65;
  double free_threshold = 0.196;

  // Define origin offset
  std::vector<double> origin = {0.0, 0.0, 0.0};

  nav2_map_server::MapMode mode = nav2_map_server::MapMode::Trinary;

  std::string file_path = "";
  char const * path = getenv("TEST_MAP");
  if (path == NULL) {
    throw std::runtime_error(
            "Path to map image file"
            " has not been specified in environment variable `TEST_MAP`.");
  } else {
    file_path = std::string(path);
  }

  RCLCPP_INFO(this->get_logger(), "Loading map with file_path: %s", file_path.c_str());

  try {
    map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    nav2_map_server::LoadParameters load_parameters;
    load_parameters.image_file_name = file_path;
    load_parameters.resolution = resolution;
    load_parameters.origin = origin;
    load_parameters.free_thresh = free_threshold;
    load_parameters.occupied_thresh = occupancy_threshold;
    load_parameters.mode = mode;
    load_parameters.negate = negate;
    loadMapFromFile(load_parameters, *map_);
  } catch (...) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load image from file: %s", file_path.c_str());
    throw;
  }

  map_->header.stamp = this->now();
  map_->header.frame_id = "map";
  map_->info.map_load_time = this->now();

  // TODO(orduno): #443 replace with a latched topic
  map_timer_ = create_wall_timer(1s, [this]() -> void {map_pub_->publish(*map_);});

  map_set_ = true;
  costmap_set_ = false;
  using_fake_costmap_ = false;

  setCostmap();
}

void PlannerTester::loadSimpleCostmap(const TestCostmap & testCostmapType)
{
  RCLCPP_INFO(get_logger(), "loadSimpleCostmap called.");
  if (costmap_set_) {
    RCLCPP_DEBUG(this->get_logger(), "Setting a new costmap with fake values");
  }

  costmap_->set_test_costmap(testCostmapType);

  costmap_set_ = true;
  using_fake_costmap_ = true;
}

void PlannerTester::setCostmap()
{
  if (!map_set_) {
    RCLCPP_ERROR(this->get_logger(), "Map has not been provided");
    return;
  }

  costmap_ = std::make_unique<Costmap>(
    this, trinary_costmap_, track_unknown_space_, lethal_threshold_, unknown_cost_value_);

  costmap_->set_static_map(*map_);

  costmap_set_ = true;
  using_fake_costmap_ = false;
}

bool PlannerTester::defaultPlannerTest(
  ComputePathToPoseResult & path,
  const double /*deviation_tolerance*/)
{
  if (!costmap_set_) {
    RCLCPP_ERROR(this->get_logger(), "Costmap must be set before requesting a plan");
    return false;
  }

  // TODO(orduno) #443 Add support for planners that take into account robot orientation
  geometry_msgs::msg::Point robot_position;
  ComputePathToPoseCommand goal;
  auto costmap_properties = costmap_->get_properties();

  if (using_fake_costmap_) {
    RCLCPP_DEBUG(this->get_logger(), "Planning using a fake costmap");

    robot_position.x = 1.0;
    robot_position.y = 1.0;

    goal.pose.position.x = 8.0;
    goal.pose.position.y = 8.0;

  } else {
    RCLCPP_DEBUG(this->get_logger(), "Planning using the provided map");

    // Defined with respect to world coordinate system
    //  Planner will do coordinate transformation to map internally
    robot_position.x = 390.0;
    robot_position.y = 10.0;

    goal.pose.position.x = 10.0;
    goal.pose.position.y = 390.0;
  }

  // TODO(orduno): #443 On a default test, provide the reference path to compare with the planner
  //               result.
  return plannerTest(robot_position, goal, path);
}

bool PlannerTester::defaultPlannerRandomTests(
  const unsigned int number_tests,
  const float acceptable_fail_ratio = 0.1)
{
  if (!costmap_set_) {
    RCLCPP_ERROR(this->get_logger(), "Costmap must be set before requesting a plan");
    return false;
  }

  if (using_fake_costmap_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Randomized testing with hardcoded costmaps not implemented yet");
    return false;
  }

  // Initialize random number generator
  std::random_device random_device;
  std::mt19937 generator(random_device());

  // Obtain random positions within map
  std::uniform_int_distribution<> distribution_x(1, costmap_->get_properties().size_x - 1);
  std::uniform_int_distribution<> distribution_y(1, costmap_->get_properties().size_y - 1);

  auto generate_random = [&]() mutable -> std::pair<int, int> {
      bool point_is_free = false;
      int x, y;
      while (!point_is_free) {
        x = distribution_x(generator);
        y = distribution_y(generator);
        point_is_free = costmap_->is_free(x, y);
      }
      return std::make_pair(x, y);
    };

  // TODO(orduno) #443 Add support for planners that take into account robot orientation
  geometry_msgs::msg::Point robot_position;
  ComputePathToPoseCommand goal;
  ComputePathToPoseResult path;

  unsigned int num_fail = 0;
  auto start = high_resolution_clock::now();
  for (unsigned int test_num = 0; test_num < number_tests; ++test_num) {
    RCLCPP_DEBUG(this->get_logger(), "Running test #%u", test_num + 1);

    // Compose the robot start position and goal using random numbers
    // Defined with respect to world coordinate system
    // Planner will do coordinate transformation to map internally

    auto vals = generate_random();
    robot_position.x = vals.first;
    robot_position.y = vals.second;

    vals = generate_random();
    goal.pose.position.x = vals.first;
    goal.pose.position.y = vals.second;

    if (!plannerTest(robot_position, goal, path)) {
      RCLCPP_WARN(
        this->get_logger(), "Failed with start at %0.2f, %0.2f and goal at %0.2f, %0.2f",
        robot_position.x, robot_position.y, goal.pose.position.x, goal.pose.position.y);
      ++num_fail;
    }
  }
  auto end = high_resolution_clock::now();
  auto elapsed = duration_cast<milliseconds>(end - start);

  RCLCPP_INFO(
    this->get_logger(),
    "Tested with %u tests. Planner failed on %u. Test time %ld ms",
    number_tests, num_fail, elapsed.count());

  if ((num_fail / number_tests) > acceptable_fail_ratio) {
    return false;
  }

  return true;
}

bool PlannerTester::plannerTest(
  const geometry_msgs::msg::Point & robot_position,
  const ComputePathToPoseCommand & goal,
  ComputePathToPoseResult & path)
{
  RCLCPP_DEBUG(this->get_logger(), "Getting the path from the planner");

  // First make available the current robot position for the planner to take as starting point
  updateRobotPosition(robot_position);
  sleep(0.05);

  // Then request to compute a path
  TaskStatus status = createPlan(goal, path);

  RCLCPP_DEBUG(this->get_logger(), "Path request status: %d", static_cast<int8_t>(status));

  if (status == TaskStatus::FAILED) {
    return false;
  } else if (status == TaskStatus::SUCCEEDED) {
    // TODO(orduno): #443 check why task may report success while planner returns a path of 0 points
    RCLCPP_DEBUG(this->get_logger(), "Got path, checking for possible collisions");
    return isCollisionFree(path) && isWithinTolerance(robot_position, goal, path);
  }

  return false;
}

TaskStatus PlannerTester::createPlan(
  const ComputePathToPoseCommand & goal,
  ComputePathToPoseResult & path)
{
  // Update the costmap of the planner to the set data
  planner_tester_->setCostmap(costmap_.get());

  // Call planning algorithm
  if (planner_tester_->createPath(goal, path)) {
    return TaskStatus::SUCCEEDED;
  }

  return TaskStatus::FAILED;
}

bool PlannerTester::isCollisionFree(const ComputePathToPoseResult & path)
{
  // At each point of the path, check if the corresponding cell is free

  // TODO(orduno): #443 for now we are assuming the robot is the size of a single cell
  //               costmap/world_model has consider the robot footprint

  // TODO(orduno): #443 Tweak criteria for defining if a path goes into obstacles.
  //               Current navfn planner will sometimes produce paths that cut corners
  //               i.e. some points are around the corner are actually inside the obstacle

  bool collisionFree = true;

  for (auto pose : path.poses) {
    collisionFree = costmap_->is_free(
      static_cast<unsigned int>(std::round(pose.pose.position.x)),
      static_cast<unsigned int>(std::round(pose.pose.position.y)));

    if (!collisionFree) {
      RCLCPP_WARN(
        this->get_logger(), "Path has collision at (%.2f, %.2f)",
        pose.pose.position.x, pose.pose.position.y);
      printPath(path);
      return false;
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Path has no collisions");
  return true;
}

bool PlannerTester::isWithinTolerance(
  const geometry_msgs::msg::Point & robot_position,
  const ComputePathToPoseCommand & goal,
  const ComputePathToPoseResult & path) const
{
  return isWithinTolerance(
    robot_position, goal, path, 0.0, ComputePathToPoseResult());
}

bool PlannerTester::isWithinTolerance(
  const geometry_msgs::msg::Point & robot_position,
  const ComputePathToPoseCommand & goal,
  const ComputePathToPoseResult & path,
  const double /*deviationTolerance*/,
  const ComputePathToPoseResult & /*reference_path*/) const
{
  // TODO(orduno) #443 Work in progress, for now we only check that the path start matches the
  //              robot start location and that the path end matches the goal.

  auto path_start = path.poses[0];
  auto path_end = path.poses.end()[-1];

  if (
    path_start.pose.position.x == robot_position.x &&
    path_start.pose.position.y == robot_position.y &&
    path_end.pose.position.x == goal.pose.position.x &&
    path_end.pose.position.y == goal.pose.position.y)
  {
    RCLCPP_DEBUG(this->get_logger(), "Path has correct start and end points");

    return true;
  }
  RCLCPP_WARN(this->get_logger(), "Path deviates from requested start and end points");

  RCLCPP_DEBUG(
    this->get_logger(), "Requested path starts at (%.2f, %.2f) and ends at (%.2f, %.2f)",
    robot_position.x, robot_position.y, goal.pose.position.x, goal.pose.position.y);

  RCLCPP_DEBUG(
    this->get_logger(), "Computed path starts at (%.2f, %.2f) and ends at (%.2f, %.2f)",
    path_start.pose.position.x, path_start.pose.position.y,
    path_end.pose.position.x, path_end.pose.position.y);

  return false;
}

void PlannerTester::printPath(const ComputePathToPoseResult & path) const
{
  auto index = 0;
  auto ss = std::stringstream{};

  for (auto pose : path.poses) {
    ss << "   point #" << index << " with" <<
      " x: " << std::setprecision(3) << pose.pose.position.x <<
      " y: " << std::setprecision(3) << pose.pose.position.y << '\n';
    ++index;
  }

  RCLCPP_INFO(get_logger(), ss.str().c_str());
}

}  // namespace nav2_system_tests
