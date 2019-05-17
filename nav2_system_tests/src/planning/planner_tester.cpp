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
#include <memory>
#include <chrono>

#include "planner_tester.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/map_loader/map_loader.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"

using namespace std::chrono_literals;
using nav2_util::Costmap;
using nav2_util::TestCostmap;

namespace nav2_system_tests
{

PlannerTester::PlannerTester()
: Node("PlannerTester"), map_publish_rate_(100s), map_set_(false), costmap_set_(false),
  using_fake_costmap_(true), costmap_server_running_(false), trinary_costmap_(true),
  track_unknown_space_(false), lethal_threshold_(100), unknown_cost_value_(-1),
  testCostmapType_(TestCostmap::open_space), spin_thread_(nullptr), spinning_ok_(false)
{
  // The client used to invoke the services of the global planner (ComputePathToPose)
  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
  planner_client_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(temp_node);

  if (!planner_client_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Planner not running");
    throw std::runtime_error("Planner not running");
  }

  // Publisher of the faked current robot pose
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose");

  // For visualization, we'll publish the map and the path endpoints
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map");

  // We start with a 10x10 grid with no obstacles
  loadSimpleCostmap(TestCostmap::open_space);

  startCostmapServer("GetCostmap");

  // Launch a thread to process the messages for this node
  spinning_ok_ = true;
  spin_thread_ = new std::thread(&PlannerTester::spinThread, this);
}

PlannerTester::~PlannerTester()
{
  spinning_ok_ = false;
  spin_thread_->join();
  delete spin_thread_;
  spin_thread_ = nullptr;
}

void PlannerTester::spinThread()
{
  while (spinning_ok_) {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
  geometry_msgs::msg::Twist origin;
  origin.linear.x = 0.0;
  origin.linear.y = 0.0;
  origin.linear.z = 0.0;
  origin.angular.x = 0.0;
  origin.angular.y = 0.0;
  origin.angular.z = 0.0;

  MapMode mode = TRINARY;

  std::string file_path = "";
  char const * path = getenv("TEST_MAP");
  if (path == NULL) {
    throw std::runtime_error("Path to map image file"
            " has not been specified in environment variable `TEST_MAP`.");
  } else {
    file_path = std::string(path);
  }

  RCLCPP_INFO(this->get_logger(), "Loading map with file_path: %s", file_path.c_str());

  try {
    map_ =
      std::make_shared<nav_msgs::msg::OccupancyGrid>(
      map_loader::loadMapFromFile(
        file_path, resolution, negate,
        occupancy_threshold, free_threshold, origin, mode));
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to load image from file: %s", file_path.c_str());
    throw;
  }

  map_->header.stamp = this->now();
  map_->header.frame_id = "map";
  map_->info.map_load_time = this->now();

  // TODO(orduno): #443 replace with a latched topic
  map_timer_ = create_wall_timer(1s, [this]() -> void {map_pub_->publish(map_);});

  map_set_ = true;
  costmap_set_ = false;
  using_fake_costmap_ = false;

  setCostmap();
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

void PlannerTester::loadSimpleCostmap(const TestCostmap & testCostmapType)
{
  if (costmap_set_) {
    RCLCPP_DEBUG(this->get_logger(), "Setting a new costmap with fake values");
  }

  costmap_ = std::make_unique<Costmap>(this);

  costmap_->set_test_costmap(testCostmapType);

  costmap_set_ = true;
  using_fake_costmap_ = true;
}

void PlannerTester::startCostmapServer(std::string serviceName)
{
  if (!costmap_set_) {
    RCLCPP_ERROR(this->get_logger(), "Costmap must be set before starting the service");
    return;
  }

  auto costmap_service_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> request,
    std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response) -> void
    {
      RCLCPP_DEBUG(this->get_logger(), "Incoming costmap request");
      response->map = costmap_->getCostmap(request->specs);
    };

  // Create a service that will use the callback function to handle requests.
  costmap_server_ = create_service<nav2_msgs::srv::GetCostmap>(
    serviceName, costmap_service_callback);

  costmap_server_running_ = true;
}

bool PlannerTester::defaultPlannerTest(
  nav2_tasks::ComputePathToPoseResult::SharedPtr & path,
  const double /*deviation_tolerance*/)
{
  if (!costmap_set_) {
    RCLCPP_ERROR(this->get_logger(), "Costmap must be set before requesting a plan");
    return false;
  }

  // TODO(orduno) #443 Add support for planners that take into account robot orientation
  geometry_msgs::msg::Point robot_position;
  auto goal = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  auto costmap_properties = costmap_->get_properties();

  if (using_fake_costmap_) {
    RCLCPP_DEBUG(this->get_logger(), "Planning using a fake costmap");

    robot_position.x = 1.0;
    robot_position.y = 1.0;

    goal->pose.position.x = 8.0;
    goal->pose.position.y = 8.0;

  } else {
    RCLCPP_DEBUG(this->get_logger(), "Planning using the provided map");

    // Defined with respect to world coordinate system
    //  Planner will do coordinate transformation to map internally
    robot_position.x = 390.0;
    robot_position.y = 10.0;

    goal->pose.position.x = 10.0;
    goal->pose.position.y = 390.0;
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
    RCLCPP_ERROR(this->get_logger(),
      "Randomized testing with hardcoded costmaps not implemented yet");
    return false;
  }

  // Initialize random number generator
  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::uniform_int_distribution<> distribution_x(0, costmap_->get_properties().size_x);
  std::uniform_int_distribution<> distribution_y(0, costmap_->get_properties().size_y);

  auto generate_random = [&]() mutable -> std::pair<int, int> {
      bool point_is_free = false;
      int x, y;
      while (!point_is_free) {
        x = distribution_x(generator);
        y = distribution_y(generator);
        point_is_free = costmap_->isFree(x, y);
      }
      return std::make_pair(x, y);
    };

  // TODO(orduno) #443 Add support for planners that take into account robot orientation
  geometry_msgs::msg::Point robot_position;
  auto goal = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  unsigned int num_fail = 0;
  for (unsigned int test_num = 0; test_num < number_tests; ++test_num) {
    RCLCPP_INFO(this->get_logger(), "Running test #%u", test_num + 1);

    // Compose the robot start position and goal using random numbers
    // Defined with respect to world coordinate system
    // Planner will do coordinate transformation to map internally

    auto vals = generate_random();
    robot_position.x = vals.first;
    robot_position.y = vals.second;

    vals = generate_random();
    goal->pose.position.x = vals.first;
    goal->pose.position.y = vals.second;

    if (!plannerTest(robot_position, goal, path)) {
      RCLCPP_WARN(this->get_logger(), "Failed with start at %0.2f, %0.2f and goal at %0.2f, %0.2f",
        robot_position.x, robot_position.y, goal->pose.position.x, goal->pose.position.y);
      ++num_fail;
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "Tested with %u endpoints. Planner failed on %u", number_tests, num_fail);

  if ((num_fail / number_tests) > acceptable_fail_ratio) {
    return false;
  }

  return true;
}

bool PlannerTester::plannerTest(
  const geometry_msgs::msg::Point & robot_position,
  const nav2_tasks::ComputePathToPoseCommand::SharedPtr & goal,
  nav2_tasks::ComputePathToPoseResult::SharedPtr & path)
{
  RCLCPP_DEBUG(this->get_logger(), "Getting the path from the planner");

  // First make available the current robot position for the planner to take as starting point
  publishRobotPosition(robot_position);

  // Then request to compute a path
  TaskStatus status = sendRequest(goal, path);

  RCLCPP_DEBUG(this->get_logger(), "Path request status: %d", status);

  if (status == TaskStatus::FAILED) {
    return false;
  } else if (status == TaskStatus::SUCCEEDED) {
    // TODO(orduno): #443 check why task may report success while planner returns a path of 0 points
    RCLCPP_DEBUG(this->get_logger(), "Got path, checking endpoints and possible collisions");

    return isCollisionFree(*path) && isWithinTolerance(robot_position, *goal, *path);
  }

  return false;
}

void PlannerTester::publishRobotPosition(const geometry_msgs::msg::Point & position) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped p;
  p.header.frame_id = "map";
  p.header.stamp = rclcpp::Time();
  p.pose.pose.position = position;
  p.pose.pose.orientation.x = 0.0;
  p.pose.pose.orientation.y = 0.0;
  p.pose.pose.orientation.z = 0.0;
  p.pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 12; i++) {
    p.pose.covariance[i] = 0.0;
  }

  pose_pub_->publish(p);
}

TaskStatus PlannerTester::sendRequest(
  const nav2_tasks::ComputePathToPoseCommand::SharedPtr & goal,
  nav2_tasks::ComputePathToPoseResult::SharedPtr & path)
{
  planner_client_->sendCommand(goal);

  // Loop until the subtask is completed
  while (true) {
    TaskStatus status = planner_client_->waitForResult(path, std::chrono::milliseconds(100));

    if (status != TaskStatus::RUNNING) {
      return status;
    }
  }
}

bool PlannerTester::isCollisionFree(const nav2_tasks::ComputePathToPoseResult & path)
{
  // At each point of the path, check if the corresponding cell is free

  // TODO(orduno): #443 for now we are assuming the robot is the size of a single cell
  //               costmap/world_model has consider the robot footprint

  // TODO(orduno): #443 Tweak criteria for defining if a path goes into obstacles.
  //               Current navfn planner will sometimes produce paths that cut corners
  //               i.e. some points are around the corner are actually inside the obstacle

  bool collisionFree = true;

  for (auto pose : path.poses) {
    collisionFree = costmap_->isFree(
      static_cast<unsigned int>(std::round(pose.position.x)),
      static_cast<unsigned int>(std::round(pose.position.y)));

    if (!collisionFree) {
      RCLCPP_WARN(this->get_logger(), "Path has collision at (%.2f, %.2f)",
        pose.position.x, pose.position.y);
      printPath(path);
      return false;
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Path has no collisions");
  return true;
}

bool PlannerTester::isWithinTolerance(
  const geometry_msgs::msg::Point & robot_position,
  const nav2_tasks::ComputePathToPoseCommand & goal,
  const nav2_tasks::ComputePathToPoseResult & path) const
{
  return isWithinTolerance(
    robot_position, goal, path, 0.0, nav2_tasks::ComputePathToPoseResult());
}

bool PlannerTester::isWithinTolerance(
  const geometry_msgs::msg::Point & robot_position,
  const nav2_tasks::ComputePathToPoseCommand & goal,
  const nav2_tasks::ComputePathToPoseResult & path,
  const double /*deviationTolerance*/,
  const nav2_tasks::ComputePathToPoseResult & /*reference_path*/) const
{
  // TODO(orduno) #443 Work in progress, for now we only check that the path start matches the
  //              robot start location and that the path end matches the goal.

  auto path_start = path.poses[0];
  auto path_end = path.poses.end()[-1];

  if (
    path_start.position.x == robot_position.x &&
    path_start.position.y == robot_position.y &&
    path_end.position.x == goal.pose.position.x &&
    path_end.position.y == goal.pose.position.y)
  {
    RCLCPP_DEBUG(this->get_logger(), "Path endpoints have correct start and end points");

    return true;
  }
  RCLCPP_WARN(this->get_logger(), "Path endpoints deviate from requested start and end points");

  RCLCPP_DEBUG(this->get_logger(), "Requested path starts at (%.2f, %.2f) and ends at (%.2f, %.2f)",
    robot_position.x, robot_position.y, goal.pose.position.x, goal.pose.position.y);

  RCLCPP_DEBUG(this->get_logger(), "Computed path starts at (%.2f, %.2f) and ends at (%.2f, %.2f)",
    path_start.position.x, path_start.position.y, path_end.position.x, path_end.position.y);

  return false;
}

bool PlannerTester::sendCancel()
{
  RCLCPP_ERROR(this->get_logger(), "Function not implemented yet");

  // TODO(orduno) #443
  return false;
}

void PlannerTester::printPath(const nav2_tasks::ComputePathToPoseResult & path) const
{
  int index = 0;
  for (auto pose : path.poses) {
    RCLCPP_INFO(get_logger(), "  point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    ++index;
  }
}

}  // namespace nav2_system_tests
