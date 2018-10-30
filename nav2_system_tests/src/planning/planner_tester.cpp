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
using nav2_tasks::TaskStatus;

namespace nav2_util
{

PlannerTester::PlannerTester()
: Node("PlannerTester"), map_publish_rate_(100s), map_set_(false), costmap_set_(false),
  using_fake_costmap_(true), costmap_server_running_(false), trinary_costmap_(true),
  track_unknown_space_(false), lethal_threshold_(100), unknown_cost_value_(-1),
  testCostmapType_(TestCostmap::open_space), spin_thread_(nullptr), spinning_ok_(false)
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::PlannerTester");

  // Our client used to invoke the services of the global planner (ComputePathToPose)

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  planner_client_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(temp_node);

  if (!planner_client_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "PlannerTester::PlannerTester: planner not running");
    throw std::runtime_error("PlannerTester::sendRequest: planner not running");
  }

  // For visualization, we'll publish the map and the path endpoints
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map");

  // We start with a 10x10 grid with no obstacles
  loadSimpleCostmap(TestCostmap::open_space);

  // TODO(orduno): get service name from param server
  startCostmapServer("GetCostmap");

  // Launch a thread to process the messages for this node
  spinning_ok_ = true;
  spin_thread_ = new std::thread(&PlannerTester::spinThread, this);
}

PlannerTester::~PlannerTester()
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::~PlannerTester");
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

void PlannerTester::loadMap(const std::string image_file_path, const std::string yaml_file_name)
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::loadMap");
  RCLCPP_INFO(
    this->get_logger(), "PlannerTester::loadMap: image_file: %s, yaml_file: %s",
    image_file_path.c_str(), yaml_file_name.c_str());

  // Specs for the default map
  // double resolution = 0.05;
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

  if (!image_file_path.empty()) {
    file_path = image_file_path;
  } else {
    // User can set an environment variable to the location of the test src
    char const * path = getenv("TEST_MAP");
    if (path == NULL) {
      file_path = "../../nav2_system_tests/maps/map.pgm";
    } else {
      file_path = std::string(path);
    }
  }

  RCLCPP_INFO(this->get_logger(), "PlannerTester::loadMap: file_path: %s", file_path.c_str());

  if (!yaml_file_name.empty()) {
    // TODO(orduno): parse yaml file
    RCLCPP_WARN(this->get_logger(), "PlannerTester::loadMap: yaml file parser not implemented"
      " yet, will use default values instead");
  }

  try {
    map_ =
      std::make_shared<nav_msgs::msg::OccupancyGrid>(
      map_loader::loadMapFromFile(
        file_path, resolution, negate,
        occupancy_threshold, free_threshold, origin, mode));
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
      "PlannerTester::loadDefaultMap: failed to load image from file: %s", file_path.c_str());
    throw;
  }

  map_->header.stamp = this->now();
  map_->header.frame_id = "map";
  map_->info.map_load_time = this->now();

  // TODO(orduno): replace with a latched topic
  map_timer_ = create_wall_timer(1s, std::bind(&PlannerTester::mapCallback, this));

  map_set_ = true;
  costmap_set_ = false;
  using_fake_costmap_ = false;

  setCostmap();
}

void PlannerTester::mapCallback()
{
  map_publisher_->publish(map_);
}

void PlannerTester::setCostmap()
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::setCostmap");

  if (!map_set_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PlannerTester::setCostmap: map has not been provided");

    // TODO(orduno) clarify how to handle errors here
    return;
  }

  // TODO(orduno): clarify if we should be passing the node as shared or raw pointer
  costmap_ = std::make_unique<Costmap>(
    this, trinary_costmap_, track_unknown_space_, lethal_threshold_, unknown_cost_value_);

  costmap_->setStaticMap(*map_);

  costmap_set_ = true;
  using_fake_costmap_ = false;
}

void PlannerTester::loadSimpleCostmap(const TestCostmap & testCostmapType)
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::loadSimpleCostmap");

  if (costmap_set_) {
    RCLCPP_WARN(
      this->get_logger(),
      "PlannerTester::loadSimpleCostmap: setting a new costmap with fake values");
  }

  costmap_ = std::make_unique<Costmap>(this);

  costmap_->setTestCostmap(testCostmapType);

  costmap_set_ = true;
  using_fake_costmap_ = true;
}

void PlannerTester::startCostmapServer(std::string serviceName)
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::startCostmapServer");

  if (!costmap_set_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PlannerTester::startCostmapServer:: you need to set a costmap before starting the service");
    return;
  }

  auto costmap_service_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> request,
    std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response) -> void
    {
      RCLCPP_INFO(this->get_logger(), "PlannerTester: Incoming costmap request");
      response->map = costmap_->getCostmap(request->specs);
    };

  // TODO(orduno): Enable parameter server and get costmap service name from there

  // Create a service that will use the callback function to handle requests.
  costmap_server_ = create_service<nav2_msgs::srv::GetCostmap>(
    serviceName, costmap_service_callback);

  costmap_server_running_ = true;
}

bool PlannerTester::defaultPlannerTest(
  nav2_tasks::ComputePathToPoseResult::SharedPtr & path,
  const double deviation_tolerance)
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::defaultPlannerTest");

  if (!costmap_set_) {
    RCLCPP_ERROR(this->get_logger(), "PlannerTester::defaultPlannerTest:"
      " you need to set the costmap before requesting a plan");
    return false;
  }

  auto endpoints = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  auto costmap_properties = costmap_->getProperties();

  // Compose the PathEndPoints message
  if (using_fake_costmap_) {
    RCLCPP_INFO(this->get_logger(), "PlannerTester::defaultPlannerTest:"
      " planning using a fake costmap");

    endpoints->start.position.x = 1.0;
    endpoints->start.position.y = 1.0;
    endpoints->goal.position.x = 9.0;
    endpoints->goal.position.y = 9.0;
    endpoints->tolerance = 2.0;

  } else {
    RCLCPP_INFO(this->get_logger(), "PlannerTester::defaultPlannerTest:"
      " planning using the provided map");

    // Defined with respect to world coordinate system
    //  Planner will do coordinate transformation to map internally
    endpoints->start.position.x = 390.0;
    endpoints->start.position.y = 10.0;
    endpoints->goal.position.x = 10.0;
    endpoints->goal.position.y = 390.0;
    endpoints->tolerance = 2.0;
  }

  bool pathIsCollisionFree = plannerTest(endpoints, path);

  // TODO(orduno): On a default test, provide the 'right answer' to compare with the planner result
  //               given that we know the start, end and costmap is either preloaded or coming from
  //               the provided map

  bool pathIsWithinTolerance = isWithinTolerance(*path, deviation_tolerance);

  return pathIsCollisionFree && pathIsWithinTolerance;
}

bool PlannerTester::defaultPlannerRandomTests(const unsigned int number_tests)
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::defaultPlannerRandomTests");

  if (!costmap_set_) {
    RCLCPP_ERROR(this->get_logger(), "PlannerTester::defaultPlannerRandomTests:"
      " you need to set the costmap before requesting a plan");
    return false;
  }

  // TODO(orduno): enable for costmaps
  if (using_fake_costmap_) {
    RCLCPP_ERROR(this->get_logger(), "PlannerTester::defaultPlannerRandomTests:"
      " randomized testing with hardcoded costmaps not implemented yet");
    return false;
  }

  // Initialize random number generator
  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::uniform_int_distribution<> distribution_x(0, costmap_->getProperties().size_x);
  std::uniform_int_distribution<> distribution_y(0, costmap_->getProperties().size_y);

  auto generate_random = [&]() mutable -> std::pair<int, int>{
      bool point_is_free = false;
      int x, y;
      while (!point_is_free) {
        x = distribution_x(generator);
        y = distribution_y(generator);
        point_is_free = costmap_->isFree(x, y);
      }
      return std::make_pair(x, y);
    };

  auto endpoints = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  bool all_tests_OK = true;
  unsigned int num_fail = 0;
  for (unsigned int test_num = 0; test_num < number_tests; ++test_num) {
    RCLCPP_INFO(this->get_logger(), "PlannerTester::defaultPlannerRandomTests:"
      " running test #%u", test_num + 1);

    // Compose the path endpoints using random numbers
    // Defined with respect to world coordinate system
    // Planner will do coordinate transformation to map internally
    auto start = generate_random();
    auto goal = generate_random();
    endpoints->start.position.x = start.first;
    endpoints->start.position.y = start.second;
    endpoints->goal.position.x = goal.first;
    endpoints->goal.position.y = goal.second;

    endpoints->tolerance = 2.0;

    // TODO(orduno): Tweak criteria for defining if a path goes into obstacles.
    //               Current Dijkstra planner will sometimes produce paths that cut corners
    //               i.e. some points are around the corner are actually inside the obstacle
    bool pathIsCollisionFree = plannerTest(endpoints, path);

    if (!pathIsCollisionFree) {
      RCLCPP_INFO(this->get_logger(), "PlannerTester::defaultPlannerRandomTests:"
        " failed or found a collision with start at %0.2f, %0.2f and end at %0.2f, %0.2f",
        endpoints->start.position.x, endpoints->start.position.y,
        endpoints->goal.position.x, endpoints->goal.position.y);
      all_tests_OK = false;
      ++num_fail;
    }
  }

  RCLCPP_INFO(this->get_logger(), "PlannerTester::defaultPlannerRandomTests:"
    " tested with %u endpoints. Planner failed on %u", number_tests, num_fail);

  return all_tests_OK;
}

bool PlannerTester::plannerTest(
  const nav2_tasks::ComputePathToPoseCommand::SharedPtr & endpoints,
  nav2_tasks::ComputePathToPoseResult::SharedPtr & path)
{
  RCLCPP_INFO(this->get_logger(), "PlannerTester::plannerTest:"
    " getting the path from the planner");

  TaskStatus status = sendRequest(endpoints, path);

  RCLCPP_INFO(this->get_logger(), "PlannerTester::plannerTest: status: %d", status);

  if (status == TaskStatus::FAILED) {
    return false;
  } else if (status == TaskStatus::SUCCEEDED) {
    // TODO(orduno): check why task may report success while planner fails in some cases
    //               in those cases, planner returns a path of 0 points
    RCLCPP_INFO(this->get_logger(), "PlannerTester::plannerTest:"
      " got path, checking for collisions");

    return isCollisionFree(*path);
  }

  return false;
}

TaskStatus PlannerTester::sendRequest(
  const nav2_tasks::ComputePathToPoseCommand::SharedPtr & endpoints,
  nav2_tasks::ComputePathToPoseResult::SharedPtr & path)
{
  planner_client_->sendCommand(endpoints);

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
  // TODO(orduno): for now we are assuming the robot is the size of a single cell
  //               costmap/world_model has consider the robot footprint

  bool collisionFree = true;

  for (auto pose : path.poses) {
    collisionFree = costmap_->isFree(
      static_cast<unsigned int>(pose.position.x), static_cast<unsigned int>(pose.position.y));

    if (!collisionFree) {
      RCLCPP_INFO(this->get_logger(), "PathTester::isCollisionFree: path has collisions :(");
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "PathTester::isCollisionFree: path has no collisions :)");
  return true;
}

bool PlannerTester::isWithinTolerance(
  const nav2_tasks::ComputePathToPoseResult & /*path*/,
  const double /*deviationTolerance*/)
{
  // TODO(orduno)
  return true;
}

bool PlannerTester::sendCancel()
{
  RCLCPP_ERROR(this->get_logger(), "PlannerTester::sendCancel:"
    " function not implemented yet");

  // TODO(orduno)
  return false;
}

}  // namespace nav2_util
