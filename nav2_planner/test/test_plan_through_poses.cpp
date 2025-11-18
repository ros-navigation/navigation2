// Copyright (c) 2025 Abhishekh Reddy
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

#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_planner/planner_server.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

// Some parameters for planner server
static const double EXPECTED_PLANNER_FREQ{20.0};
static const std::vector<std::string> PLANNER_PLUGINS{"GridBased"};
static const double COSTMAP_UPDATE_TIMEOUT{1.0};
static const char PLANNER_PLUGIN_NAME[]{"nav2_smac_planner::SmacPlanner2D"};
static const double PLANNER_TOLERANCE{0.1};

// Some parameters for costmap
static const int COSTMAP_HEIGHT_METERS{1};
static const int COSTMAP_WIDTH_METERS{1};
static const double COSTMAP_RESOLUTION{0.1};
static const std::vector<std::string> COSTMAP_PLUGINS{"static_layer"};
static const char COSTMAP_LAYER_NAME[]{"nav2_costmap_2d::StaticLayer"};

class PlannerServerWrapper : public nav2_planner::PlannerServer
{
public:
  PlannerServerWrapper()
  : nav2_planner::PlannerServer(rclcpp::NodeOptions())
  {
  }

  void start()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_activate(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  void stop()
  {
    ASSERT_EQ(on_deactivate(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_cleanup(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_shutdown(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  nav2_costmap_2d::Costmap2DROS::SharedPtr getCostmapROS()
  {
    return costmap_ros_;
  }

  nav2_costmap_2d::Costmap2D * getCostmap()
  {
    return costmap_ros_->getCostmap();
  }
};

class Tester : public ::testing::Test
{
public:
  using Action = nav2_msgs::action::ComputePathThroughPoses;
  using ActionClient = rclcpp_action::Client<Action>;
  using ActionGoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using ActionGHFuture = std::shared_future<ActionGoalHandle::SharedPtr>;

  Tester();
  ~Tester();

  void setParameters();
  void startTesterNode();
  void stopTesterNode();

  void setGoalFromPoses(
    const std::vector<geometry_msgs::msg::Pose> & poses, Action::Goal & goal_out);

  void waitForResult(
    ActionGHFuture & gh_future, Tester::ActionGoalHandle::WrappedResult & result_out,
    const std::chrono::milliseconds & timeout);

  void addObstacleCells(unsigned int x, unsigned int y, unsigned int width, unsigned int height);
  void addObstacle(double x, double y, double width, double height);

protected:
  void broadcastRobotPose();
  void publishMap();

  std::shared_ptr<PlannerServerWrapper> planner_;
  std::shared_ptr<nav2::LifecycleNode> tester_node_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<nav2::NodeThread> executor_thread_;

  nav_msgs::msg::OccupancyGrid map_;
  nav2::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> robot_pose_broadcaster_;
  rclcpp::TimerBase::SharedPtr broadcast_timer_;

  ActionClient::SharedPtr action_client_;
};

Tester::Tester()
{
  using namespace std::chrono_literals;

  planner_ = std::make_shared<PlannerServerWrapper>();
  tester_node_ = std::make_shared<nav2::LifecycleNode>("tester_node");

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(planner_->get_node_base_interface());
  executor_->add_node(tester_node_->get_node_base_interface());
  executor_thread_ = std::make_unique<nav2::NodeThread>(executor_);

  robot_pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(tester_node_);
  action_client_ = tester_node_->create_action_client<Action>("compute_path_through_poses");

  map_publisher_ = tester_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", nav2::qos::LatchedPublisherQoS());

  broadcast_timer_ =
    tester_node_->create_wall_timer(200ms, std::bind(&Tester::broadcastRobotPose, this));
}

Tester::~Tester()
{
  action_client_.reset();
  broadcast_timer_.reset();
  map_publisher_.reset();
  robot_pose_broadcaster_.reset();
  executor_thread_.reset();
  executor_.reset();
  tester_node_.reset();
  planner_.reset();
}

void Tester::setParameters()
{
  planner_->set_parameter(rclcpp::Parameter("expected_planner_frequency", EXPECTED_PLANNER_FREQ));
  planner_->set_parameter(rclcpp::Parameter("planner_plugins", PLANNER_PLUGINS));
  planner_->set_parameter(rclcpp::Parameter("costmap_update_timeout", COSTMAP_UPDATE_TIMEOUT));
  planner_->set_parameter(rclcpp::Parameter("allow_partial_planning", true));

  planner_->set_parameter(
    rclcpp::Parameter(PLANNER_PLUGINS.front() + ".plugin", PLANNER_PLUGIN_NAME));

  planner_->declare_parameter(
    PLANNER_PLUGINS.front() + ".tolerance", rclcpp::ParameterValue(PLANNER_TOLERANCE));

  auto costmap = planner_->getCostmapROS();
  costmap->set_parameter(rclcpp::Parameter("height", COSTMAP_HEIGHT_METERS));
  costmap->set_parameter(rclcpp::Parameter("width", COSTMAP_WIDTH_METERS));
  costmap->set_parameter(rclcpp::Parameter("resolution", COSTMAP_RESOLUTION));
  costmap->set_parameter(rclcpp::Parameter("plugins", COSTMAP_PLUGINS));

  costmap->declare_parameter("static_layer.plugin", rclcpp::ParameterValue(COSTMAP_LAYER_NAME));
  costmap->declare_parameter("static_layer.lethal_cost_threshold", rclcpp::ParameterValue(100));
  costmap->declare_parameter(
    "static_layer.map_subscribe_transient_local", rclcpp::ParameterValue(true));

  map_.info.height = COSTMAP_HEIGHT_METERS * (1 / COSTMAP_RESOLUTION);
  map_.info.width = COSTMAP_WIDTH_METERS * (1 / COSTMAP_RESOLUTION);
  map_.info.resolution = COSTMAP_RESOLUTION;
  map_.header.frame_id = "map";

  map_.data.resize(map_.info.width * map_.info.height);
  std::fill(map_.data.begin(), map_.data.end(), 0);

  // planner_->get_logger().set_level(rclcpp::Logger::Level::Debug);
  // costmap->get_logger().set_level(rclcpp::Logger::Level::Debug);
  // tester_node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
}

void Tester::startTesterNode()
{
  tester_node_->activate();
  map_publisher_->on_activate();
}

void Tester::stopTesterNode()
{
  map_publisher_->on_deactivate();

  tester_node_->deactivate();
  tester_node_->cleanup();
  tester_node_->shutdown();
}

void Tester::setGoalFromPoses(
  const std::vector<geometry_msgs::msg::Pose> & poses, Action::Goal & goal_out)
{
  // First pose in list is the starting point and the rest are goals.
  goal_out.start.header.frame_id = "map";
  goal_out.start.pose = poses.front();

  for (auto pose_it = poses.begin() + 1; pose_it != poses.end(); pose_it++) {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "map";
    stamped_pose.pose = *pose_it;

    goal_out.goals.goals.push_back(stamped_pose);
  }

  goal_out.goals.header.frame_id = "map";
  goal_out.planner_id = PLANNER_PLUGINS.front();
  goal_out.use_start = true;
}

void Tester::waitForResult(
  ActionGHFuture & gh_future, Tester::ActionGoalHandle::WrappedResult & result_out,
  const std::chrono::milliseconds & timeout)
{
  // Check if the goal has been accepted
  ASSERT_TRUE(gh_future.get());

  // Get result
  auto result_future = action_client_->async_get_result(gh_future.get());
  {
    // Wait for the result
    auto status = result_future.wait_for(timeout);
    ASSERT_EQ(status, std::future_status::ready);
  }

  result_out = result_future.get();
}

void Tester::addObstacleCells(
  unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
  const unsigned int size_x = map_.info.width;
  const unsigned int size_y = map_.info.height;

  RCLCPP_DEBUG_STREAM(tester_node_->get_logger(), "Adding obstacle at: "
    << x << ", " << y << ", " << "with size " << width << ", " << height);

  ASSERT_FALSE(x >= size_x || y >= size_y);
  ASSERT_FALSE(width > size_x || height > size_y);

  unsigned int min_x, max_x, min_y, max_y = 0;
  {
    const auto x_int = static_cast<int>(x);
    const auto y_int = static_cast<int>(y);
    const auto width_int = static_cast<int>(width);
    const auto height_int = static_cast<int>(height);
    const auto size_x_int = static_cast<int>(size_x);
    const auto size_y_int = static_cast<int>(size_y);

    min_x = std::max(0, x_int - (width_int / 2));
    max_x = std::min(size_x_int - 1, x_int + (width_int / 2));
    min_y = std::max(0, y_int - (height_int / 2));
    max_y = std::min(size_y_int - 1, y_int + (height_int / 2));
  }

  RCLCPP_DEBUG_STREAM(tester_node_->get_logger(), "Bounding values for reference:"
    << min_x << ", " << max_x << ", " << min_y << ", " << max_y);

  for (unsigned int my = min_y; my <= max_y; ++my) {
    for (unsigned int mx = min_x; mx <= max_x; ++mx) {
      const unsigned int index = (my * size_x) + mx;

      map_.data.at(index) = 100;
    }
  }
}

void Tester::addObstacle(double x, double y, double width, double height)
{
  unsigned int x_cells, y_cells;
  bool point_valid = planner_->getCostmap()->worldToMap(x, y, x_cells, y_cells);
  ASSERT_TRUE(point_valid);

  const unsigned int width_cells = width * (1 / planner_->getCostmap()->getResolution());
  const unsigned int height_cells = height * (1 / planner_->getCostmap()->getResolution());

  addObstacleCells(x_cells, y_cells, width_cells, height_cells);
}

void Tester::broadcastRobotPose()
{
  geometry_msgs::msg::TransformStamped pose;
  pose.header.stamp = tester_node_->get_clock()->now();
  pose.header.frame_id = "map";
  pose.child_frame_id = "base_link";

  pose.transform.translation.x = 0.5;
  pose.transform.translation.y = 0.1;
  pose.transform.translation.z = 0.0;
  pose.transform.rotation.x = 0.0;
  pose.transform.rotation.y = 0.0;
  pose.transform.rotation.z = 0.0;
  pose.transform.rotation.w = 1.0;

  robot_pose_broadcaster_->sendTransform(pose);
}

void Tester::publishMap()
{
  map_.header.stamp = tester_node_->now();

  RCLCPP_DEBUG(tester_node_->get_logger(), "Publishing map...");
  map_publisher_->publish(map_);
}

TEST_F(Tester, testPlannerNoObstacles)
{
  using namespace std::chrono_literals;

  setParameters();
  startTesterNode();
  planner_->start();

  publishMap();
  std::this_thread::sleep_for(500ms);

  ASSERT_TRUE(action_client_->wait_for_action_server(2s));

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.5;
  pose1.position.y = 0.1;
  pose1.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.5;
  pose2.position.y = 0.3;
  pose2.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.5;
  pose3.position.y = 0.7;
  pose3.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.5;
  pose4.position.y = 0.9;
  pose4.orientation.w = 1.0;

  Action::Goal goal;
  setGoalFromPoses({pose1, pose2, pose3, pose4}, goal);

  auto gh_future = action_client_->async_send_goal(goal);

  ActionGoalHandle::WrappedResult action_result;
  waitForResult(gh_future, action_result, 2s);

  ASSERT_EQ(action_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  auto & result = *action_result.result;

  EXPECT_EQ(result.error_code, Action::Result::NONE);
  EXPECT_EQ(result.last_reached_index, Action::Result::ALL_GOALS);

  const auto & result_poses = result.path.poses;
  EXPECT_NEAR(result_poses.back().pose.position.x, pose4.position.x, PLANNER_TOLERANCE);
  EXPECT_NEAR(result_poses.back().pose.position.y, pose4.position.y, PLANNER_TOLERANCE);

  planner_->stop();
  stopTesterNode();
}

TEST_F(Tester, testPlannerWithNearObstacle)
{
  using namespace std::chrono_literals;

  setParameters();
  startTesterNode();
  planner_->start();

  addObstacle(0.5, 0.3, 1, 0.1);
  publishMap();
  std::this_thread::sleep_for(500ms);

  ASSERT_TRUE(action_client_->wait_for_action_server(2s));

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.5;
  pose1.position.y = 0.1;
  pose1.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.5;
  pose2.position.y = 0.5;
  pose2.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.5;
  pose3.position.y = 0.7;
  pose3.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.5;
  pose4.position.y = 0.9;
  pose4.orientation.w = 1.0;

  Action::Goal goal;
  setGoalFromPoses({pose1, pose2, pose3, pose4}, goal);

  auto gh_future = action_client_->async_send_goal(goal);

  ActionGoalHandle::WrappedResult action_result;
  waitForResult(gh_future, action_result, 2s);

  ASSERT_EQ(action_result.code, rclcpp_action::ResultCode::ABORTED);

  planner_->stop();
  stopTesterNode();
}

TEST_F(Tester, testPlannerWithMiddleObstacle)
{
  using namespace std::chrono_literals;

  setParameters();
  startTesterNode();
  planner_->start();

  addObstacle(0.5, 0.5, 1, 0.1);
  publishMap();
  std::this_thread::sleep_for(500ms);

  ASSERT_TRUE(action_client_->wait_for_action_server(2s));

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.5;
  pose1.position.y = 0.1;
  pose1.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.5;
  pose2.position.y = 0.3;
  pose2.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.5;
  pose3.position.y = 0.7;
  pose3.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.5;
  pose4.position.y = 0.9;
  pose4.orientation.w = 1.0;

  Action::Goal goal;
  setGoalFromPoses({pose1, pose2, pose3, pose4}, goal);

  auto gh_future = action_client_->async_send_goal(goal);

  ActionGoalHandle::WrappedResult action_result;
  waitForResult(gh_future, action_result, 2s);

  ASSERT_EQ(action_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  auto & result = *action_result.result;

  EXPECT_EQ(result.error_code, Action::Result::NONE);

  // Pose 2 will be the first goal with index 0 in the goals list.
  EXPECT_EQ(result.last_reached_index, 0);

  const auto & result_poses = result.path.poses;
  EXPECT_NEAR(result_poses.back().pose.position.x, pose2.position.x, PLANNER_TOLERANCE);
  EXPECT_NEAR(result_poses.back().pose.position.y, pose2.position.y, PLANNER_TOLERANCE);

  planner_->stop();
  stopTesterNode();
}

TEST_F(Tester, testPlannerWithFarObstacle)
{
  using namespace std::chrono_literals;

  setParameters();
  startTesterNode();
  planner_->start();

  addObstacle(0.5, 0.7, 1, 0.1);
  publishMap();
  std::this_thread::sleep_for(500ms);

  ASSERT_TRUE(action_client_->wait_for_action_server(2s));

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.5;
  pose1.position.y = 0.1;
  pose1.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.5;
  pose2.position.y = 0.3;
  pose2.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.5;
  pose3.position.y = 0.5;
  pose3.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.5;
  pose4.position.y = 0.9;
  pose4.orientation.w = 1.0;

  Action::Goal goal;
  setGoalFromPoses({pose1, pose2, pose3, pose4}, goal);

  auto gh_future = action_client_->async_send_goal(goal);

  ActionGoalHandle::WrappedResult action_result;
  waitForResult(gh_future, action_result, 2s);

  ASSERT_EQ(action_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  auto & result = *action_result.result;

  EXPECT_EQ(result.error_code, Action::Result::NONE);

  // Pose 3 will be the second goal with index 1 in the goals list.
  EXPECT_EQ(result.last_reached_index, 1);

  const auto & result_poses = result.path.poses;
  EXPECT_NEAR(result_poses.back().pose.position.x, pose3.position.x, PLANNER_TOLERANCE);
  EXPECT_NEAR(result_poses.back().pose.position.y, pose3.position.y, PLANNER_TOLERANCE);

  planner_->stop();
  stopTesterNode();
}

TEST_F(Tester, testPlannerWithoutPartialPlanning)
{
  using namespace std::chrono_literals;

  setParameters();
  startTesterNode();
  planner_->start();

  // Disable partial planning
  planner_->set_parameter(rclcpp::Parameter("allow_partial_planning", false));

  addObstacle(0.5, 0.5, 1, 0.1);
  publishMap();
  std::this_thread::sleep_for(500ms);

  ASSERT_TRUE(action_client_->wait_for_action_server(2s));

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.5;
  pose1.position.y = 0.1;
  pose1.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.5;
  pose2.position.y = 0.3;
  pose2.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.5;
  pose3.position.y = 0.7;
  pose3.orientation.w = 1.0;

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.5;
  pose4.position.y = 0.9;
  pose4.orientation.w = 1.0;

  Action::Goal goal;
  setGoalFromPoses({pose1, pose2, pose3, pose4}, goal);

  auto gh_future = action_client_->async_send_goal(goal);

  ActionGoalHandle::WrappedResult action_result;
  waitForResult(gh_future, action_result, 2s);

  ASSERT_EQ(action_result.code, rclcpp_action::ResultCode::ABORTED);

  planner_->stop();
  stopTesterNode();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
