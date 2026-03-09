// Copyright (c) 2025 Berkan Tali
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
// limitations under the License.

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <cmath>
#include <chrono>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/bounded_tracking_error_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcutils/logging.h"

static constexpr double TEST_CORRIDOR_WIDTH = 2.0;
static constexpr int TEST_PATH_POINTS = 50;
static constexpr double TEST_PATH_LENGTH = 5.0;

class BoundedTrackingErrorLayerWrapper : public nav2_costmap_2d::BoundedTrackingErrorLayer
{
public:
  using BoundedTrackingErrorLayer::current_path_index_;
  using BoundedTrackingErrorLayer::pathCallback;
  using BoundedTrackingErrorLayer::trackingCallback;
  using BoundedTrackingErrorLayer::goalCallback;
  using BoundedTrackingErrorLayer::getPathSegment;
  using BoundedTrackingErrorLayer::getWallPolygons;
  using BoundedTrackingErrorLayer::validateParameterUpdatesCallback;
  using BoundedTrackingErrorLayer::updateParametersCallback;
};

class BoundedTrackingErrorLayerTestFixture : public ::testing::Test
{
public:
  BoundedTrackingErrorLayerTestFixture()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("test_tracking_error_layer");
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.spin_some();
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.05, -2.5, -2.5);
    layered_costmap_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered_costmap_->resizeMap(100, 100, 0.05, -2.5, -2.5);

    layer_ = std::make_shared<BoundedTrackingErrorLayerWrapper>();
    layer_->initialize(layered_costmap_.get(), "test_layer", nullptr, node_, nullptr);
  }

  void SetUp() override
  {
    layer_->activate();
  }

  void TearDown() override
  {
    layer_->deactivate();
    if (node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
      node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
      node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    }
  }

protected:
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<BoundedTrackingErrorLayerWrapper> layer_;

  nav_msgs::msg::Path createStraightPath(
    double start_x = 0.0, double start_y = 0.0,
    double length = TEST_PATH_LENGTH, int num_points = TEST_PATH_POINTS)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (int i = 0; i < num_points; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = start_x + (i * length / num_points);
      pose.pose.position.y = start_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  nav2_msgs::msg::TrackingFeedback createTrackingFeedback(
    uint32_t path_index = 0,
    double error = 0.1)
  {
    nav2_msgs::msg::TrackingFeedback feedback;
    feedback.header.frame_id = "map";
    feedback.header.stamp = node_->now();
    feedback.current_path_index = path_index;
    feedback.position_tracking_error = error;

    feedback.robot_pose.header = feedback.header;
    feedback.robot_pose.pose.position.x = path_index * 0.1;
    feedback.robot_pose.pose.position.y = 0.0;
    feedback.robot_pose.pose.orientation.w = 1.0;

    return feedback;
  }

  nav_msgs::msg::Path createPathWithEndpoint(
    double start_x, double start_y,
    double end_x, double end_y,
    int num_points)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (int i = 0; i < num_points; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      double t = static_cast<double>(i) / (num_points - 1);
      pose.pose.position.x = start_x + t * (end_x - start_x);
      pose.pose.position.y = start_y + t * (end_y - start_y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  geometry_msgs::msg::PoseStamped createGoal(double x, double y)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = node_->now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;
    return goal;
  }

  bool hasObstacles(const nav2_costmap_2d::Costmap2D & costmap)
  {
    for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
        unsigned char cost = costmap.getCost(i, j);
        if (cost > nav2_costmap_2d::FREE_SPACE && cost != nav2_costmap_2d::NO_INFORMATION) {
          return true;
        }
      }
    }
    return false;
  }

  void setupLayerWithPathAndTracking(uint32_t path_index = 10)
  {
    auto path = createStraightPath(0.0, 0.0, 5.0, 50);
    auto tracking = createTrackingFeedback(path_index);
    layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
    layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));
  }
};

// Basic initialization and lifecycle tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_initialization)
{
  EXPECT_TRUE(layer_ != nullptr);
  EXPECT_FALSE(layer_->isClearable());
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_empty_data_handling)
{
  nav2_costmap_2d::Costmap2D master_grid(100, 100, 0.05, -2.5, -2.5);
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, 100, 100));
  EXPECT_FALSE(hasObstacles(master_grid));
}

// Path segment tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_path_segment_generation)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 50);
  auto tracking = createTrackingFeedback(25);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  auto segment = layer_->getPathSegment();

  EXPECT_FALSE(segment.poses.empty());
  EXPECT_LT(segment.poses.size(), path.poses.size());

  // Segment should start at or after robot position
  double first_x = segment.poses.front().pose.position.x;
  double robot_x = tracking.robot_pose.pose.position.x;
  EXPECT_GE(first_x, robot_x - 0.1);
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_path_segment_edge_indices)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  // Test start of path
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(2)));
  EXPECT_FALSE(layer_->getPathSegment().poses.empty());

  // Test near end of path
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(47)));
  EXPECT_FALSE(layer_->getPathSegment().poses.empty());

  // Test at last index
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(49)));
  EXPECT_FALSE(layer_->getPathSegment().poses.empty());

  // Test invalid index beyond path
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(1000)));
  EXPECT_TRUE(layer_->getPathSegment().poses.empty());
}

// Wall polygon tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_wall_polygons)
{
  auto path = createStraightPath();
  auto walls = layer_->getWallPolygons(path);

  // Valid walls should be generated
  EXPECT_FALSE(walls.left_outer.empty());
  EXPECT_FALSE(walls.right_outer.empty());
  EXPECT_EQ(walls.left_outer.size(), walls.left_inner.size());
  EXPECT_EQ(walls.right_outer.size(), walls.right_inner.size());

  // Wall points should be finite
  for (const auto & point : walls.left_outer) {
    EXPECT_EQ(point.size(), 2u);
    EXPECT_TRUE(std::isfinite(point[0]));
    EXPECT_TRUE(std::isfinite(point[1]));
  }

  // Left and right walls should be on opposite sides
  if (!walls.left_outer.empty() && !walls.right_outer.empty()) {
    double left_y = walls.left_outer[0][1];
    double right_y = walls.right_outer[0][1];
    EXPECT_TRUE((left_y > 0 && right_y < 0) || (left_y < 0 && right_y > 0));
  }
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_wall_polygons_edge_cases)
{
  // Single point path - no walls
  nav_msgs::msg::Path single_point_path;
  single_point_path.header.frame_id = "map";
  single_point_path.header.stamp = node_->now();
  geometry_msgs::msg::PoseStamped single_pose;
  single_pose.header = single_point_path.header;
  single_pose.pose.position.x = 1.0;
  single_pose.pose.position.y = 1.0;
  single_pose.pose.orientation.w = 1.0;
  single_point_path.poses.push_back(single_pose);

  auto walls = layer_->getWallPolygons(single_point_path);
  EXPECT_TRUE(walls.left_outer.empty());
  EXPECT_TRUE(walls.right_outer.empty());

  // NaN in path - should not crash
  nav_msgs::msg::Path nan_path;
  nan_path.header.frame_id = "map";
  nan_path.header.stamp = node_->now();
  geometry_msgs::msg::PoseStamped nan_pose;
  nan_pose.header = nan_path.header;
  nan_pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  nan_pose.pose.position.y = 0.0;
  nan_pose.pose.orientation.w = 1.0;
  nan_path.poses.push_back(nan_pose);

  EXPECT_NO_THROW(layer_->getWallPolygons(nan_path));
}

// Goal callback tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_goal_callback)
{
  setupLayerWithPathAndTracking(10);
  EXPECT_FALSE(layer_->getPathSegment().poses.empty());
  EXPECT_EQ(layer_->current_path_index_.load(), 10u);

  // New goal should clear state
  auto new_goal = createGoal(10.0, 10.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(new_goal));
  EXPECT_TRUE(layer_->getPathSegment().poses.empty());
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);

  // Same goal (within threshold) should not clear state
  setupLayerWithPathAndTracking(15);
  EXPECT_EQ(layer_->current_path_index_.load(), 15u);

  auto same_goal = createGoal(10.05, 10.05);  // Within 0.1 threshold
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(same_goal));
  EXPECT_EQ(layer_->current_path_index_.load(), 15u);
}

// Stale data rejection tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_stale_data_rejection)
{
  // Stale path should be rejected
  auto stale_path = createStraightPath(0.0, 0.0, 5.0, 50);
  stale_path.header.stamp = rclcpp::Time(node_->now().nanoseconds() - 3000000000LL);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(stale_path));
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(10)));
  EXPECT_TRUE(layer_->getPathSegment().poses.empty());

  // Fresh path with stale tracking feedback
  auto fresh_path = createStraightPath(0.0, 0.0, 5.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(fresh_path));

  auto fresh_tracking = createTrackingFeedback(5);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(fresh_tracking));
  EXPECT_EQ(layer_->current_path_index_.load(), 5u);

  auto stale_tracking = createTrackingFeedback(10);
  stale_tracking.header.stamp = rclcpp::Time(node_->now().nanoseconds() - 2000000000LL);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(stale_tracking));
  EXPECT_EQ(layer_->current_path_index_.load(), 5u);  // Should remain unchanged
}

// Path update detection tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_path_update_resets_state)
{
  // Create path with specific endpoint
  auto path1 = createPathWithEndpoint(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path1));
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(10)));
  EXPECT_EQ(layer_->current_path_index_.load(), 10u);

  // Path with same endpoint but different size triggers reset via isPathUpdated
  auto path2 = createPathWithEndpoint(0.0, 0.0, 5.0, 0.0, 30);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path2));
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

// Reset and deactivate tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_reset_clears_all_state)
{
  setupLayerWithPathAndTracking(10);

  EXPECT_FALSE(layer_->getPathSegment().poses.empty());
  EXPECT_EQ(layer_->current_path_index_.load(), 10u);

  layer_->reset();

  EXPECT_TRUE(layer_->getPathSegment().poses.empty());
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_deactivate_clears_all_state)
{
  setupLayerWithPathAndTracking(10);

  EXPECT_FALSE(layer_->getPathSegment().poses.empty());
  EXPECT_EQ(layer_->current_path_index_.load(), 10u);

  layer_->deactivate();

  EXPECT_TRUE(layer_->getPathSegment().poses.empty());
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);

  // Re-activate for TearDown
  layer_->activate();
}

// Parameter validation tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_parameter_validation)
{
  std::vector<rclcpp::Parameter> params;
  rcl_interfaces::msg::SetParametersResult result;

  // Valid parameters
  params = {rclcpp::Parameter("test_layer.look_ahead", 2.5)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);

  params = {rclcpp::Parameter("test_layer.corridor_width", 3.0)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);

  params = {rclcpp::Parameter("test_layer.enabled", false)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);

  params = {rclcpp::Parameter("test_layer.step", 10)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);

  params = {rclcpp::Parameter("test_layer.corridor_cost", 200)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);

  params = {rclcpp::Parameter("test_layer.wall_thickness", 2)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);

  // Invalid parameters
  params = {rclcpp::Parameter("test_layer.look_ahead", -1.0)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);

  params = {rclcpp::Parameter("test_layer.corridor_width", 0.0)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);

  params = {rclcpp::Parameter("test_layer.step", 0)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);

  params = {rclcpp::Parameter("test_layer.corridor_cost", 300)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);

  params = {rclcpp::Parameter("test_layer.wall_thickness", -1)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_parameter_updates_applied)
{
  std::vector<rclcpp::Parameter> params;

  params = {rclcpp::Parameter("test_layer.look_ahead", 2.5)};
  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
  EXPECT_NO_THROW(layer_->updateParametersCallback(params));

  params = {rclcpp::Parameter("test_layer.corridor_width", 3.0)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
  EXPECT_NO_THROW(layer_->updateParametersCallback(params));

  params = {rclcpp::Parameter("test_layer.enabled", false)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
  EXPECT_NO_THROW(layer_->updateParametersCallback(params));
}

int main(int argc, char ** argv)
{
  if (rcutils_logging_set_logger_level("test_tracking_error_layer",
    RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK)
  {
    std::cerr << "Failed to set logger level" << std::endl;
  }
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
