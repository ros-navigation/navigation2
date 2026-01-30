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

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/bounded_tracking_error_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcutils/logging.h"
// Test constants
static constexpr double TEST_CORRIDOR_WIDTH = 2.0;
static constexpr int TEST_PATH_POINTS = 50;
static constexpr double TEST_PATH_LENGTH = 5.0;

class BoundedTrackingErrorLayerWrapper : public nav2_costmap_2d::BoundedTrackingErrorLayer
{
public:
  using BoundedTrackingErrorLayer::last_tracking_feedback_;
  using BoundedTrackingErrorLayer::pathCallback;
  using BoundedTrackingErrorLayer::trackingCallback;
  using BoundedTrackingErrorLayer::getPathSegment;
  using BoundedTrackingErrorLayer::getWallPolygons;
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
    layer_->last_tracking_feedback_.current_path_index = static_cast<uint32_t>(-1);
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

  nav_msgs::msg::Path createCurvedPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (int i = 0; i <= 50; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      double angle = M_PI * i / 50.0;
      pose.pose.position.x = 2.0 * cos(angle);
      pose.pose.position.y = 2.0 * sin(angle);
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
    feedback.tracking_error = error;

    feedback.robot_pose.header = feedback.header;
    feedback.robot_pose.pose.position.x = path_index * 0.1;
    feedback.robot_pose.pose.position.y = 0.0;
    feedback.robot_pose.pose.orientation.w = 1.0;

    return feedback;
  }

  bool hasObstacles(const nav2_costmap_2d::Costmap2D & costmap)
  {
    for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
        unsigned char cost = costmap.getCost(i, j);
        // Check for any non-free cost (corridor walls have cost 250 by default)
        if (cost > nav2_costmap_2d::FREE_SPACE && cost != nav2_costmap_2d::NO_INFORMATION) {
          return true;
        }
      }
    }
    return false;
  }
};

// Basic initialization and lifecycle tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_initialization_and_parameters)
{
  EXPECT_TRUE(layer_ != nullptr);
  EXPECT_FALSE(layer_->isClearable());
  EXPECT_NO_THROW(layer_->activate());
  EXPECT_NO_THROW(layer_->deactivate());
}

// Empty data handling tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_empty_data_handling)
{
  nav2_costmap_2d::Costmap2D master_grid(100, 100, 0.05, -2.5, -2.5);
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, 100, 100));
  EXPECT_FALSE(hasObstacles(master_grid));
}

// Path segment extraction tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_path_segment_extraction)
{
  auto path = createStraightPath();
  auto tracking_error = createTrackingFeedback(10);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking_error));

  auto segment = layer_->getPathSegment();
  EXPECT_FALSE(segment.poses.empty());
  EXPECT_EQ(segment.header.frame_id, "map");
  EXPECT_GT(segment.poses.size(), 0u);
  EXPECT_GE(segment.poses[0].pose.position.x, tracking_error.robot_pose.pose.position.x);
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_path_segment_bounds)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 50);  // 50 points, 5m long
  auto tracking_error = createTrackingFeedback(25);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking_error));

  auto segment = layer_->getPathSegment();

  // Should extract segment around index 25
  EXPECT_FALSE(segment.poses.empty());
  EXPECT_LT(segment.poses.size(), path.poses.size());

  // Check segment boundaries make sense
  double first_x = segment.poses.front().pose.position.x;
  double last_x = segment.poses.back().pose.position.x;
  double robot_x = tracking_error.robot_pose.pose.position.x;

  EXPECT_LE(first_x, robot_x);
  EXPECT_GE(last_x, robot_x);
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_path_segment_edge_cases)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 50);

  // Test near start of path
  auto tracking_error_start = createTrackingFeedback(2);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(
    tracking_error_start));
  auto segment_start = layer_->getPathSegment();
  EXPECT_FALSE(segment_start.poses.empty());

  // Test near end of path
  auto tracking_error_end = createTrackingFeedback(47);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking_error_end));
  auto segment_end = layer_->getPathSegment();
  EXPECT_FALSE(segment_end.poses.empty());

  // Test exactly at end
  auto tracking_error_last = createTrackingFeedback(49);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking_error_last));
  auto segment_last = layer_->getPathSegment();
  EXPECT_FALSE(segment_last.poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_path_segment_length_consistency)
{
  auto path = createStraightPath(0.0, 0.0, 10.0, 100);

  // Test different positions, segment length should be consistent
  std::vector<size_t> test_indices = {10, 25, 50, 75, 90};
  std::vector<size_t> segment_sizes;

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  for (auto index : test_indices) {
    auto tracking_error = createTrackingFeedback(index);
    layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking_error));
    auto segment = layer_->getPathSegment();
    segment_sizes.push_back(segment.poses.size());
  }

  // Segments should have similar sizes
  for (size_t i = 1; i < segment_sizes.size() - 1; ++i) {
    EXPECT_GT(segment_sizes[i], 0u);
  }
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_invalid_path_index)
{
  auto path = createStraightPath();
  auto tracking_error = createTrackingFeedback(1000);  // Invalid index
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking_error));
  auto segment = layer_->getPathSegment();
  EXPECT_TRUE(segment.poses.empty());
}

// Wall points generation tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_wall_points_straight_path)
{
  auto path = createStraightPath();
  auto walls = layer_->getWallPolygons(path);
  EXPECT_FALSE(walls.left_outer.empty());
  EXPECT_FALSE(walls.right_outer.empty());
  EXPECT_EQ(walls.left_outer.size(), walls.left_inner.size());
  EXPECT_EQ(walls.right_outer.size(), walls.right_inner.size());

  // Check that all points are valid
  for (const auto & point : walls.left_outer) {
    EXPECT_EQ(point.size(), 2u);  // [x, y]
    EXPECT_TRUE(std::isfinite(point[0]));
    EXPECT_TRUE(std::isfinite(point[1]));
  }

  // Left and right should be on opposite sides for straight path
  if (!walls.left_outer.empty() && !walls.right_outer.empty()) {
    double left_y = walls.left_outer[0][1];
    double right_y = walls.right_outer[0][1];
    EXPECT_TRUE((left_y > 0 && right_y < 0) || (left_y < 0 && right_y > 0));
  }
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_wall_points_curved_path)
{
  auto curved_path = createCurvedPath();
  auto walls = layer_->getWallPolygons(curved_path);
  EXPECT_FALSE(walls.left_outer.empty());
  EXPECT_FALSE(walls.right_outer.empty());
  EXPECT_EQ(walls.left_outer.size(), walls.left_inner.size());
  EXPECT_EQ(walls.right_outer.size(), walls.right_inner.size());

  // Points should change along the curve
  if (walls.left_outer.size() >= 2) {
    double first_left_x = walls.left_outer[0][0];
    double second_left_x = walls.left_outer[1][0];
    EXPECT_NE(first_left_x, second_left_x);
  }
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_single_point_path)
{
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
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_update_costs_same_frame)
{
  auto path = createStraightPath();
  auto tracking_error = createTrackingFeedback(5);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking_error));

  nav2_costmap_2d::Costmap2D master_grid(100, 100, 0.05, -2.5, -2.5);
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, 100, 100));
}

// Edge cases and robustness tests
TEST_F(BoundedTrackingErrorLayerTestFixture, test_step_size_edge_cases)
{
  auto path = createStraightPath(0.0, 0.0, 1.0, 10);
  auto walls = layer_->getWallPolygons(path);

  // Should have reasonable number of points
  size_t total_points = walls.left_outer.size() + walls.right_outer.size();
  EXPECT_TRUE(total_points <= path.poses.size() * 2);
}

TEST_F(BoundedTrackingErrorLayerTestFixture, test_malformed_data)
{
  // Test with NaN coordinates
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
