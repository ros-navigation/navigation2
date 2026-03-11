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
#include <limits>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/bounded_tracking_error_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcutils/logging.h"

namespace nav2_costmap_2d
{

// Constants for test configuration
static constexpr double TEST_CORRIDOR_WIDTH = 2.0;
static constexpr double TEST_LOOK_AHEAD = 3.0;
static constexpr int TEST_STEP = 5;
static constexpr int TEST_WALL_THICKNESS = 1;
static constexpr int TEST_CORRIDOR_COST = 190;
static constexpr double TEST_RESOLUTION = 0.05;
static constexpr unsigned int TEST_SIZE_X = 100;
static constexpr unsigned int TEST_SIZE_Y = 100;
static constexpr double TEST_ORIGIN_X = -2.5;
static constexpr double TEST_ORIGIN_Y = -2.5;

/**
 * @brief Wrapper class to expose protected/private members for testing
 */
class BoundedTrackingErrorLayerTestable : public BoundedTrackingErrorLayer
{
public:
  // Expose atomic member directly (public access)
  using BoundedTrackingErrorLayer::current_path_index_;

  // Expose protected methods
  using BoundedTrackingErrorLayer::pathCallback;
  using BoundedTrackingErrorLayer::trackingCallback;
  using BoundedTrackingErrorLayer::goalCallback;
  using BoundedTrackingErrorLayer::resetState;
  using BoundedTrackingErrorLayer::validateParameterUpdatesCallback;
  using BoundedTrackingErrorLayer::updateParametersCallback;

  // Getter methods for private members (thread-safe where needed)
  nav_msgs::msg::Path getLastPath() const
  {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(data_mutex_));
    return last_path_;
  }

  geometry_msgs::msg::PoseStamped getLastGoal() const
  {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(data_mutex_));
    return last_goal_;
  }

  bool getEnabled() const
  {
    return enabled_.load();
  }

  double getLookAhead() const
  {
    return look_ahead_;
  }

  double getCorridorWidth() const
  {
    return corridor_width_;
  }

  int getStep() const
  {
    return step_;
  }

  size_t getStepSize() const
  {
    return step_size_;
  }

  int getWallThickness() const
  {
    return wall_thickness_;
  }

  unsigned char getCorridorCost() const
  {
    return corridor_cost_;
  }

  int getPathSegmentResolution() const
  {
    return path_segment_resolution_;
  }

  // Public overloads for testing
  void getPathSegmentPublic(
    const nav_msgs::msg::Path & path,
    size_t path_index,
    nav_msgs::msg::Path & segment)
  {
    getPathSegment(path, path_index, segment);
  }

  void getWallPolygonsPublic(const nav_msgs::msg::Path & segment, WallPolygons & walls)
  {
    getWallPolygons(segment, walls);
  }
};

/**
 * @brief Test fixture for BoundedTrackingErrorLayer tests
 */
class BoundedTrackingErrorLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node
    node_ = std::make_shared<nav2::LifecycleNode>("test_bounded_tracking_error_layer");

    // Configure and activate node
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    // Allow time for initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.spin_some();

    // Create costmap infrastructure
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
      TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);
    layered_costmap_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>(
      "map", false, false);
    layered_costmap_->resizeMap(
      TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);

    // Create and initialize layer
    layer_ = std::make_shared<BoundedTrackingErrorLayerTestable>();
    layer_->initialize(layered_costmap_.get(), "test_layer", nullptr, node_, nullptr);
    layer_->activate();
  }

  void TearDown() override
  {
    if (layer_) {
      layer_->deactivate();
    }

    if (node_ &&
      node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
    {
      node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
      node_->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    }
  }

  // Helper methods for creating test data
  nav_msgs::msg::Path createStraightPath(
    double start_x, double start_y, double end_x, double end_y, int num_points)
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

  nav_msgs::msg::Path createCurvedPath(double radius, int num_points)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (int i = 0; i < num_points; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      double angle = M_PI * static_cast<double>(i) / (num_points - 1);
      pose.pose.position.x = radius * std::cos(angle);
      pose.pose.position.y = radius * std::sin(angle);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  nav2_msgs::msg::TrackingFeedback createTrackingFeedback(
    uint32_t path_index, double error, double robot_x = 0.0, double robot_y = 0.0)
  {
    nav2_msgs::msg::TrackingFeedback feedback;
    feedback.header.frame_id = "map";
    feedback.header.stamp = node_->now();
    feedback.current_path_index = path_index;
    feedback.position_tracking_error = error;
    feedback.robot_pose.header = feedback.header;
    feedback.robot_pose.pose.position.x = robot_x;
    feedback.robot_pose.pose.position.y = robot_y;
    feedback.robot_pose.pose.position.z = 0.0;
    feedback.robot_pose.pose.orientation.w = 1.0;
    return feedback;
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

  bool hasObstaclesInRegion(
    const nav2_costmap_2d::Costmap2D & costmap,
    unsigned int min_x, unsigned int min_y,
    unsigned int max_x, unsigned int max_y)
  {
    for (unsigned int i = min_x; i < max_x && i < costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = min_y; j < max_y && j < costmap.getSizeInCellsY(); ++j) {
        unsigned char cost = costmap.getCost(i, j);
        if (cost >= TEST_CORRIDOR_COST) {
          return true;
        }
      }
    }
    return false;
  }

  int countObstaclesInCostmap(const nav2_costmap_2d::Costmap2D & costmap)
  {
    int count = 0;
    for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
        unsigned char cost = costmap.getCost(i, j);
        if (cost >= TEST_CORRIDOR_COST) {
          count++;
        }
      }
    }
    return count;
  }

  // Test members
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<BoundedTrackingErrorLayerTestable> layer_;
};

// ============================================================================
// INITIALIZATION AND LIFECYCLE TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, InitializationSuccess)
{
  ASSERT_NE(layer_, nullptr);
  EXPECT_FALSE(layer_->isClearable());
  EXPECT_TRUE(layer_->getEnabled());
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

TEST_F(BoundedTrackingErrorLayerTest, ActivateDeactivateCycle)
{
  // Layer should be active after setup
  EXPECT_TRUE(layer_->getEnabled());

  // Deactivate
  layer_->deactivate();
  EXPECT_FALSE(layer_->getEnabled());
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);

  // Reactivate
  layer_->activate();
  EXPECT_TRUE(layer_->getEnabled());
}

TEST_F(BoundedTrackingErrorLayerTest, ResetClearsState)
{
  // Set up some state
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  auto tracking = createTrackingFeedback(25, 0.1);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  EXPECT_EQ(layer_->current_path_index_.load(), 25u);

  // Reset should clear everything
  layer_->reset();

  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
  EXPECT_TRUE(layer_->getLastPath().poses.empty());
}

// ============================================================================
// PATH CALLBACK TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, PathCallbackStoresValidPath)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  auto cached_path = layer_->getLastPath();
  EXPECT_EQ(cached_path.poses.size(), 50u);
  EXPECT_EQ(cached_path.header.frame_id, "map");
}

TEST_F(BoundedTrackingErrorLayerTest, PathCallbackRejectsStaleData)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);

  // Make path stale (>2 seconds old)
  path.header.stamp = rclcpp::Time(node_->now().nanoseconds() - 3000000000LL);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  EXPECT_TRUE(layer_->getLastPath().poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, PathCallbackDetectsPathUpdate)
{
  // Set initial path
  auto path1 = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path1));
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(10, 0.1)));

  EXPECT_EQ(layer_->current_path_index_.load(), 10u);

  // Send updated path with different size
  auto path2 = createStraightPath(0.0, 0.0, 5.0, 0.0, 30);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path2));

  // State should be reset
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

TEST_F(BoundedTrackingErrorLayerTest, PathCallbackHandlesEmptyPath)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "map";
  empty_path.header.stamp = node_->now();

  EXPECT_NO_THROW(layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(empty_path)));
}

// ============================================================================
// TRACKING CALLBACK TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, TrackingCallbackUpdatesIndex)
{
  auto tracking = createTrackingFeedback(42, 0.15);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  EXPECT_EQ(layer_->current_path_index_.load(), 42u);
}

TEST_F(BoundedTrackingErrorLayerTest, TrackingCallbackRejectsStaleData)
{
  auto tracking1 = createTrackingFeedback(10, 0.1);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking1));
  EXPECT_EQ(layer_->current_path_index_.load(), 10u);

  // Send stale tracking data (>1 second old)
  auto tracking2 = createTrackingFeedback(20, 0.1);
  tracking2.header.stamp = rclcpp::Time(node_->now().nanoseconds() - 2000000000LL);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking2));

  // Index should remain unchanged
  EXPECT_EQ(layer_->current_path_index_.load(), 10u);
}

TEST_F(BoundedTrackingErrorLayerTest, TrackingCallbackThreadSafe)
{
  std::atomic<bool> running{true};
  std::atomic<int> errors{0};

  // Spawn multiple threads updating tracking feedback
  auto worker = [&]() {
      for (int i = 0; i < 100 && running; ++i) {
        try {
          auto tracking = createTrackingFeedback(i % 50, 0.1);
          layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));
          std::this_thread::sleep_for(std::chrono::microseconds(10));
        } catch (...) {
          errors++;
        }
      }
    };

  std::vector<std::thread> threads;
  for (int i = 0; i < 5; ++i) {
    threads.emplace_back(worker);
  }

  for (auto & t : threads) {
    t.join();
  }

  running = false;
  EXPECT_EQ(errors.load(), 0);
}

// ============================================================================
// GOAL CALLBACK TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, GoalCallbackDetectsNewGoal)
{
  // Set up state with path and tracking
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(15, 0.1)));

  EXPECT_EQ(layer_->current_path_index_.load(), 15u);

  // Send new goal (>0.1m away)
  auto new_goal = createGoal(10.0, 10.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(new_goal));

  // State should be reset
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
  EXPECT_TRUE(layer_->getLastPath().poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, GoalCallbackIgnoresSameGoal)
{
  // Set initial goal
  auto goal1 = createGoal(10.0, 10.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(goal1));

  // Set up state
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(20, 0.1)));

  EXPECT_EQ(layer_->current_path_index_.load(), 20u);

  // Send nearly identical goal (within 0.1m threshold)
  auto goal2 = createGoal(10.05, 10.05);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(goal2));

  // State should NOT be reset
  EXPECT_EQ(layer_->current_path_index_.load(), 20u);
}

TEST_F(BoundedTrackingErrorLayerTest, GoalCallbackFirstGoalAlwaysNew)
{
  // First goal should always be considered new
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(
    std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTrackingFeedback(10, 0.1)));

  EXPECT_EQ(layer_->current_path_index_.load(), 10u);

  auto goal = createGoal(5.0, 0.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(goal));

  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

// ============================================================================
// PATH SEGMENT GENERATION TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, PathSegmentBasicGeneration)
{
  auto path = createStraightPath(0.0, 0.0, 10.0, 0.0, 100);
  nav_msgs::msg::Path segment;

  layer_->getPathSegmentPublic(path, 0, segment);

  EXPECT_FALSE(segment.poses.empty());
  EXPECT_LT(segment.poses.size(), path.poses.size());
  EXPECT_EQ(segment.header.frame_id, path.header.frame_id);
}

TEST_F(BoundedTrackingErrorLayerTest, PathSegmentRespectsLookAhead)
{
  auto path = createStraightPath(0.0, 0.0, 10.0, 0.0, 100);
  nav_msgs::msg::Path segment;

  layer_->getPathSegmentPublic(path, 0, segment);

  // Calculate actual length of segment
  double segment_length = 0.0;
  for (size_t i = 0; i < segment.poses.size() - 1; ++i) {
    double dx = segment.poses[i + 1].pose.position.x - segment.poses[i].pose.position.x;
    double dy = segment.poses[i + 1].pose.position.y - segment.poses[i].pose.position.y;
    segment_length += std::hypot(dx, dy);
  }

  // Segment length should be approximately look_ahead distance
  EXPECT_LE(segment_length, layer_->getLookAhead() + 0.5);  // Allow some tolerance
}

TEST_F(BoundedTrackingErrorLayerTest, PathSegmentAtDifferentIndices)
{
  auto path = createStraightPath(0.0, 0.0, 10.0, 0.0, 100);
  nav_msgs::msg::Path segment1, segment2, segment3;

  layer_->getPathSegmentPublic(path, 0, segment1);
  layer_->getPathSegmentPublic(path, 25, segment2);
  layer_->getPathSegmentPublic(path, 75, segment3);

  EXPECT_FALSE(segment1.poses.empty());
  EXPECT_FALSE(segment2.poses.empty());
  EXPECT_FALSE(segment3.poses.empty());

  // Segment near end should be shorter
  EXPECT_LT(segment3.poses.size(), segment1.poses.size());
}

TEST_F(BoundedTrackingErrorLayerTest, PathSegmentInvalidIndex)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  nav_msgs::msg::Path segment;

  // Index beyond path
  layer_->getPathSegmentPublic(path, 100, segment);
  EXPECT_TRUE(segment.poses.empty());

  // Index at exactly path size
  layer_->getPathSegmentPublic(path, 50, segment);
  EXPECT_TRUE(segment.poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, PathSegmentEmptyPath)
{
  nav_msgs::msg::Path empty_path;
  nav_msgs::msg::Path segment;

  layer_->getPathSegmentPublic(empty_path, 0, segment);
  EXPECT_TRUE(segment.poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, PathSegmentLastPose)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  nav_msgs::msg::Path segment;

  // Segment from last pose
  layer_->getPathSegmentPublic(path, 49, segment);

  EXPECT_EQ(segment.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(segment.poses[0].pose.position.x, path.poses[49].pose.position.x);
}

// ============================================================================
// WALL POLYGON GENERATION TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsBasicGeneration)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  WallPolygons walls;

  layer_->getWallPolygonsPublic(path, walls);

  EXPECT_FALSE(walls.isEmpty());
  EXPECT_FALSE(walls.left_outer.empty());
  EXPECT_FALSE(walls.right_outer.empty());
  EXPECT_EQ(walls.left_outer.size(), walls.right_outer.size());
  EXPECT_EQ(walls.left_outer.size(), walls.perpendiculars.size());
  EXPECT_EQ(walls.left_outer.size(), walls.tangents.size());
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsValidCoordinates)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  WallPolygons walls;

  layer_->getWallPolygonsPublic(path, walls);

  // All coordinates should be finite
  for (const auto & point : walls.left_outer) {
    EXPECT_TRUE(std::isfinite(point[0])) << "Left outer X not finite";
    EXPECT_TRUE(std::isfinite(point[1])) << "Left outer Y not finite";
  }

  for (const auto & point : walls.right_outer) {
    EXPECT_TRUE(std::isfinite(point[0])) << "Right outer X not finite";
    EXPECT_TRUE(std::isfinite(point[1])) << "Right outer Y not finite";
  }
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsSeparation)
{
  // Straight horizontal path
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  WallPolygons walls;

  layer_->getWallPolygonsPublic(path, walls);

  ASSERT_FALSE(walls.left_outer.empty());
  ASSERT_FALSE(walls.right_outer.empty());

  // For horizontal path, left and right walls should be on opposite sides of Y=0
  // Left should have positive Y, right should have negative Y (or vice versa)
  double left_y = walls.left_outer[0][1];
  double right_y = walls.right_outer[0][1];

  EXPECT_TRUE((left_y > 0 && right_y < 0) || (left_y < 0 && right_y > 0))
    << "Left Y: " << left_y << ", Right Y: " << right_y;
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsCorridorWidth)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  WallPolygons walls;

  layer_->getWallPolygonsPublic(path, walls);

  ASSERT_FALSE(walls.left_outer.empty());
  ASSERT_FALSE(walls.right_outer.empty());

  // Distance between left and right walls should be approximately corridor_width
  // plus wall thickness on each side
  double expected_separation = layer_->getCorridorWidth() +
    2 * layer_->getWallThickness() * TEST_RESOLUTION;

  double left_y = walls.left_outer[0][1];
  double right_y = walls.right_outer[0][1];
  double actual_separation = std::abs(left_y - right_y);

  EXPECT_NEAR(actual_separation, expected_separation, 0.1);
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsEmptyPath)
{
  nav_msgs::msg::Path empty_path;
  WallPolygons walls;

  layer_->getWallPolygonsPublic(empty_path, walls);
  EXPECT_TRUE(walls.isEmpty());
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsSinglePoint)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = node_->now();

  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 1.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  WallPolygons walls;
  layer_->getWallPolygonsPublic(path, walls);

  EXPECT_TRUE(walls.isEmpty());
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsCurvedPath)
{
  auto path = createCurvedPath(5.0, 50);
  WallPolygons walls;

  layer_->getWallPolygonsPublic(path, walls);

  EXPECT_FALSE(walls.isEmpty());

  // Verify perpendiculars and tangents are unit vectors
  for (size_t i = 0; i < walls.perpendiculars.size(); ++i) {
    double perp_mag = std::hypot(walls.perpendiculars[i][0], walls.perpendiculars[i][1]);
    double tang_mag = std::hypot(walls.tangents[i][0], walls.tangents[i][1]);

    EXPECT_NEAR(perp_mag, 1.0, 0.01) << "Perpendicular not unit at index " << i;
    EXPECT_NEAR(tang_mag, 1.0, 0.01) << "Tangent not unit at index " << i;
  }
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsNaNHandling)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = node_->now();

  for (int i = 0; i < 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = (i == 5) ? std::numeric_limits<double>::quiet_NaN() : i * 0.5;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  WallPolygons walls;
  EXPECT_NO_THROW(layer_->getWallPolygonsPublic(path, walls));
}

// ============================================================================
// UPDATE BOUNDS TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, UpdateBoundsExpandsRegion)
{
  double min_x = 0.0, min_y = 0.0, max_x = 1.0, max_y = 1.0;
  double robot_x = 0.5, robot_y = 0.5, robot_yaw = 0.0;

  layer_->updateBounds(robot_x, robot_y, robot_yaw, &min_x, &min_y, &max_x, &max_y);

  // Bounds should expand by look_ahead in all directions
  EXPECT_LE(min_x, robot_x - layer_->getLookAhead());
  EXPECT_LE(min_y, robot_y - layer_->getLookAhead());
  EXPECT_GE(max_x, robot_x + layer_->getLookAhead());
  EXPECT_GE(max_y, robot_y + layer_->getLookAhead());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateBoundsDisabledLayer)
{
  layer_->deactivate();

  double min_x = 0.0, min_y = 0.0, max_x = 1.0, max_y = 1.0;
  double robot_x = 0.5, robot_y = 0.5, robot_yaw = 0.0;

  layer_->updateBounds(robot_x, robot_y, robot_yaw, &min_x, &min_y, &max_x, &max_y);

  // When disabled, updateBounds returns early without modifying bounds significantly
  // Re-enable for other tests
  layer_->activate();
}

// ============================================================================
// UPDATE COSTS TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsWithNoData)
{
  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));

  // Should not add any obstacles without data
  EXPECT_EQ(countObstaclesInCostmap(master_grid), 0);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsWithValidData)
{
  // Set up path and tracking
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  auto tracking = createTrackingFeedback(10, 0.1, 1.0, 0.0);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  // Allow time for callbacks to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y);

  // Should have obstacles representing corridor walls
  int obstacle_count = countObstaclesInCostmap(master_grid);
  EXPECT_GT(obstacle_count, 0) << "No corridor walls generated";
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsDisabledLayer)
{
  // Set up data
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  auto tracking = createTrackingFeedback(10, 0.1);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  // Disable layer
  layer_->deactivate();

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y);

  // Should not add obstacles when disabled
  EXPECT_EQ(countObstaclesInCostmap(master_grid), 0);

  // Re-enable
  layer_->activate();
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsShortSegment)
{
  // Path with only 1 pose in segment (too short for walls)
  auto path = createStraightPath(0.0, 0.0, 0.5, 0.0, 2);
  auto tracking = createTrackingFeedback(0, 0.1);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsMultipleTimes)
{
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  auto tracking = createTrackingFeedback(10, 0.1);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  // Multiple updates should work without errors
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));

  int obstacle_count = countObstaclesInCostmap(master_grid);
  EXPECT_GT(obstacle_count, 0);
}

// ============================================================================
// PARAMETER VALIDATION TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterLookAheadValid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.look_ahead", 2.5)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterLookAheadInvalid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.look_ahead", -1.0)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "look_ahead must be positive");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterCorridorWidthValid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.corridor_width", 3.0)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterCorridorWidthInvalid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.corridor_width", 0.0)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "corridor_width must be positive");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterStepValid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.step", 10)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterStepInvalid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.step", 0)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "step must be greater than zero");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterCorridorCostValid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.corridor_cost", 200)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterCorridorCostInvalid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.corridor_cost", 300)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "corridor_cost must be between 1 and 254");

  params = {rclcpp::Parameter("test_layer.corridor_cost", 0)};
  result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterWallThicknessValid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.wall_thickness", 2)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterWallThicknessInvalid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.wall_thickness", -1)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "wall_thickness must be greater than zero");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterPathSegmentResolutionValid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.path_segment_resolution", 5)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterPathSegmentResolutionInvalid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.path_segment_resolution", 0)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "path_segment_resolution must be at least 1");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterEnabledValid)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.enabled", false)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameterNonLayerParameter)
{
  // Parameter for different layer/component should be ignored
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("other_layer.some_param", 42)
  };

  auto result = layer_->validateParameterUpdatesCallback(params);
  EXPECT_TRUE(result.successful);
}

// ============================================================================
// PARAMETER UPDATE TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, UpdateParameterLookAhead)
{
  double original_value = layer_->getLookAhead();

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.look_ahead", 4.0)
  };

  EXPECT_NO_THROW(layer_->updateParametersCallback(params));

  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 4.0);
  EXPECT_NE(layer_->getLookAhead(), original_value);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParameterCorridorWidth)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.corridor_width", 3.5)
  };

  EXPECT_NO_THROW(layer_->updateParametersCallback(params));

  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 3.5);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParameterEnabled)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.enabled", false)
  };

  EXPECT_NO_THROW(layer_->updateParametersCallback(params));
  EXPECT_FALSE(layer_->getEnabled());

  params = {rclcpp::Parameter("test_layer.enabled", true)};
  EXPECT_NO_THROW(layer_->updateParametersCallback(params));
  EXPECT_TRUE(layer_->getEnabled());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParameterStep)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.step", 8)
  };

  EXPECT_NO_THROW(layer_->updateParametersCallback(params));

  EXPECT_EQ(layer_->getStep(), 8);
  EXPECT_EQ(layer_->getStepSize(), 8u);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateMultipleParameters)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.look_ahead", 2.5),
    rclcpp::Parameter("test_layer.corridor_width", 2.5),
    rclcpp::Parameter("test_layer.step", 7),
    rclcpp::Parameter("test_layer.enabled", false)
  };

  EXPECT_NO_THROW(layer_->updateParametersCallback(params));

  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 2.5);
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 2.5);
  EXPECT_EQ(layer_->getStep(), 7);
  EXPECT_FALSE(layer_->getEnabled());
}

// ============================================================================
// INTEGRATION TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, IntegrationFullPipeline)
{
  // Complete workflow: goal -> path -> tracking -> costmap update
  auto goal = createGoal(5.0, 0.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(goal));

  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  auto tracking = createTrackingFeedback(10, 0.1, 1.0, 0.0);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));

  int obstacle_count = countObstaclesInCostmap(master_grid);
  EXPECT_GT(obstacle_count, 0) << "Integration test: no corridor generated";
}

TEST_F(BoundedTrackingErrorLayerTest, IntegrationMovingRobot)
{
  // Create a path within costmap bounds (costmap is -2.5 to 2.5 in x and y)
  auto path = createStraightPath(-1.5, 0.0, 1.5, 0.0, 100);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  // Test at a few representative positions along the path
  std::vector<int> test_indices = {10, 25, 40, 55};
  int successful_corridors = 0;

  for (int i : test_indices) {
    auto tracking = createTrackingFeedback(i, 0.1);
    layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

    // Allow time for callback processing
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Clear costmap
    master_grid.resetMap(0, 0, TEST_SIZE_X, TEST_SIZE_Y);

    EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));

    // Count obstacles
    int obstacle_count = countObstaclesInCostmap(master_grid);
    if (obstacle_count > 0) {
      successful_corridors++;
    }
  }

  // At least half of the test positions should successfully generate corridors
  EXPECT_GE(successful_corridors, 2)
    << "Only " << successful_corridors << " out of " << test_indices.size()
    << " positions generated corridors";
}

TEST_F(BoundedTrackingErrorLayerTest, IntegrationNewGoalClearsOldCorridor)
{
  // Set up initial path and corridor
  auto path1 = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path1));
  auto tracking1 = createTrackingFeedback(10, 0.1);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking1));

  nav2_costmap_2d::Costmap2D master_grid1(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);
  layer_->updateCosts(master_grid1, 0, 0, TEST_SIZE_X, TEST_SIZE_Y);

  int initial_obstacles = countObstaclesInCostmap(master_grid1);
  EXPECT_GT(initial_obstacles, 0);

  // Send new goal (far away)
  auto new_goal = createGoal(10.0, 10.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(new_goal));

  // State should be cleared
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
  EXPECT_TRUE(layer_->getLastPath().poses.empty());
}

// ============================================================================
// EDGE CASE AND STRESS TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, EdgeCaseVeryShortPath)
{
  auto path = createStraightPath(0.0, 0.0, 0.1, 0.0, 2);
  auto tracking = createTrackingFeedback(0, 0.01);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
}

TEST_F(BoundedTrackingErrorLayerTest, EdgeCaseVeryLongPath)
{
  auto path = createStraightPath(0.0, 0.0, 100.0, 0.0, 1000);
  auto tracking = createTrackingFeedback(500, 0.1);

  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
}

TEST_F(BoundedTrackingErrorLayerTest, EdgeCaseRapidCallbacks)
{
  // Rapid-fire callbacks to test thread safety
  auto path = createStraightPath(0.0, 0.0, 5.0, 0.0, 50);

  for (int i = 0; i < 100; ++i) {
    layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
    layer_->trackingCallback(
      std::make_shared<nav2_msgs::msg::TrackingFeedback>(
        createTrackingFeedback(i % 50, 0.1)));
  }

  // Should complete without crashes
  EXPECT_EQ(layer_->current_path_index_.load(), 49u);
}

TEST_F(BoundedTrackingErrorLayerTest, StressTestManyUpdates)
{
  auto path = createStraightPath(0.0, 0.0, 10.0, 0.0, 100);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  nav2_costmap_2d::Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION,
    TEST_ORIGIN_X, TEST_ORIGIN_Y);

  // Many update cycles
  for (int iteration = 0; iteration < 50; ++iteration) {
    auto tracking = createTrackingFeedback(iteration % 90, 0.1);
    layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

    master_grid.resetMap(0, 0, TEST_SIZE_X, TEST_SIZE_Y);
    EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
  }
}

}  // namespace nav2_costmap_2d

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char ** argv)
{
  // Set logging level to reduce noise during tests
  if (rcutils_logging_set_logger_level(
      "test_bounded_tracking_error_layer",
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
