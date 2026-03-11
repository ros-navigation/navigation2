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

namespace nav2_costmap_2d
{

// Test configuration constants
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

// Testable wrapper class exposing protected members
class BoundedTrackingErrorLayerTestable : public BoundedTrackingErrorLayer
{
public:
  using BoundedTrackingErrorLayer::current_path_index_;
  using BoundedTrackingErrorLayer::pathCallback;
  using BoundedTrackingErrorLayer::trackingCallback;
  using BoundedTrackingErrorLayer::goalCallback;
  using BoundedTrackingErrorLayer::resetState;
  using BoundedTrackingErrorLayer::validateParameterUpdatesCallback;
  using BoundedTrackingErrorLayer::updateParametersCallback;

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

  bool getEnabled() const {return enabled_.load();}
  double getLookAhead() const {return look_ahead_;}
  double getCorridorWidth() const {return corridor_width_;}
  int getStep() const {return step_;}
  size_t getStepSize() const {return step_size_;}
  int getWallThickness() const {return wall_thickness_;}
  unsigned char getCorridorCost() const {return corridor_cost_;}
  int getPathSegmentResolution() const {return path_segment_resolution_;}

  void getPathSegmentPublic(
    const nav_msgs::msg::Path & path, size_t path_index, nav_msgs::msg::Path & segment)
  {
    getPathSegment(path, path_index, segment);
  }

  void getWallPolygonsPublic(const nav_msgs::msg::Path & segment, WallPolygons & walls)
  {
    getWallPolygons(segment, walls);
  }
};

// Main test fixture
class BoundedTrackingErrorLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("test_bounded_tracking_error_layer");
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.spin_some();

    costmap_ = std::make_shared<Costmap2D>(
      TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);
    layered_costmap_ = std::make_shared<LayeredCostmap>("map", false, false);
    layered_costmap_->resizeMap(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X,
        TEST_ORIGIN_Y);

    layer_ = std::make_shared<BoundedTrackingErrorLayerTestable>();
    layer_->initialize(layered_costmap_.get(), "test_layer", nullptr, node_, nullptr);
    layer_->activate();
  }

  void TearDown() override
  {
    if (layer_) {
      layer_->deactivate();
    }
    if (node_ && node_->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
    {
      node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
      node_->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    }
  }

  // Consolidated helper to create paths
  nav_msgs::msg::Path createPath(
    bool curved, int num_points,
    double length = 5.0, double radius = 5.0)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    for (int i = 0; i < num_points; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      if (curved) {
        double angle = M_PI * static_cast<double>(i) / (num_points - 1);
        pose.pose.position.x = radius * std::cos(angle);
        pose.pose.position.y = radius * std::sin(angle);
      } else {
        double t = static_cast<double>(i) / (num_points - 1);
        pose.pose.position.x = t * length;
        pose.pose.position.y = 0.0;
      }
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  nav2_msgs::msg::TrackingFeedback createTracking(
    uint32_t idx, double error = 0.1, double x = 0.0, double y = 0.0)
  {
    nav2_msgs::msg::TrackingFeedback feedback;
    feedback.header.frame_id = "map";
    feedback.header.stamp = node_->now();
    feedback.current_path_index = idx;
    feedback.position_tracking_error = error;
    feedback.robot_pose.header = feedback.header;
    feedback.robot_pose.pose.position.x = x;
    feedback.robot_pose.pose.position.y = y;
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

  int countObstacles(const Costmap2D & costmap)
  {
    int count = 0;
    for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
        if (costmap.getCost(i, j) >= TEST_CORRIDOR_COST) {
          count++;
        }
      }
    }
    return count;
  }

  // Helper for parameter validation tests
  void testParamValidation(
    const std::string & name, const rclcpp::Parameter & valid,
    const rclcpp::Parameter & invalid, const std::string & err_msg)
  {
    auto result = layer_->validateParameterUpdatesCallback({valid});
    EXPECT_TRUE(result.successful) << "Valid " << name << " rejected";

    result = layer_->validateParameterUpdatesCallback({invalid});
    EXPECT_FALSE(result.successful) << "Invalid " << name << " accepted";
    EXPECT_EQ(result.reason, err_msg);
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<Costmap2D> costmap_;
  std::shared_ptr<LayeredCostmap> layered_costmap_;
  std::shared_ptr<BoundedTrackingErrorLayerTestable> layer_;
};

// ============================================================================
// INITIALIZATION TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, Initialization) {
  EXPECT_FALSE(layer_->isClearable());
  EXPECT_TRUE(layer_->getEnabled());
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

TEST_F(BoundedTrackingErrorLayerTest, ActivateDeactivateCycle) {
  EXPECT_TRUE(layer_->getEnabled());
  layer_->deactivate();
  EXPECT_FALSE(layer_->getEnabled());
  layer_->activate();
  EXPECT_TRUE(layer_->getEnabled());
}

TEST_F(BoundedTrackingErrorLayerTest, Reset) {
  auto path = createPath(false, 50);
  auto tracking = createTracking(25, 0.1);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));

  layer_->reset();
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
  EXPECT_TRUE(layer_->getLastPath().poses.empty());
}

// ============================================================================
// PATH CALLBACK TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, PathCallback) {
  auto path = createPath(false, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  EXPECT_EQ(layer_->getLastPath().poses.size(), 50u);
}

TEST_F(BoundedTrackingErrorLayerTest, PathCallbackStaleData) {
  auto path = createPath(false, 50);
  path.header.stamp = rclcpp::Time(node_->now().nanoseconds() - 3000000000LL);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  EXPECT_TRUE(layer_->getLastPath().poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, PathUpdate) {
  auto path1 = createPath(false, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path1));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTracking(10)));

  auto path2 = createPath(false, 30);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path2));
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

// ============================================================================
// TRACKING CALLBACK TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, TrackingCallback) {
  auto tracking = createTracking(42, 0.15);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));
  EXPECT_EQ(layer_->current_path_index_.load(), 42u);
}

TEST_F(BoundedTrackingErrorLayerTest, TrackingCallbackThreadSafe) {
  std::atomic<int> errors{0};
  auto worker = [&]() {
      for (int i = 0; i < 100; ++i) {
        try {
          layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(
          createTracking(i % 50)));
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
  EXPECT_EQ(errors.load(), 0);
}

// ============================================================================
// GOAL CALLBACK TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, GoalCallback) {
  auto path = createPath(false, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTracking(15)));

  auto goal = createGoal(10.0, 10.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(goal));
  EXPECT_EQ(layer_->current_path_index_.load(), 0u);
}

// ============================================================================
// PATH SEGMENT TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, PathSegment) {
  auto path = createPath(false, 100, 10.0);
  nav_msgs::msg::Path segment;
  layer_->getPathSegmentPublic(path, 0, segment);

  EXPECT_FALSE(segment.poses.empty());
  EXPECT_LT(segment.poses.size(), path.poses.size());
}

TEST_F(BoundedTrackingErrorLayerTest, PathSegmentLookAhead) {
  auto path = createPath(false, 100, 10.0);
  nav_msgs::msg::Path segment;
  layer_->getPathSegmentPublic(path, 0, segment);

  double length = 0.0;
  for (size_t i = 1; i < segment.poses.size(); ++i) {
    double dx = segment.poses[i].pose.position.x - segment.poses[i - 1].pose.position.x;
    double dy = segment.poses[i].pose.position.y - segment.poses[i - 1].pose.position.y;
    length += std::hypot(dx, dy);
  }
  EXPECT_LE(length, layer_->getLookAhead() + 0.5);
}

// ============================================================================
// WALL POLYGON TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, WallPolygons) {
  auto path = createPath(false, 50);
  WallPolygons walls;
  layer_->getWallPolygonsPublic(path, walls);

  EXPECT_FALSE(walls.isEmpty());
  EXPECT_EQ(walls.left_outer.size(), walls.right_outer.size());
}

TEST_F(BoundedTrackingErrorLayerTest, WallPolygonsCurved) {
  auto path = createPath(true, 50);
  WallPolygons walls;
  layer_->getWallPolygonsPublic(path, walls);

  EXPECT_FALSE(walls.isEmpty());
  for (size_t i = 0; i < walls.perpendiculars.size(); ++i) {
    double perp_mag = std::hypot(walls.perpendiculars[i][0], walls.perpendiculars[i][1]);
    EXPECT_NEAR(perp_mag, 1.0, 0.01);
  }
}

// ============================================================================
// UPDATE COSTS TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsNoData) {
  Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
  EXPECT_EQ(countObstacles(master_grid), 0);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsWithData) {
  auto path = createPath(false, 50);
  auto tracking = createTracking(10, 0.1, 1.0, 0.0);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);
  layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y);
  EXPECT_GT(countObstacles(master_grid), 0);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCostsDisabled) {
  auto path = createPath(false, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(createTracking(10)));
  layer_->deactivate();

  Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);
  layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y);
  EXPECT_EQ(countObstacles(master_grid), 0);
  layer_->activate();
}

// ============================================================================
// PARAMETER VALIDATION TESTS (CONSOLIDATED)
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, ValidateParameters) {
  testParamValidation("look_ahead",
    rclcpp::Parameter("test_layer.look_ahead", 2.5),
    rclcpp::Parameter("test_layer.look_ahead", -1.0),
    "look_ahead must be positive");

  testParamValidation("corridor_width",
    rclcpp::Parameter("test_layer.corridor_width", 3.0),
    rclcpp::Parameter("test_layer.corridor_width", 0.0),
    "corridor_width must be positive");

  testParamValidation("step",
    rclcpp::Parameter("test_layer.step", 10),
    rclcpp::Parameter("test_layer.step", 0),
    "step must be greater than zero");

  testParamValidation("wall_thickness",
    rclcpp::Parameter("test_layer.wall_thickness", 2),
    rclcpp::Parameter("test_layer.wall_thickness", -1),
    "wall_thickness must be greater than zero");

  testParamValidation("path_segment_resolution",
    rclcpp::Parameter("test_layer.path_segment_resolution", 5),
    rclcpp::Parameter("test_layer.path_segment_resolution", 0),
    "path_segment_resolution must be at least 1");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateCorridorCost) {
  auto result = layer_->validateParameterUpdatesCallback(
    {rclcpp::Parameter("test_layer.corridor_cost", 200)});
  EXPECT_TRUE(result.successful);

  result = layer_->validateParameterUpdatesCallback(
    {rclcpp::Parameter("test_layer.corridor_cost", 300)});
  EXPECT_FALSE(result.successful);
}

// ============================================================================
// PARAMETER UPDATE TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, UpdateParameters) {
  double orig = layer_->getLookAhead();
  layer_->updateParametersCallback({rclcpp::Parameter("test_layer.look_ahead", 4.0)});
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 4.0);
  EXPECT_NE(layer_->getLookAhead(), orig);

  layer_->updateParametersCallback({rclcpp::Parameter("test_layer.corridor_width", 3.5)});
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 3.5);

  layer_->updateParametersCallback({rclcpp::Parameter("test_layer.step", 8)});
  EXPECT_EQ(layer_->getStep(), 8);
  EXPECT_EQ(layer_->getStepSize(), 8u);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateMultipleParameters) {
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("test_layer.look_ahead", 2.5),
    rclcpp::Parameter("test_layer.corridor_width", 2.5),
    rclcpp::Parameter("test_layer.step", 7)
  };
  layer_->updateParametersCallback(params);

  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 2.5);
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 2.5);
  EXPECT_EQ(layer_->getStep(), 7);
}

// ============================================================================
// INTEGRATION TESTS
// ============================================================================

TEST_F(BoundedTrackingErrorLayerTest, IntegrationFullPipeline) {
  auto goal = createGoal(5.0, 0.0);
  layer_->goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(goal));

  auto path = createPath(false, 50);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  auto tracking = createTracking(10, 0.1, 1.0, 0.0);
  layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(tracking));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);
  EXPECT_NO_THROW(layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y));
  EXPECT_GT(countObstacles(master_grid), 0);
}

TEST_F(BoundedTrackingErrorLayerTest, IntegrationMovingRobot) {
  auto path = createPath(false, 100, 3.0);
  layer_->pathCallback(std::make_shared<nav_msgs::msg::Path>(path));

  Costmap2D master_grid(TEST_SIZE_X, TEST_SIZE_Y, TEST_RESOLUTION, TEST_ORIGIN_X, TEST_ORIGIN_Y);
  int successful = 0;

  for (int idx : {10, 25, 40, 55}) {
    layer_->trackingCallback(std::make_shared<nav2_msgs::msg::TrackingFeedback>(
      createTracking(idx)));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    master_grid.resetMap(0, 0, TEST_SIZE_X, TEST_SIZE_Y);
    layer_->updateCosts(master_grid, 0, 0, TEST_SIZE_X, TEST_SIZE_Y);
    if (countObstacles(master_grid) > 0) {
      successful++;
    }
  }
  EXPECT_GE(successful, 2);
}

}  // namespace nav2_costmap_2d

int main(int argc, char ** argv)
{
  if (rcutils_logging_set_logger_level(
      "test_bounded_tracking_error_layer", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK)
  {
    std::cerr << "Failed to set logger level" << std::endl;
  }
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
