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

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/bounded_tracking_error_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2::LifecycleNode(name) {}

  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
};

class TestableBoundedTrackingErrorLayer
  : public nav2_costmap_2d::BoundedTrackingErrorLayer
{
public:
  using WallPolygons = BoundedTrackingErrorLayer::WallPolygons;

  void testGetPathSegment(
    const nav_msgs::msg::Path & path, size_t idx,
    nav_msgs::msg::Path & segment)
  {
    getPathSegment(path, idx, segment);
  }

  void testGetWallPolygons(
    const nav_msgs::msg::Path & segment,
    WallPolygons & walls)
  {
    getWallPolygons(segment, walls);
  }

  void testDrawCorridorWalls(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<std::array<double, 2>> & inner_points,
    const std::vector<std::array<double, 2>> & outer_points)
  {
    drawCorridorWalls(master_grid, inner_points, outer_points);
  }

  void testPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    pathCallback(msg);
  }

  void testResetState() {resetState();}

  rcl_interfaces::msg::SetParametersResult testValidateParams(
    const std::vector<rclcpp::Parameter> & params)
  {
    return validateParameterUpdatesCallback(params);
  }

  void testUpdateParams(const std::vector<rclcpp::Parameter> & params)
  {
    updateParametersCallback(params);
  }

  bool & enabledRef() {return enabled_;}
  bool & currentRef() {return current_;}
  std::atomic<uint32_t> & pathIndexRef() {return current_path_index_;}

  void setStepSize(size_t s) {step_size_ = s;}
  void setLookAhead(double v) {look_ahead_ = v;}
  void setCorridorWidth(double v) {corridor_width_ = v;}
  void setWallThickness(int v) {wall_thickness_ = v;}
  void setCorridorCost(unsigned char v) {corridor_cost_ = v;}
  void setResolution(double v) {resolution_ = v;}
  void setCostmapFrame(const std::string & f) {costmap_frame_ = f;}
  void setFillOutsideCorridor(bool v) {fill_outside_corridor_ = v;}
  void setRobotBaseFrame(const std::string & f) {robot_base_frame_ = f;}

  void testSaveCorridorInterior(
    nav2_costmap_2d::Costmap2D & master_grid,
    const WallPolygons & walls,
    bool accumulate = false)
  {
    saveCorridorInterior(master_grid, walls, accumulate);
  }

  void testMarkCircleAsInterior(
    nav2_costmap_2d::Costmap2D & master_grid,
    int cx, int cy, int r_sq)
  {
    markCircleAsInterior(master_grid, cx, cy, r_sq);
  }

  void testFillOutsideCorridor(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
  {
    fillOutsideCorridor(master_grid, min_i, min_j, max_i, max_j);
  }

  bool isInterior(unsigned int flat_idx) const
  {
    return flat_idx < corridor_index_set_.size() && corridor_index_set_[flat_idx];
  }

  size_t getStepSize() const {return step_size_;}
  double getLookAhead() const {return look_ahead_;}
  double getCorridorWidth() const {return corridor_width_;}
  int getWallThickness() const {return wall_thickness_;}
  unsigned char getCorridorCost() const {return corridor_cost_;}
  double getResolution() const {return resolution_;}
  const std::string & getCostmapFrame() const {return costmap_frame_;}
  bool getFillOutsideCorridor() const {return fill_outside_corridor_;}
};

// Shared preamble for updateCosts integration tests: enables the layer, sets the
// robot frame, calls matchSize, and resets the costmap to FREE_SPACE.
static void prepareForUpdateCosts(
  TestableBoundedTrackingErrorLayer * layer,
  nav2_costmap_2d::Costmap2D * costmap)
{
  layer->matchSize();
  layer->enabledRef() = true;
  layer->setRobotBaseFrame("base_link");
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
}

static geometry_msgs::msg::PoseStamped makePose(
  double x, double y,
  const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;
  ps.pose.orientation.w = 1.0;
  return ps;
}

// dir_x / dir_y form a direction vector (need not be unit length — normalised internally).
static nav_msgs::msg::Path makeStraightPath(
  double x0, double y0,
  double dir_x, double dir_y,
  size_t num_poses, double spacing,
  const std::string & frame = "map")
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  double len = std::hypot(dir_x, dir_y);
  double ux = dir_x / len;
  double uy = dir_y / len;
  for (size_t i = 0; i < num_poses; ++i) {
    path.poses.push_back(
      makePose(x0 + ux * spacing * i, y0 + uy * spacing * i, frame));
  }
  return path;
}

// num_poses must be >= 2 (start and end point at minimum).
static nav_msgs::msg::Path makeCircularPath(
  double cx, double cy, double radius,
  double start_angle, double end_angle,
  size_t num_poses,
  const std::string & frame = "map")
{
  assert(num_poses >= 2 && "makeCircularPath requires at least 2 poses");
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (size_t i = 0; i < num_poses; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(num_poses - 1);
    double angle = start_angle + t * (end_angle - start_angle);
    path.poses.push_back(
      makePose(cx + radius * std::cos(angle), cy + radius * std::sin(angle), frame));
  }
  return path;
}

static nav_msgs::msg::Path makeRandomPath(
  double x0, double y0, size_t num_poses,
  double step = 0.15,
  const std::string & frame = "map")
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  uint32_t seed = 0xdeadbeef;
  // Numerical Recipes 32-bit LCG: multiplier=1664525, increment=1013904223
  auto lcg = [&]() -> double {
      seed = seed * 1664525u + 1013904223u;
      return (static_cast<double>(seed & 0xFFFF) / 65535.0) * 2.0 - 1.0;
    };
  double x = x0, y = y0;
  for (size_t i = 0; i < num_poses; ++i) {
    path.poses.push_back(makePose(x, y, frame));
    // x advances by step ± 0.04, so the path is not strictly monotone in x.
    x += step + lcg() * 0.04;
    y += lcg() * 0.04;
  }
  return path;
}

class BoundedTrackingErrorLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<TestLifecycleNode>("bte_test_node");
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layers_->resizeMap(100, 100, 0.05, 0.0, 0.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    layer_ = std::make_shared<TestableBoundedTrackingErrorLayer>();
    layer_->initialize(layers_.get(), "bte_layer", tf_buffer_.get(), node_, nullptr);
  }

  void TearDown() override
  {
    layer_->deactivate();
    layer_.reset();
    layers_.reset();
  }

  std::shared_ptr<TestLifecycleNode> node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<TestableBoundedTrackingErrorLayer> layer_;
};

TEST(WallPolygonsTest, testClearAndReserve)
{
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  walls.left_inner.push_back({1.0, 2.0});
  walls.left_outer.push_back({3.0, 4.0});
  walls.right_inner.push_back({5.0, 6.0});
  walls.right_outer.push_back({7.0, 8.0});

  walls.clearAndReserve(10);
  EXPECT_TRUE(walls.left_inner.empty());
  EXPECT_TRUE(walls.left_outer.empty());
  EXPECT_TRUE(walls.right_inner.empty());
  EXPECT_TRUE(walls.right_outer.empty());
  EXPECT_GE(walls.left_inner.capacity(), 10u);
  EXPECT_GE(walls.left_outer.capacity(), 10u);
  EXPECT_GE(walls.right_inner.capacity(), 10u);
  EXPECT_GE(walls.right_outer.capacity(), 10u);
}

TEST_F(BoundedTrackingErrorLayerTest, testDefaultParameterValues)
{
  EXPECT_EQ(layer_->getStepSize(), 10u);
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 2.5);
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 2.0);
  EXPECT_EQ(layer_->getWallThickness(), 1);
  EXPECT_EQ(layer_->getCorridorCost(), 190);
  EXPECT_TRUE(layer_->enabledRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testIsClearable)
{
  EXPECT_TRUE(layer_->isClearable());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentEmptyPath)
{
  nav_msgs::msg::Path empty_path;
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(empty_path, 0, segment);
  EXPECT_TRUE(segment.poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentIndexBeyondPath)
{
  auto path = makeStraightPath(0, 0, 1, 0, 5, 0.1);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 100, segment);
  EXPECT_TRUE(segment.poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentSinglePoseAtEnd)
{
  auto path = makeStraightPath(0, 0, 1, 0, 5, 0.1);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 4, segment);
  EXPECT_EQ(segment.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(segment.poses[0].pose.position.x, path.poses[4].pose.position.x);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentLookAheadExceedsPath)
{
  // 25 poses at 0.1 m spacing = 2.4 m total, just under look_ahead=2.5 m.
  const size_t num_poses = 25;
  const double spacing = 0.1;
  auto path = makeStraightPath(0, 0, 1, 0, num_poses, spacing);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 0, segment);
  EXPECT_EQ(segment.poses.size(), num_poses);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentIncludesBoundaryPose)
{
  // loop crosses threshold at step 3 (dist=0.3 >= look_ahead=0.3); the boundary
  // pose is included so getWallPolygons receives a complete segment.
  layer_->setLookAhead(0.3);
  auto path = makeStraightPath(0, 0, 1, 0, 10, 0.1);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 0, segment);
  ASSERT_EQ(segment.poses.size(), 4u);
  EXPECT_NEAR(segment.poses.front().pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(segment.poses.back().pose.position.x, 0.3, 1e-6);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentRandomPathRespectLookAhead)
{
  auto path = makeRandomPath(1.0, 1.0, 60);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 0, segment);
  ASSERT_GT(segment.poses.size(), 0u);
  EXPECT_DOUBLE_EQ(segment.poses.front().pose.position.x, path.poses[0].pose.position.x);
  EXPECT_DOUBLE_EQ(segment.poses.front().pose.position.y, path.poses[0].pose.position.y);
  double dist = 0.0;
  for (size_t i = 0; i + 1 < segment.poses.size(); ++i) {
    double dx = segment.poses[i + 1].pose.position.x - segment.poses[i].pose.position.x;
    double dy = segment.poses[i + 1].pose.position.y - segment.poses[i].pose.position.y;
    dist += std::hypot(dx, dy);
  }
  // Max step in makeRandomPath is base_step + 0.04 = 0.15 + 0.04 = 0.19 m.
  // The segment may overshoot by at most one step beyond look_ahead.
  const double max_step = 0.15 + 0.04;
  EXPECT_LE(dist, layer_->getLookAhead() + max_step);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentRandomPathStartFromMiddle)
{
  auto path = makeRandomPath(0.5, 0.5, 80);
  nav_msgs::msg::Path segment;
  const size_t start = 15;
  layer_->testGetPathSegment(path, start, segment);
  ASSERT_GT(segment.poses.size(), 0u);
  EXPECT_DOUBLE_EQ(
    segment.poses.front().pose.position.x, path.poses[start].pose.position.x);
  EXPECT_DOUBLE_EQ(
    segment.poses.front().pose.position.y, path.poses[start].pose.position.y);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegmentNonUniformSpacing)
{
  // Build a path with alternating large (0.3 m) and small (0.02 m) steps.
  // The accumulated distance must not overshoot look_ahead by more than
  // the largest single step in the path.
  const double large_step = 0.3;
  const double small_step = 0.02;
  const double max_single_step = large_step;

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  double x = 0.0;
  for (int i = 0; i < 30; ++i) {
    path.poses.push_back(makePose(x, 0.0));
    x += (i % 2 == 0) ? large_step : small_step;
  }

  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 0, segment);

  ASSERT_GT(segment.poses.size(), 0u);

  double dist = 0.0;
  for (size_t i = 0; i + 1 < segment.poses.size(); ++i) {
    double dx = segment.poses[i + 1].pose.position.x - segment.poses[i].pose.position.x;
    double dy = segment.poses[i + 1].pose.position.y - segment.poses[i].pose.position.y;
    dist += std::hypot(dx, dy);
  }
  EXPECT_LE(dist, layer_->getLookAhead() + max_single_step);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsEmptySegment)
{
  nav_msgs::msg::Path empty;
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->setResolution(0.05);
  layer_->testGetWallPolygons(empty, walls);
  EXPECT_TRUE(walls.left_inner.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsSinglePoseSegment)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);
  nav_msgs::msg::Path segment;
  segment.header.frame_id = "map";
  segment.poses.push_back(makePose(1.0, 1.0));
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  EXPECT_TRUE(walls.left_inner.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsDegenerateSegmentSkipped)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);
  nav_msgs::msg::Path segment;
  segment.header.frame_id = "map";
  segment.poses.push_back(makePose(1.0, 1.0));
  segment.poses.push_back(makePose(1.0, 1.0));
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  EXPECT_TRUE(walls.left_inner.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsDiagonalPath)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(0, 0, 1, 1, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  EXPECT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 1.0;
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
  EXPECT_NEAR(walls.left_inner[0][0], -inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.left_inner[0][1], inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.right_inner[0][0], inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.right_inner[0][1], -inv_sqrt2 * inner_offset, 1e-6);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsStepSizeSkipsPoses)
{
  layer_->setStepSize(5);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  // With num_poses=21 and step=5, current_index advances as 0,5,10,15 and
  // next_index=20 on the last iteration — producing 4 wall points total.
  // Expected count is derived directly from the setup variables.
  const size_t num_poses = 21;
  const size_t step_size = 5;
  const size_t expected_points = (num_poses - 1) / step_size;
  auto segment = makeStraightPath(0, 0, 1, 0, num_poses, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  EXPECT_EQ(walls.left_inner.size(), expected_points);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsThicknessAddedToOutside)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(3);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(0.0, 0.0, 1, 0, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 1.0;
  const double outer_offset = 1.0 + 3 * 0.05;

  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    EXPECT_NEAR(walls.left_inner[i][1], inner_offset, 1e-6);
    EXPECT_NEAR(walls.right_inner[i][1], -inner_offset, 1e-6);
    EXPECT_NEAR(walls.left_outer[i][1], outer_offset, 1e-6);
    EXPECT_NEAR(walls.right_outer[i][1], -outer_offset, 1e-6);
  }
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsCurvedPathOffsetMagnitudes)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.6);
  layer_->setWallThickness(2);
  layer_->setResolution(0.05);

  auto segment = makeCircularPath(0.0, 0.0, 2.0, 0.0, M_PI / 2.0, 21);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);
  EXPECT_EQ(walls.left_inner.size(), walls.left_outer.size());
  EXPECT_EQ(walls.left_inner.size(), walls.right_inner.size());

  const double inner_offset = 0.3;
  const double outer_offset = 0.3 + 2 * 0.05;

  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    size_t pose_idx = i * layer_->getStepSize();
    double px = segment.poses[pose_idx].pose.position.x;
    double py = segment.poses[pose_idx].pose.position.y;

    double li_dist = std::hypot(walls.left_inner[i][0] - px, walls.left_inner[i][1] - py);
    double lo_dist = std::hypot(walls.left_outer[i][0] - px, walls.left_outer[i][1] - py);
    double ri_dist = std::hypot(walls.right_inner[i][0] - px, walls.right_inner[i][1] - py);
    double ro_dist = std::hypot(walls.right_outer[i][0] - px, walls.right_outer[i][1] - py);

    EXPECT_NEAR(li_dist, inner_offset, 1e-5) << "left inner offset at " << i;
    EXPECT_NEAR(lo_dist, outer_offset, 1e-5) << "left outer offset at " << i;
    EXPECT_NEAR(ri_dist, inner_offset, 1e-5) << "right inner offset at " << i;
    EXPECT_NEAR(ro_dist, outer_offset, 1e-5) << "right outer offset at " << i;

    const double expected_gap = layer_->getWallThickness() * layer_->getResolution();
    EXPECT_NEAR(
      lo_dist - li_dist, expected_gap,
      1e-5) << "left outer-inner gap must equal wall_thickness * resolution at " << i;
    EXPECT_NEAR(
      ro_dist - ri_dist, expected_gap,
      1e-5) << "right outer-inner gap must equal wall_thickness * resolution at " << i;

    double lx = walls.left_inner[i][0] - px;
    double ly = walls.left_inner[i][1] - py;
    double rx = walls.right_inner[i][0] - px;
    double ry = walls.right_inner[i][1] - py;
    EXPECT_LT(lx * rx + ly * ry, 0.0) << "left and right must be on opposite sides";
  }
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsNegativeXDirection)
{
  // Path travels right-to-left (dx=-1, dy=0). The perpendicular "left" of travel
  // is now (0,-1), so left_inner must be at -inner_offset in Y and right_inner
  // at +inner_offset — opposite of the positive-X case.
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(1.0, 0.0, -1, 0, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 1.0;
  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    EXPECT_NEAR(walls.left_inner[i][1], -inner_offset, 1e-6)
      << "left wall must be on negative-Y side when travelling in -X at " << i;
    EXPECT_NEAR(walls.right_inner[i][1], inner_offset, 1e-6)
      << "right wall must be on positive-Y side when travelling in -X at " << i;
  }
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsSharpTurn)
{
  // Path makes a near-180 degree turn: goes right then sharply back left.
  // Verifies no crash and that wall offsets remain correct at every sampled point.
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.6);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  nav_msgs::msg::Path segment;
  segment.header.frame_id = "map";
  // Rightward leg
  for (int i = 0; i <= 5; ++i) {
    segment.poses.push_back(makePose(i * 0.1, 0.0));
  }
  // Sharp turn — step sideways then go back left
  segment.poses.push_back(makePose(0.5, 0.05));
  for (int i = 5; i >= 0; --i) {
    segment.poses.push_back(makePose(i * 0.1, 0.1));
  }

  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  ASSERT_NO_THROW(layer_->testGetWallPolygons(segment, walls));
  ASSERT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 0.3;
  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    size_t pose_idx = i * layer_->getStepSize();
    double px = segment.poses[pose_idx].pose.position.x;
    double py = segment.poses[pose_idx].pose.position.y;
    double li_dist = std::hypot(walls.left_inner[i][0] - px, walls.left_inner[i][1] - py);
    double ri_dist = std::hypot(walls.right_inner[i][0] - px, walls.right_inner[i][1] - py);
    EXPECT_NEAR(li_dist, inner_offset, 1e-5) << "left inner offset at " << i;
    EXPECT_NEAR(ri_dist, inner_offset, 1e-5) << "right inner offset at " << i;
    double lo_dist = std::hypot(walls.left_outer[i][0] - px, walls.left_outer[i][1] - py);
    double ro_dist = std::hypot(walls.right_outer[i][0] - px, walls.right_outer[i][1] - py);
    const double expected_gap = layer_->getWallThickness() * layer_->getResolution();
    EXPECT_NEAR(lo_dist - li_dist, expected_gap, 1e-5) << "left gap at " << i;
    EXPECT_NEAR(ro_dist - ri_dist, expected_gap, 1e-5) << "right gap at " << i;
  }
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsRandomPathWallsInCostmapBounds)
{
  layer_->matchSize();
  layer_->setStepSize(2);
  layer_->setCorridorWidth(0.4);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeRandomPath(1.0, 1.0, 30, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);

  auto * costmap = layers_->getCostmap();
  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    unsigned int mx, my;
    EXPECT_TRUE(costmap->worldToMap(walls.left_inner[i][0], walls.left_inner[i][1], mx, my))
      << "left_inner[" << i << "] out of bounds";
    EXPECT_TRUE(costmap->worldToMap(walls.left_outer[i][0], walls.left_outer[i][1], mx, my))
      << "left_outer[" << i << "] out of bounds";
    EXPECT_TRUE(costmap->worldToMap(walls.right_inner[i][0], walls.right_inner[i][1], mx, my))
      << "right_inner[" << i << "] out of bounds";
    EXPECT_TRUE(costmap->worldToMap(walls.right_outer[i][0], walls.right_outer[i][1], mx, my))
      << "right_outer[" << i << "] out of bounds";
  }
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateBoundsExpandsBounds)
{
  layer_->matchSize();
  layer_->enabledRef() = true;

  double min_x = 1e10, min_y = 1e10, max_x = -1e10, max_y = -1e10;
  double rx = 2.5, ry = 2.5;
  layer_->updateBounds(rx, ry, 0.0, &min_x, &min_y, &max_x, &max_y);

  double margin = layer_->getLookAhead() +
    layer_->getCorridorWidth() * 0.5 +
    layer_->getWallThickness() * layer_->getResolution();

  EXPECT_DOUBLE_EQ(min_x, rx - margin);
  EXPECT_DOUBLE_EQ(max_x, rx + margin);
  EXPECT_DOUBLE_EQ(min_y, ry - margin);
  EXPECT_DOUBLE_EQ(max_y, ry + margin);
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateBoundsDisabledDoesNotExpand)
{
  layer_->enabledRef() = false;

  double min_x = 0.0, min_y = 0.0, max_x = 1.0, max_y = 1.0;
  layer_->updateBounds(0.5, 0.5, 0.0, &min_x, &min_y, &max_x, &max_y);

  EXPECT_DOUBLE_EQ(min_x, 0.0);
  EXPECT_DOUBLE_EQ(max_x, 1.0);
  EXPECT_DOUBLE_EQ(min_y, 0.0);
  EXPECT_DOUBLE_EQ(max_y, 1.0);
}

TEST_F(BoundedTrackingErrorLayerTest, testMatchSizeCachesResolutionAndFrame)
{
  layer_->matchSize();
  EXPECT_DOUBLE_EQ(layer_->getResolution(), 0.05);
  EXPECT_EQ(layer_->getCostmapFrame(), "map");
}

TEST_F(BoundedTrackingErrorLayerTest, testPathCallbackNewStampResetsPathIndex)
{
  // Store a non-zero path index, then deliver a path with a different stamp.
  // The callback must reset current_path_index_ to 0 because it's a new path.
  layer_->pathIndexRef().store(42);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  msg->poses.push_back(makePose(0.0, 0.0));
  layer_->testPathCallback(msg);

  EXPECT_EQ(layer_->pathIndexRef().load(), 0u)
    << "New stamp must reset path index to 0";
}

TEST_F(BoundedTrackingErrorLayerTest, testPathCallbackSameStampPreservesPathIndex)
{
  // Deliver a first path to initialise last_path_ptr_, then advance the index.
  // A second callback with the identical stamp must NOT reset the index —
  // it is a duplicate delivery of the same plan, not a new one.
  auto stamp = node_->now();

  auto msg1 = std::make_shared<nav_msgs::msg::Path>();
  msg1->header.frame_id = "map";
  msg1->header.stamp = stamp;
  msg1->poses.push_back(makePose(0.0, 0.0));
  layer_->testPathCallback(msg1);

  layer_->pathIndexRef().store(15);

  auto msg2 = std::make_shared<nav_msgs::msg::Path>();
  msg2->header.frame_id = "map";
  msg2->header.stamp = stamp;  // identical stamp
  msg2->poses.push_back(makePose(0.0, 0.0));
  layer_->testPathCallback(msg2);

  EXPECT_EQ(layer_->pathIndexRef().load(), 15u)
    << "Same stamp must preserve existing path index";
}

TEST_F(BoundedTrackingErrorLayerTest, testResetClearsState)
{
  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->poses.push_back(makePose(0.0, 0.0));
  layer_->testPathCallback(msg);
  layer_->pathIndexRef().store(42);

  layer_->reset();

  EXPECT_EQ(layer_->pathIndexRef().load(), 0u);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testResetStateDoesNotTouchCurrent)
{
  // resetState() clears path data and path index but must NOT touch current_.
  // reset() is the only caller that should modify current_.
  // This pins the contract so a future merge of the two functions is caught.
  layer_->currentRef() = true;
  layer_->pathIndexRef().store(42);

  layer_->testResetState();

  EXPECT_EQ(layer_->pathIndexRef().load(), 0u);
  EXPECT_TRUE(layer_->currentRef()) << "resetState() must not modify current_";
}

TEST_F(BoundedTrackingErrorLayerTest, testDeactivateAndReactivate)
{
  layer_->activate();
  layer_->deactivate();
  EXPECT_NO_THROW(layer_->activate());
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsRejectNegativeLookAhead)
{
  layer_->activate();
  auto result_negative = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.look_ahead", -1.0)});
  EXPECT_FALSE(result_negative.successful);
  EXPECT_EQ(result_negative.reason, "look_ahead must be positive");

  auto result_zero = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.look_ahead", 0.0)});
  EXPECT_FALSE(result_zero.successful);
  EXPECT_EQ(result_zero.reason, "look_ahead must be positive");
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsRejectNegativeCorridorWidth)
{
  layer_->activate();
  auto result_negative = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_width", -0.5)});
  EXPECT_FALSE(result_negative.successful);
  EXPECT_EQ(result_negative.reason, "corridor_width must be positive");

  auto result_zero = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_width", 0.0)});
  EXPECT_FALSE(result_zero.successful);
  EXPECT_EQ(result_zero.reason, "corridor_width must be positive");
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsRejectZeroStep)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.step", 0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "step must be greater than zero");
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsRejectCorridorCostZero)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.corridor_cost", 0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "corridor_cost must be between 1 and 254");
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsRejectCorridorCostAbove254)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.corridor_cost", 255),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "corridor_cost must be between 1 and 254");
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsCorridorCostBoundariesAccepted)
{
  layer_->activate();
  auto result1 = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_cost", 1)});
  EXPECT_TRUE(result1.successful);
  auto result254 = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_cost", 254)});
  EXPECT_TRUE(result254.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsRejectZeroWallThickness)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.wall_thickness", 0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "wall_thickness must be greater than zero");
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsIgnoreUnrelatedParameter)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("some_other_layer.look_ahead", -99.0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsLookAhead)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.look_ahead", 5.0)});
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 5.0);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsCorridorWidth)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.corridor_width", 3.0)});
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 3.0);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsEnabled)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->enabledRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.enabled", false)});
  EXPECT_FALSE(layer_->enabledRef());
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsStep)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.step", 20)});
  EXPECT_EQ(layer_->getStepSize(), 20u);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsCorridorCost)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.corridor_cost", 250)});
  EXPECT_EQ(layer_->getCorridorCost(), 250);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsWallThickness)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.wall_thickness", 3)});
  EXPECT_EQ(layer_->getWallThickness(), 3);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsSameValueNoCurrentReset)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.look_ahead", layer_->getLookAhead())});
  EXPECT_TRUE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsIgnoreUnrelated)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("other_layer.look_ahead", 99.0)});
  EXPECT_TRUE(layer_->currentRef());
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 2.5);
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsHorizontalPathCellsMarked)
{
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(2);
  layer_->setCorridorCost(190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  auto segment = makeStraightPath(1.0, 2.5, 1, 0, 21, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);
  layer_->testDrawCorridorWalls(*costmap, walls.right_inner, walls.right_outer);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.80, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190) << "Left wall cell must be marked";

  ASSERT_TRUE(costmap->worldToMap(2.0, 2.20, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190) << "Right wall cell must be marked";

  ASSERT_TRUE(costmap->worldToMap(2.0, 2.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Centre of corridor must remain free";
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsVerticalPathCellsMarked)
{
  // Path moves in Y — wall quads are nearly horizontal, meaning all four quad
  // vertices share very few Y rows. This exercises the span-buffer edge case
  // where a thin horizontal quad could collapse and mark nothing.
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(2);
  layer_->setCorridorCost(190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  // Path along Y axis at x=2.5; left wall at x=2.25, right wall at x=2.75
  auto segment = makeStraightPath(2.5, 1.0, 0, 1, 21, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);
  layer_->testDrawCorridorWalls(*costmap, walls.right_inner, walls.right_outer);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(2.25, 1.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190) << "Left wall of vertical path must be marked";

  ASSERT_TRUE(costmap->worldToMap(2.75, 1.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190) << "Right wall of vertical path must be marked";

  ASSERT_TRUE(costmap->worldToMap(2.5, 1.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Centre of vertical corridor must remain free";
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsPreservesHigherCost)
{
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(2);
  layer_->setCorridorCost(190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.80, mx, my));
  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto segment = makeStraightPath(1.0, 2.5, 1, 0, 21, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);
  layer_->testDrawCorridorWalls(*costmap, walls.right_inner, walls.right_outer);

  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "LETHAL_OBSTACLE must not be overwritten by corridor_cost";
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsOverwritesLowerCost)
{
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(2);
  layer_->setCorridorCost(190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.80, mx, my));
  costmap->setCost(mx, my, 50);

  auto segment = makeStraightPath(1.0, 2.5, 1, 0, 21, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);

  EXPECT_EQ(costmap->getCost(mx, my), 190) << "Lower cost must be overwritten";
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsTooFewPointsNoOp)
{
  layer_->matchSize();
  layer_->setCorridorCost(190);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  std::vector<std::array<double, 2>> inner = {{1.0, 1.0}};
  std::vector<std::array<double, 2>> outer = {{1.0, 1.1}};
  layer_->testDrawCorridorWalls(*costmap, inner, outer);

  bool found_marked = false;
  for (unsigned int y = 0; y < costmap->getSizeInCellsY() && !found_marked; ++y) {
    for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
      if (costmap->getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {
        found_marked = true; break;
      }
    }
  }
  EXPECT_FALSE(found_marked) << "Single-point walls should not mark any cells";
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsOutOfBoundsQuadSkipped)
{
  layer_->matchSize();
  layer_->setCorridorCost(190);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  std::vector<std::array<double, 2>> inner = {{10.0, 10.0}, {11.0, 10.0}};
  std::vector<std::array<double, 2>> outer = {{10.0, 10.5}, {11.0, 10.5}};
  layer_->testDrawCorridorWalls(*costmap, inner, outer);

  bool found_marked = false;
  for (unsigned int y = 0; y < costmap->getSizeInCellsY() && !found_marked; ++y) {
    for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
      if (costmap->getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {
        found_marked = true; break;
      }
    }
  }
  EXPECT_FALSE(found_marked) << "Out-of-bounds quads should not mark any cells";
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsQuadGrazingBottomEdgeCellsMarked)
{
  layer_->matchSize();
  layer_->setCorridorCost(200);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  std::vector<std::array<double, 2>> inner = {{1.0, 0.05}, {2.0, 0.05}};
  std::vector<std::array<double, 2>> outer = {{1.0, 0.20}, {2.0, 0.20}};
  layer_->testDrawCorridorWalls(*costmap, inner, outer);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(1.5, 0.10, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 200) << "Cell near bottom edge must be marked";
  ASSERT_TRUE(costmap->worldToMap(1.5, 0.15, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 200) << "Cell near bottom edge must be marked";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsDisabledNoCostmapChanges)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->enabledRef() = false;  // override: this test verifies the disabled-layer early return

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  for (int i = 0; i < 30; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 0.0));
  }
  layer_->testPathCallback(msg);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(1.5, 1.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Disabled layer must not modify costmap";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsNoPathNoCostmapChanges)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(2.5, 2.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "No-path layer must not modify costmap";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsStalePathNoCostmapChanges)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  // Use 5.1 s — just over the 5.0 s staleness threshold — to test the boundary
  // precisely rather than being trivially far from it.
  msg->header.stamp = node_->now() - rclcpp::Duration::from_seconds(5.1);
  for (int i = 0; i < 30; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 0.0));
  }
  layer_->testPathCallback(msg);
  layer_->pathIndexRef().store(7);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  // Costmap must be untouched.
  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(1.5, 0.0, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Stale-path layer must not modify costmap";

  // resetState() must have been called: path index zeroed and path cleared so
  // a second updateCosts also returns early without touching the costmap.
  EXPECT_EQ(layer_->pathIndexRef().load(), 0u)
    << "Stale path must reset path index to zero";

  layer_->updateCosts(*costmap, 0, 0, 100, 100);
  ASSERT_TRUE(costmap->worldToMap(1.5, 0.0, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "After stale reset, second updateCosts must also be a no-op";
}

TEST_F(BoundedTrackingErrorLayerTest, testSaveCorridorInteriorMarksInteriorCells)
{
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.6);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  auto segment = makeStraightPath(1.0, 2.5, 1, 0, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  layer_->testSaveCorridorInterior(*costmap, walls, /*accumulate=*/false);

  unsigned int cx, cy;
  ASSERT_TRUE(costmap->worldToMap(1.5, 2.5, cx, cy));
  const unsigned int flat = cy * costmap->getSizeInCellsX() + cx;
  EXPECT_TRUE(layer_->isInterior(flat))
    << "Centre cell must be recorded as interior";

  unsigned int ox, oy;
  ASSERT_TRUE(costmap->worldToMap(1.5, 4.0, ox, oy));
  const unsigned int flat_out = oy * costmap->getSizeInCellsX() + ox;
  EXPECT_FALSE(layer_->isInterior(flat_out))
    << "Cell far outside corridor must not be interior";
}

TEST_F(BoundedTrackingErrorLayerTest, testSaveCorridorInteriorAccumulateAddsToExisting)
{
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.6);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();

  auto seg1 = makeStraightPath(1.0, 2.5, 1, 0, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls1;
  layer_->testGetWallPolygons(seg1, walls1);
  layer_->testSaveCorridorInterior(*costmap, walls1, /*accumulate=*/false);
  unsigned int cx1, cy1;
  ASSERT_TRUE(costmap->worldToMap(1.5, 2.5, cx1, cy1));
  const unsigned int flat1 = cy1 * costmap->getSizeInCellsX() + cx1;
  ASSERT_TRUE(layer_->isInterior(flat1));

  // Second segment is disjoint (y=3.5); both must be interior after accumulate=true.
  auto seg2 = makeStraightPath(1.0, 3.5, 1, 0, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls2;
  layer_->testGetWallPolygons(seg2, walls2);
  layer_->testSaveCorridorInterior(*costmap, walls2, /*accumulate=*/true);

  unsigned int cx2, cy2;
  ASSERT_TRUE(costmap->worldToMap(1.5, 3.5, cx2, cy2));
  const unsigned int flat2 = cy2 * costmap->getSizeInCellsX() + cx2;
  EXPECT_TRUE(layer_->isInterior(flat2))
    << "Accumulate=true must add new interior cells to the existing set";
  EXPECT_TRUE(layer_->isInterior(flat1))
    << "Accumulate=true must preserve previously recorded interior cells";
}

TEST_F(BoundedTrackingErrorLayerTest, testMarkCircleAsInteriorCellsWithinRadiusAdded)
{
  layer_->matchSize();

  auto * costmap = layers_->getCostmap();

  const int cx = 50, cy = 50;
  const int r = 3;
  layer_->testMarkCircleAsInterior(*costmap, cx, cy, r * r);

  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy <= r * r) {
        unsigned int flat = static_cast<unsigned int>(cy + dy) *
          costmap->getSizeInCellsX() + static_cast<unsigned int>(cx + dx);
        EXPECT_TRUE(layer_->isInterior(flat))
          << "Cell (" << cx + dx << "," << cy + dy << ") must be interior";
      }
    }
  }

  unsigned int flat_out = static_cast<unsigned int>(cy + r + 1) *
    costmap->getSizeInCellsX() + static_cast<unsigned int>(cx);
  EXPECT_FALSE(layer_->isInterior(flat_out))
    << "Cell just outside radius must not be interior";
}

TEST_F(BoundedTrackingErrorLayerTest, testFillOutsideCorridorMarksNonInteriorCells)
{
  layer_->matchSize();
  layer_->setCorridorCost(190);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  layer_->testMarkCircleAsInterior(*costmap, 50, 50, 0);
  layer_->testFillOutsideCorridor(*costmap, 48, 48, 52, 52);

  EXPECT_EQ(costmap->getCost(50, 50), nav2_costmap_2d::FREE_SPACE)
    << "Interior cell must not be marked";

  EXPECT_EQ(costmap->getCost(48, 48), 190)
    << "Non-interior cell in bbox must be marked with corridor_cost";
}

TEST_F(BoundedTrackingErrorLayerTest, testFillOutsideCorridorPreservesHigherCost)
{
  layer_->matchSize();
  layer_->setCorridorCost(190);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  costmap->setCost(48, 48, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->testFillOutsideCorridor(*costmap, 47, 47, 52, 52);

  EXPECT_EQ(costmap->getCost(48, 48), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "LETHAL_OBSTACLE must not be overwritten by fillOutsideCorridor";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsStaleIndexRecoveredSilently)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  for (int i = 0; i < 10; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 0.0));
  }
  layer_->testPathCallback(msg);
  layer_->pathIndexRef().store(999);

  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));
  EXPECT_EQ(layer_->pathIndexRef().load(), 0u)
    << "Out-of-range path index must be silently reset to 0";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsSegmentTooSmallNoCostmapChanges)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  // step_size=5 => min_poses = (5*2)+1 = 11. A 5-pose path is too small.
  layer_->setStepSize(5);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  for (int i = 0; i < 5; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 0.0));
  }
  layer_->testPathCallback(msg);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  // Costmap must be untouched — segment was too small to form a wall
  bool found_marked = false;
  for (unsigned int y = 0; y < costmap->getSizeInCellsY() && !found_marked; ++y) {
    for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
      if (costmap->getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {
        found_marked = true; break;
      }
    }
  }
  EXPECT_FALSE(found_marked) << "Too-small segment must not mark any cells";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsFreshPathMarksWalls)
{
  // Path at 4.9 s old — just under the 5.0 s staleness threshold.
  // updateCosts must proceed and mark corridor walls.
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(2);
  layer_->setCorridorCost(190);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now() - rclcpp::Duration::from_seconds(4.9);
  for (int i = 0; i < 30; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 2.5));
  }
  layer_->testPathCallback(msg);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  // Left wall should be marked at y = 2.5 + corridor_width/2 = 2.75
  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(1.5, 2.80, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190)
    << "Path just under staleness threshold must draw corridor walls";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsFillOutsideCorridorRobotCellAlwaysInterior)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->setFillOutsideCorridor(true);
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(1);
  layer_->setCorridorCost(190);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  for (int i = 0; i < 40; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 3.0));
  }
  layer_->testPathCallback(msg);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  // Robot cell (at origin) must never be marked even though it's off the path
  unsigned int rx, ry;
  ASSERT_TRUE(costmap->worldToMap(0.0, 0.0, rx, ry));
  EXPECT_EQ(costmap->getCost(rx, ry), nav2_costmap_2d::FREE_SPACE)
    << "Robot cell must always remain free regardless of corridor position";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsZeroResolutionEarlyReturn)
{
  // prepareForUpdateCosts calls matchSize(); override resolution_ to 0 afterwards.
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->setResolution(0.0);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  for (int i = 0; i < 30; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 2.5));
  }
  layer_->testPathCallback(msg);

  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));

  // Nothing should be marked — early return before any work is done
  bool found_marked = false;
  for (unsigned int y = 0; y < costmap->getSizeInCellsY() && !found_marked; ++y) {
    for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
      if (costmap->getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {
        found_marked = true;
        break;
      }
    }
  }
  EXPECT_FALSE(found_marked) << "Zero resolution must prevent any costmap writes";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsFillBranchFlushSegmentFires)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->setFillOutsideCorridor(true);
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(1);
  layer_->setCorridorCost(190);
  layer_->setLookAhead(2.5);

  // Path runs through the middle of the costmap (y=2.5) so both corridor sides
  // (y=2.5±0.25) are well within map bounds. Robot is at origin via TF fixture.
  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  for (int i = 0; i < 60; ++i) {
    msg->poses.push_back(makePose(i * 0.05, 2.5));
  }
  layer_->testPathCallback(msg);

  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(0.5, 0.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190)
    << "Cells outside corridor but inside fill bbox must be elevated to corridor_cost";

  ASSERT_TRUE(costmap->worldToMap(0.5, 2.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Cell inside corridor interior must not be penalised by fill";
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsMixedDegenerateAndValidSteps)
{
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.5);
  layer_->setWallThickness(1);
  // resolution = 0.05 from matchSize; identical poses produce norm=0 < resolution → skipped
  nav_msgs::msg::Path segment;
  segment.header.frame_id = "map";
  // Step 0→1: identical poses → norm = 0, must be skipped
  segment.poses.push_back(makePose(1.0, 1.0));
  segment.poses.push_back(makePose(1.0, 1.0));
  // Step 1→2 (current=1, next clamped to 2): valid, norm > resolution, must be emitted
  segment.poses.push_back(makePose(1.5, 1.0));

  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  // The degenerate first step is skipped; the valid step must still produce a point
  EXPECT_GE(walls.left_inner.size(), 1u)
    << "Valid steps must produce wall points even when earlier steps are degenerate";
  EXPECT_EQ(walls.left_inner.size(), walls.right_inner.size());
  EXPECT_EQ(walls.left_outer.size(), walls.right_outer.size());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsFillOutsideCorridor)
{
  layer_->activate();
  layer_->setFillOutsideCorridor(false);
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.fill_outside_corridor", true)});
  EXPECT_TRUE(layer_->getFillOutsideCorridor())
    << "fill_outside_corridor must be updated to true";
  EXPECT_FALSE(layer_->currentRef())
    << "current_ must be reset when fill_outside_corridor changes";
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsSameValueIntegersNoCurrentReset)
{
  layer_->activate();

  // corridor_cost same value
  layer_->currentRef() = true;
  layer_->testUpdateParams(
    {rclcpp::Parameter("bte_layer.corridor_cost",
    static_cast<int>(layer_->getCorridorCost()))});
  EXPECT_TRUE(layer_->currentRef())
    << "Same corridor_cost must not reset current_";

  // wall_thickness same value
  layer_->currentRef() = true;
  layer_->testUpdateParams(
    {rclcpp::Parameter("bte_layer.wall_thickness", layer_->getWallThickness())});
  EXPECT_TRUE(layer_->currentRef())
    << "Same wall_thickness must not reset current_";

  // step same value
  layer_->currentRef() = true;
  layer_->testUpdateParams(
    {rclcpp::Parameter("bte_layer.step", static_cast<int>(layer_->getStepSize()))});
  EXPECT_TRUE(layer_->currentRef())
    << "Same step must not reset current_";
}

// Path starts inside the fill bbox, exits beyond the radius, then re-enters from
// a different Y. flush_segment must fire at both the exit and re-entry boundaries,
// producing two separate interior sub-segments. Cells on the spine of both chunks
// must remain interior; a cell inside the bbox but off both spines must be penalised.
//
// Geometry (robot at origin, fill_radius = 1.0 + 0.2 + 0.05 = 1.25 m):
//   chunk A: x = 0.0..1.0, y = 0.3  — inside radius, flushed at x=1.0
//   gap:     x = 1.3..2.0, y = 0.3  — outside radius, no interior recorded
//   chunk B: x = 1.0..0.0, y = 0.7  — re-enters radius travelling in -X
//
// Both y=0.3 and y=0.7 are well inside the costmap (origin 0,0 size 100x100 res 0.05).
TEST_F(BoundedTrackingErrorLayerTest, testFillBranchPathExitsAndReentersBbox)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->setFillOutsideCorridor(true);
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.4);
  layer_->setWallThickness(1);
  layer_->setCorridorCost(190);
  layer_->setLookAhead(1.0);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();

  // Chunk A: travels +X at y=0.3, stays inside fill radius (x <= 1.0 < 1.25)
  for (int i = 0; i <= 10; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 0.3));
  }
  // Gap: x = 1.3..2.0 — outside fill radius, flush_segment fires at boundary
  for (int i = 13; i <= 20; ++i) {
    msg->poses.push_back(makePose(i * 0.1, 0.3));
  }
  // Chunk B: travels -X at y=0.7, re-enters fill radius (x <= 1.0 < 1.25)
  for (int i = 10; i >= 0; --i) {
    msg->poses.push_back(makePose(i * 0.1, 0.7));
  }

  layer_->testPathCallback(msg);
  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));

  unsigned int mx, my;

  // Mid-spine of chunk A must be interior (not penalised)
  ASSERT_TRUE(costmap->worldToMap(0.5, 0.3, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Spine of first in-bbox chunk must be interior";

  // Mid-spine of chunk B must be interior (not penalised)
  ASSERT_TRUE(costmap->worldToMap(0.5, 0.7, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Spine of second in-bbox chunk must be interior after re-entry";

  // A cell inside the fill bbox but between and off both spines must be penalised
  ASSERT_TRUE(costmap->worldToMap(0.5, 1.1, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190)
    << "Cell inside fill bbox but off both path spines must be penalised";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
