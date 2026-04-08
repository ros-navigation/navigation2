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

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <array>

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
  void testGetPathSegment(
    const nav_msgs::msg::Path & path, size_t idx,
    nav_msgs::msg::Path & segment)
  {
    getPathSegment(path, idx, segment);
  }

  void testGetWallPolygons(
    const nav_msgs::msg::Path & segment,
    nav2_costmap_2d::WallPolygons & walls)
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

  size_t getStepSize() const {return step_size_;}
  double getLookAhead() const {return look_ahead_;}
  double getCorridorWidth() const {return corridor_width_;}
  int getWallThickness() const {return wall_thickness_;}
  unsigned char getCorridorCost() const {return corridor_cost_;}
  double getResolution() const {return resolution_;}
  const std::string & getCostmapFrame() const {return costmap_frame_;}
};

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

static nav_msgs::msg::Path makeStraightPath(
  double x0, double y0,
  double dx, double dy,
  size_t num_poses, double spacing,
  const std::string & frame = "map")
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  double len = std::hypot(dx, dy);
  double ux = dx / len;
  double uy = dy / len;
  for (size_t i = 0; i < num_poses; ++i) {
    path.poses.push_back(
      makePose(x0 + ux * spacing * i, y0 + uy * spacing * i, frame));
  }
  return path;
}

static nav_msgs::msg::Path makeCircularPath(
  double cx, double cy, double radius,
  double start_angle, double end_angle,
  size_t num_poses,
  const std::string & frame = "map")
{
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
  auto lcg = [&]() -> double {
      seed = seed * 1664525u + 1013904223u;
      return (static_cast<double>(seed & 0xFFFF) / 65535.0) * 2.0 - 1.0;
    };
  double x = x0, y = y0;
  for (size_t i = 0; i < num_poses; ++i) {
    path.poses.push_back(makePose(x, y, frame));
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

TEST(WallPolygonsTest, ClearAndReserve)
{
  nav2_costmap_2d::WallPolygons walls;
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

TEST_F(BoundedTrackingErrorLayerTest, DefaultParameterValues)
{
  EXPECT_EQ(layer_->getStepSize(), 10u);
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 2.5);
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 2.0);
  EXPECT_EQ(layer_->getWallThickness(), 1);
  EXPECT_EQ(layer_->getCorridorCost(), 190);
  EXPECT_TRUE(layer_->enabledRef());
}

TEST_F(BoundedTrackingErrorLayerTest, IsClearable)
{
  EXPECT_FALSE(layer_->isClearable());
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_EmptyPath)
{
  nav_msgs::msg::Path empty_path;
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(empty_path, 0, segment);
  EXPECT_TRUE(segment.poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_IndexBeyondPath)
{
  auto path = makeStraightPath(0, 0, 1, 0, 5, 0.1);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 100, segment);
  EXPECT_TRUE(segment.poses.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_SinglePoseAtEnd)
{
  auto path = makeStraightPath(0, 0, 1, 0, 5, 0.1);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 4, segment);
  EXPECT_EQ(segment.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(segment.poses[0].pose.position.x, path.poses[4].pose.position.x);
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_LookAheadExceedsPath)
{
  // 5 poses, total 0.4 m, look_ahead=2.5 m => all 5 poses returned (look_ahead exceeds path)
  auto path = makeStraightPath(0, 0, 1, 0, 5, 0.1);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 0, segment);
  EXPECT_EQ(segment.poses.size(), 5u);
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_TwoPosePath)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0));
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 0, segment);
  EXPECT_EQ(segment.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(segment.poses[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(segment.poses[1].pose.position.x, 1.0);
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_IncludesBoundaryPose)
{
  // With look_ahead=0.3 and spacing=0.1 the loop crosses the threshold at step 3
  // (dist = 0.3 >= 0.3), setting end_index=3. assign([0,4)) gives poses at
  // x=0.0, 0.1, 0.2, 0.3 — the boundary pose at x=0.3 (index 3) is included
  // so that getWallPolygons receives a complete segment with no missing final quad.
  layer_->setLookAhead(0.3);
  auto path = makeStraightPath(0, 0, 1, 0, 10, 0.1);
  nav_msgs::msg::Path segment;
  layer_->testGetPathSegment(path, 0, segment);
  ASSERT_EQ(segment.poses.size(), 4u);
  EXPECT_NEAR(segment.poses.front().pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(segment.poses.back().pose.position.x, 0.3, 1e-6);
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_RandomPath_RespectLookAhead)
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
  EXPECT_LE(dist, layer_->getLookAhead() + 0.5);
}

TEST_F(BoundedTrackingErrorLayerTest, GetPathSegment_RandomPath_StartFromMiddle)
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

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_EmptySegment)
{
  nav_msgs::msg::Path empty;
  nav2_costmap_2d::WallPolygons walls;
  layer_->setResolution(0.05);
  layer_->testGetWallPolygons(empty, walls);
  EXPECT_TRUE(walls.left_inner.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_SinglePoseSegment)
{
  layer_->setStepSize(1);
  layer_->setResolution(0.05);
  nav_msgs::msg::Path segment;
  segment.header.frame_id = "map";
  segment.poses.push_back(makePose(1.0, 1.0));
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  EXPECT_TRUE(walls.left_inner.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_DegenerateSegmentSkipped)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);
  nav_msgs::msg::Path segment;
  segment.header.frame_id = "map";
  segment.poses.push_back(makePose(1.0, 1.0));
  segment.poses.push_back(makePose(1.0, 1.0));
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  EXPECT_TRUE(walls.left_inner.empty());
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_StraightHorizontal)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(0, 0, 1, 0, 11, 0.1);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  EXPECT_EQ(walls.left_inner.size(), walls.right_inner.size());
  EXPECT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 1.0;
  const double outer_offset = 1.0 + 1 * 0.05;

  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    EXPECT_NEAR(walls.left_inner[i][1], inner_offset, 1e-6);
    EXPECT_NEAR(walls.left_outer[i][1], outer_offset, 1e-6);
    EXPECT_NEAR(walls.right_inner[i][1], -inner_offset, 1e-6);
    EXPECT_NEAR(walls.right_outer[i][1], -outer_offset, 1e-6);
  }
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_StraightVertical)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(0, 0, 0, 1, 11, 0.1);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  EXPECT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 1.0;
  const double outer_offset = 1.0 + 0.05;

  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    EXPECT_NEAR(walls.left_inner[i][0], -inner_offset, 1e-6);
    EXPECT_NEAR(walls.left_outer[i][0], -outer_offset, 1e-6);
    EXPECT_NEAR(walls.right_inner[i][0], inner_offset, 1e-6);
    EXPECT_NEAR(walls.right_outer[i][0], outer_offset, 1e-6);
  }
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_DiagonalPath)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(0, 0, 1, 1, 11, 0.1);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  EXPECT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 1.0;
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
  EXPECT_NEAR(walls.left_inner[0][0], -inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.left_inner[0][1], inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.right_inner[0][0], inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.right_inner[0][1], -inv_sqrt2 * inner_offset, 1e-6);
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_StepSizeSkipsPoses)
{
  layer_->setStepSize(5);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  // 21 poses, step=5: samples at 0,5,10,15 only (index 20 == current at last step)
  auto segment = makeStraightPath(0, 0, 1, 0, 21, 0.1);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  EXPECT_EQ(walls.left_inner.size(), 4u);
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_ThicknessAddedToOutside)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(2.0);
  layer_->setWallThickness(3);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(0.0, 0.0, 1, 0, 11, 0.1);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);

  const double inner_offset = 1.0;
  const double outer_offset = 1.0 + 3 * 0.05;
  const double thickness_gap = outer_offset - inner_offset;

  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    EXPECT_NEAR(walls.left_inner[i][1], inner_offset, 1e-6);
    EXPECT_NEAR(walls.right_inner[i][1], -inner_offset, 1e-6);
    EXPECT_NEAR(walls.left_outer[i][1], outer_offset, 1e-6);
    EXPECT_NEAR(walls.right_outer[i][1], -outer_offset, 1e-6);
    EXPECT_NEAR(walls.left_outer[i][1] - walls.left_inner[i][1], thickness_gap, 1e-6);
    EXPECT_NEAR(walls.right_inner[i][1] - walls.right_outer[i][1], thickness_gap, 1e-6);
  }
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_CollinearInnerOuter_CurvedPath)
{
  // For every sampled point, path->inner and path->outer must be parallel (same ray).
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.6);
  layer_->setWallThickness(3);
  layer_->setResolution(0.05);

  auto segment = makeCircularPath(0.0, 0.0, 2.0, 0.0, M_PI, 31);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);

  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    size_t pose_idx = i * layer_->getStepSize();
    double px = segment.poses[pose_idx].pose.position.x;
    double py = segment.poses[pose_idx].pose.position.y;

    double li_x = walls.left_inner[i][0] - px;
    double li_y = walls.left_inner[i][1] - py;
    double lo_x = walls.left_outer[i][0] - px;
    double lo_y = walls.left_outer[i][1] - py;
    EXPECT_NEAR(li_x * lo_y - li_y * lo_x, 0.0, 1e-9) << "left not collinear at " << i;
    EXPECT_GT(li_x * lo_x + li_y * lo_y, 0.0) << "left outer wrong direction at " << i;

    double ri_x = walls.right_inner[i][0] - px;
    double ri_y = walls.right_inner[i][1] - py;
    double ro_x = walls.right_outer[i][0] - px;
    double ro_y = walls.right_outer[i][1] - py;
    EXPECT_NEAR(ri_x * ro_y - ri_y * ro_x, 0.0, 1e-9) << "right not collinear at " << i;
    EXPECT_GT(ri_x * ro_x + ri_y * ro_y, 0.0) << "right outer wrong direction at " << i;
  }
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_CurvedPath_OffsetMagnitudes)
{
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.6);
  layer_->setWallThickness(2);
  layer_->setResolution(0.05);

  auto segment = makeCircularPath(0.0, 0.0, 2.0, 0.0, M_PI / 2.0, 21);
  nav2_costmap_2d::WallPolygons walls;
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

    EXPECT_GT(lo_dist, li_dist - 1e-9) << "left outer must be further than inner";
    EXPECT_GT(ro_dist, ri_dist - 1e-9) << "right outer must be further than inner";

    double lx = walls.left_inner[i][0] - px;
    double ly = walls.left_inner[i][1] - py;
    double rx = walls.right_inner[i][0] - px;
    double ry = walls.right_inner[i][1] - py;
    EXPECT_LT(lx * rx + ly * ry, 0.0) << "left and right must be on opposite sides";
  }
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_RandomPath_SymmetricOffsets)
{
  layer_->setStepSize(2);
  layer_->setCorridorWidth(0.8);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeRandomPath(0.5, 0.5, 40, 0.15);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);
  EXPECT_EQ(walls.left_inner.size(), walls.right_inner.size());
  EXPECT_EQ(walls.left_inner.size(), walls.left_outer.size());

  const double inner_offset = 0.4;
  const double outer_offset = 0.4 + 1 * 0.05;

  for (size_t i = 0; i < walls.left_inner.size(); ++i) {
    size_t pose_idx = i * layer_->getStepSize();
    double px = segment.poses[pose_idx].pose.position.x;
    double py = segment.poses[pose_idx].pose.position.y;

    double li_dist = std::hypot(walls.left_inner[i][0] - px, walls.left_inner[i][1] - py);
    double lo_dist = std::hypot(walls.left_outer[i][0] - px, walls.left_outer[i][1] - py);
    double ri_dist = std::hypot(walls.right_inner[i][0] - px, walls.right_inner[i][1] - py);
    double ro_dist = std::hypot(walls.right_outer[i][0] - px, walls.right_outer[i][1] - py);

    EXPECT_NEAR(li_dist, inner_offset, 1e-5);
    EXPECT_NEAR(lo_dist, outer_offset, 1e-5);
    EXPECT_NEAR(ri_dist, inner_offset, 1e-5);
    EXPECT_NEAR(ro_dist, outer_offset, 1e-5);
    EXPECT_GT(lo_dist, li_dist - 1e-9);
    EXPECT_GT(ro_dist, ri_dist - 1e-9);
  }
}

TEST_F(BoundedTrackingErrorLayerTest, GetWallPolygons_RandomPath_WallsInCostmapBounds)
{
  layer_->matchSize();
  layer_->setStepSize(2);
  layer_->setCorridorWidth(0.4);
  layer_->setWallThickness(1);
  layer_->setResolution(0.05);

  auto segment = makeRandomPath(1.0, 1.0, 30, 0.1);
  nav2_costmap_2d::WallPolygons walls;
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

TEST_F(BoundedTrackingErrorLayerTest, UpdateBounds_ExpandsBounds)
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

TEST_F(BoundedTrackingErrorLayerTest, UpdateBounds_DisabledDoesNotExpand)
{
  layer_->enabledRef() = false;

  double min_x = 0.0, min_y = 0.0, max_x = 1.0, max_y = 1.0;
  layer_->updateBounds(0.5, 0.5, 0.0, &min_x, &min_y, &max_x, &max_y);

  EXPECT_DOUBLE_EQ(min_x, 0.0);
  EXPECT_DOUBLE_EQ(max_x, 1.0);
  EXPECT_DOUBLE_EQ(min_y, 0.0);
  EXPECT_DOUBLE_EQ(max_y, 1.0);
}

TEST_F(BoundedTrackingErrorLayerTest, MatchSizeCachesResolutionAndFrame)
{
  layer_->matchSize();
  EXPECT_DOUBLE_EQ(layer_->getResolution(), 0.05);
  EXPECT_EQ(layer_->getCostmapFrame(), "map");
}

TEST_F(BoundedTrackingErrorLayerTest, ResetClearsState)
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

TEST_F(BoundedTrackingErrorLayerTest, ResetState_DoesNotTouchCurrent)
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

TEST_F(BoundedTrackingErrorLayerTest, ActivateSetsCurrentTrue)
{
  layer_->currentRef() = false;
  layer_->activate();
  EXPECT_TRUE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, DeactivateAndReactivate)
{
  layer_->activate();
  layer_->deactivate();
  EXPECT_NO_THROW(layer_->activate());
  EXPECT_TRUE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_RejectNegativeLookAhead)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.look_ahead", -1.0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "look_ahead must be positive");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_RejectNegativeCorridorWidth)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.corridor_width", -0.5),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "corridor_width must be positive");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_RejectZeroStep)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.step", 0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "step must be greater than zero");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_RejectCorridorCostZero)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.corridor_cost", 0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "corridor_cost must be between 1 and 254");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_RejectCorridorCostAbove254)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.corridor_cost", 255),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_CorridorCostBoundariesAccepted)
{
  layer_->activate();
  auto result1 = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_cost", 1)});
  EXPECT_TRUE(result1.successful);
  auto result254 = layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_cost", 254)});
  EXPECT_TRUE(result254.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_RejectZeroWallThickness)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("bte_layer.wall_thickness", 0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "wall_thickness must be greater than zero");
}

TEST_F(BoundedTrackingErrorLayerTest, ValidateParams_IgnoreUnrelatedParameter)
{
  layer_->activate();
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("some_other_layer.look_ahead", -99.0),
  };
  auto result = layer_->testValidateParams(params);
  EXPECT_TRUE(result.successful);
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_LookAhead)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.look_ahead", 5.0)});
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 5.0);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_CorridorWidth)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.corridor_width", 3.0)});
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 3.0);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_Enabled)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->enabledRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.enabled", false)});
  EXPECT_FALSE(layer_->enabledRef());
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_Step)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.step", 20)});
  EXPECT_EQ(layer_->getStepSize(), 20u);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_CorridorCost)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.corridor_cost", 250)});
  EXPECT_EQ(layer_->getCorridorCost(), 250);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_WallThickness)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.wall_thickness", 3)});
  EXPECT_EQ(layer_->getWallThickness(), 3);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_SameValueNoCurrentReset)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.look_ahead", layer_->getLookAhead())});
  EXPECT_TRUE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateParams_IgnoreUnrelated)
{
  layer_->activate();
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("other_layer.look_ahead", 99.0)});
  EXPECT_TRUE(layer_->currentRef());
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 2.5);
}

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_HorizontalPath_CellsMarked)
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
  nav2_costmap_2d::WallPolygons walls;
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

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_VerticalPath_CellsMarked)
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
  nav2_costmap_2d::WallPolygons walls;
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

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_CurvedPath_CellsMarked)
{
  layer_->matchSize();
  layer_->setStepSize(1);
  layer_->setCorridorWidth(0.4);
  layer_->setWallThickness(2);
  layer_->setCorridorCost(190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  auto segment = makeCircularPath(2.5, 2.5, 1.0, 0.0, M_PI / 2.0, 21);
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);
  layer_->testDrawCorridorWalls(*costmap, walls.right_inner, walls.right_outer);

  bool found_marked = false;
  for (unsigned int y = 0; y < costmap->getSizeInCellsY() && !found_marked; ++y) {
    for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
      if (costmap->getCost(x, y) == 190) {found_marked = true; break;}
    }
  }
  EXPECT_TRUE(found_marked) << "Curved corridor walls should mark cells";

  unsigned int mx, my;
  double mid_x = 2.5 + 1.0 * std::cos(M_PI / 4.0);
  double mid_y = 2.5 + 1.0 * std::sin(M_PI / 4.0);
  ASSERT_TRUE(costmap->worldToMap(mid_x, mid_y, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Path centreline must remain free";
}

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_PreservesHigherCost)
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
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);
  layer_->testDrawCorridorWalls(*costmap, walls.right_inner, walls.right_outer);

  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "LETHAL_OBSTACLE must not be overwritten by corridor_cost";
}

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_OverwritesLowerCost)
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
  nav2_costmap_2d::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);

  EXPECT_EQ(costmap->getCost(mx, my), 190) << "Lower cost must be overwritten";
}

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_TooFewPoints_NoOp)
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

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_OutOfBoundsQuadSkipped)
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

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_QuadGrazingTopEdge_CellsMarked)
{
  layer_->matchSize();
  layer_->setCorridorCost(190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  // Inner at y=4.70, outer at y=4.90 — both within the 5 m costmap bounds
  std::vector<std::array<double, 2>> inner = {{1.0, 4.70}, {2.0, 4.70}};
  std::vector<std::array<double, 2>> outer = {{1.0, 4.90}, {2.0, 4.90}};
  layer_->testDrawCorridorWalls(*costmap, inner, outer);

  unsigned int mx_inner, my_inner, mx_outer, my_outer;
  ASSERT_TRUE(costmap->worldToMap(1.5, 4.75, mx_inner, my_inner));
  ASSERT_TRUE(costmap->worldToMap(1.5, 4.85, mx_outer, my_outer));
  EXPECT_EQ(costmap->getCost(mx_inner, my_inner), 190) << "Inner boundary row must be marked";
  EXPECT_EQ(costmap->getCost(mx_outer, my_outer), 190) << "Outer boundary row must be marked";
}

TEST_F(BoundedTrackingErrorLayerTest, DrawCorridorWalls_QuadGrazingBottomEdge_CellsMarked)
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

TEST_F(BoundedTrackingErrorLayerTest, UpdateCosts_Disabled_NoCostmapChanges)
{
  layer_->matchSize();
  layer_->enabledRef() = false;

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

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

TEST_F(BoundedTrackingErrorLayerTest, UpdateCosts_ZeroResolution_NoCostmapChanges)
{
  layer_->enabledRef() = true;

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  msg->poses.push_back(makePose(0.0, 0.0));
  layer_->testPathCallback(msg);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(0.05, 0.05, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "Zero-resolution layer must not modify costmap";
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCosts_NoPath_NoCostmapChanges)
{
  layer_->matchSize();
  layer_->enabledRef() = true;

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(2.5, 2.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE)
    << "No-path layer must not modify costmap";
}

TEST_F(BoundedTrackingErrorLayerTest, UpdateCosts_StalePath_NoCostmapChanges)
{
  layer_->matchSize();
  layer_->enabledRef() = true;

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now() - rclcpp::Duration::from_seconds(10.0);
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
